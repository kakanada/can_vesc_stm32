/**
 ******************************************************************************
 * @file    vesc_bridge.c
 * @brief   Реализация транспорт-независимого моста VESC Tool <-> CAN.
 *          См. vesc_bridge.h и BRIDGE_PROTOCOL.md
 * @author  Mechanic
 * @date    13.09.2026
 * @version 1.7
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include "vesc_bridge.h"
#include <string.h>

/* ========================================================================
 *  Константы протокола (см. BRIDGE_PROTOCOL.md за источниками/обоснованием)
 * ====================================================================== */

/* COMM_PACKET_ID - "внешние" команды VESC Tool, отдельный неймспейс от
 * CAN_PACKET_ID/VESC_CAN_PacketId_t (motor_vesc.h). Только те, что мост
 * реально распознаёт локально. */
#define VESC_BRIDGE_COMM_FW_VERSION      0U
#define VESC_BRIDGE_COMM_FORWARD_CAN     34U
#define VESC_BRIDGE_COMM_PING_CAN        62U

/* Флаг send/commands_send при форвардинге - см. BRIDGE_PROTOCOL.md §4 за
 * подробным обоснованием именно этого значения (дословно как в прошивке
 * VESC для этого же сценария). */
#define VESC_BRIDGE_SEND_FLAG            0U

/* Внешнее кадрирование (packet.c) */
#define VESC_BRIDGE_PKT_START_SHORT      2U
#define VESC_BRIDGE_PKT_START_MEDIUM     3U
#define VESC_BRIDGE_PKT_STOP             3U

/* Окно "считаем вески живой" для COMM_PING_CAN (см. BRIDGE_PROTOCOL.md §5) -
 * внутренняя деталь реализации, наружу не вынесена. */
#define VESC_BRIDGE_PING_ALIVE_TIMEOUT_MS   1000U

/* ========================================================================
 *  Внутреннее состояние
 * ====================================================================== */

typedef enum
{
    VESC_BRIDGE_RX_WAIT_START = 0,
    VESC_BRIDGE_RX_WAIT_LEN_B0,   /* короткий формат - единственный байт длины; средний - старший байт */
    VESC_BRIDGE_RX_WAIT_LEN_B1,   /* только средний формат - младший байт длины */
    VESC_BRIDGE_RX_WAIT_PAYLOAD,
    VESC_BRIDGE_RX_WAIT_CRC_HI,
    VESC_BRIDGE_RX_WAIT_CRC_LO,
    VESC_BRIDGE_RX_WAIT_STOP,
} vesc_bridge_rx_state_t;

/** Состояние исходящего форвардинга (VESC Tool -> veska) - пошаговый
 *  автомат, продвигается и сразу при старте (vesc_bridge_start_forward), и
 *  на каждом VESC_Bridge_Tick(), НИКОГДА не блокируясь - см. vesc_bridge_forward_step. */
typedef enum
{
    VESC_BRIDGE_FWD_IDLE = 0,     /* нет активного форвардинга - можно начать новый */
    VESC_BRIDGE_FWD_SHORT,        /* payload <= 6 байт - один кадр PROCESS_SHORT_BUFFER */
    VESC_BRIDGE_FWD_CHUNKING,     /* идёт отправка FILL_RX_BUFFER[_LONG] */
    VESC_BRIDGE_FWD_FINAL,        /* чанки отправлены, осталось отправить PROCESS_RX_BUFFER */
    VESC_BRIDGE_FWD_WAITING,      /* всё отправлено, ждём ответа вески по CAN */
} vesc_bridge_fwd_phase_t;

struct VESC_Bridge_s
{
    uint8_t                  used;
    CANMGR_Handle_t          *bus;
    uint8_t                   own_can_id;
    VESC_Bridge_TxCallback_t tx_callback;
    uint8_t                   fw_version_major;
    uint8_t                   fw_version_minor;
    const char               *hw_name;

    /* ---- парсер внешнего кадрирования (VESC Tool -> нас), см. §2 BRIDGE_PROTOCOL.md ---- */
    vesc_bridge_rx_state_t rx_state;
    uint8_t                 rx_is_medium;
    uint16_t                rx_expected_len;
    uint16_t                rx_received_len;
    uint16_t                rx_crc_calc;
    uint16_t                rx_crc_recv;
    uint32_t                rx_last_activity_tick;
    uint8_t                 rx_payload[VESC_BRIDGE_MAX_PAYLOAD];

    /* ---- исходящий форвардинг (VESC Tool -> veska), см. §3 BRIDGE_PROTOCOL.md ----
     * forward_src - СОБСТВЕННАЯ копия вложенной команды (не указатель на
     * rx_payload!) - форвардинг продвигается пошагово через несколько
     * вызовов VESC_Bridge_Tick(), а rx_payload может быть перезаписан новым
     * входящим пакетом раньше, чем текущий форвардинг завершится. */
    uint8_t                  forward_src[VESC_BRIDGE_MAX_PAYLOAD];
    uint16_t                 forward_src_len;
    uint16_t                 forward_send_offset;
    uint8_t                  forward_target_id;
    uint32_t                 forward_start_tick;
    vesc_bridge_fwd_phase_t  forward_phase;

    /* ---- пересборка многокадрового CAN-ответа вески-цели, см. §3 BRIDGE_PROTOCOL.md ---- */
    uint8_t                  can_rx_buf[VESC_BRIDGE_MAX_PAYLOAD];

    /* ---- переиспользуемый буфер для сборки исходящего внешнего кадра ---- */
    uint8_t                  tx_frame_buf[VESC_BRIDGE_MAX_PAYLOAD + 8U];

    /* ---- диагностика, см. VESC_Bridge_GetRxErrorCount/GetCanCrcErrorCount ---- */
    uint32_t                 rx_error_count;
    uint32_t                 can_crc_error_count;
};

static VESC_Bridge_t s_pool[VESC_BRIDGE_MAX_INSTANCES];

/* ========================================================================
 *  CRC16 (XMODEM: poly 0x1021, init 0x0000) - побитовая реализация,
 *  математически идентична табличной из прошивки VESC (см. BRIDGE_PROTOCOL.md
 *  §2), но без риска опечатки при переносе 256-байтной таблицы руками.
 * ====================================================================== */

static uint16_t vesc_bridge_crc16_step(uint16_t crc, uint8_t b)
{
    crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)b << 8));
    for (uint8_t i = 0U; i < 8U; i++)
    {
        crc = (uint16_t)(((crc & 0x8000U) != 0U) ? ((uint16_t)(crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1));
    }
    return crc;
}

static uint16_t vesc_bridge_crc16(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0x0000U;
    for (uint32_t i = 0U; i < len; i++)
    {
        crc = vesc_bridge_crc16_step(crc, buf[i]);
    }
    return crc;
}

/* ========================================================================
 *  Отправка внешнего кадра (payload -> packet.c-совместимый байтовый поток)
 * ====================================================================== */

/** Заворачивает payload во внешнее кадрирование и отдаёт через tx_callback -
 *  общая точка выхода и для ответов на форвардинг, и для локальных команд
 *  (COMM_FW_VERSION, COMM_PING_CAN, VESC_Bridge_SendLocalReply). */
static void vesc_bridge_send_framed(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    if (len > VESC_BRIDGE_MAX_PAYLOAD)
    {
        return; /* не должно происходить при штатном использовании - защитная проверка */
    }

    uint8_t  *buf = br->tx_frame_buf;
    uint16_t  ind = 0U;

    if (len <= 255U)
    {
        buf[ind++] = (uint8_t)VESC_BRIDGE_PKT_START_SHORT;
        buf[ind++] = (uint8_t)len;
    }
    else
    {
        buf[ind++] = (uint8_t)VESC_BRIDGE_PKT_START_MEDIUM;
        buf[ind++] = (uint8_t)(len >> 8);
        buf[ind++] = (uint8_t)(len & 0xFFU);
    }

    memcpy(&buf[ind], payload, len);
    ind = (uint16_t)(ind + len);

    uint16_t crc = vesc_bridge_crc16(payload, len);
    buf[ind++] = (uint8_t)(crc >> 8);
    buf[ind++] = (uint8_t)(crc & 0xFFU);
    buf[ind++] = (uint8_t)VESC_BRIDGE_PKT_STOP;

    br->tx_callback(br, buf, ind);
}

/* ========================================================================
 *  Локальные команды (мост отвечает сам, не форвардя на CAN) - см. §5 BRIDGE_PROTOCOL.md
 * ====================================================================== */

/** COMM_FW_VERSION - best-effort ответ, см. честное объяснение в
 *  BRIDGE_PROTOCOL.md §5 (какие поля и почему упрощены). */
static void vesc_bridge_handle_fw_version(VESC_Bridge_t *br)
{
    uint8_t  buf[48];
    uint16_t ind = 0U;
    const char *name = (br->hw_name != NULL) ? br->hw_name : "STM32-BRIDGE";
    uint16_t name_len = (uint16_t)strlen(name);
    if (name_len > 15U)
    {
        name_len = 15U; /* защита от переполнения buf[48], см. расчёт ниже */
    }

    buf[ind++] = (uint8_t)VESC_BRIDGE_COMM_FW_VERSION;
    buf[ind++] = br->fw_version_major;
    buf[ind++] = br->fw_version_minor;
    memcpy(&buf[ind], name, name_len);
    ind = (uint16_t)(ind + name_len);
    buf[ind++] = 0U;                          /* null-терминатор HW_NAME */
    memset(&buf[ind], 0, 12U);                /* "UUID" - честно нулевой, см. BRIDGE_PROTOCOL.md */
    ind = (uint16_t)(ind + 12U);
    buf[ind++] = 0U;                          /* pairing_done */
    buf[ind++] = 0U;                          /* test_version */
    buf[ind++] = 0U;                          /* hw_type (0 = HW_TYPE_VESC) */
    buf[ind++] = 0U;                          /* custom_config_num */
    buf[ind++] = 0U;                          /* has_phase_filters */
    buf[ind++] = 0U;                          /* qmlui флаги */
    buf[ind++] = 0U;                          /* nrf флаги */
    buf[ind++] = 0U;                          /* FW_NAME - пустая строка (только null) */
    buf[ind++] = 0U; buf[ind++] = 0U; buf[ind++] = 0U; buf[ind++] = 0U; /* hw CRC = 0 */

    vesc_bridge_send_framed(br, buf, ind);
}

/** COMM_PING_CAN - осознанно упрощённая реализация: список ЗАРЕГИСТРИРОВАННЫХ
 *  (VESC_CAN_Init) и ЖИВЫХ (VESC_CAN_IsAlive) весок этой шины, БЕЗ живого
 *  пинга всей шины - см. обоснование в BRIDGE_PROTOCOL.md §5. */
static void vesc_bridge_handle_ping_can(VESC_Bridge_t *br)
{
    uint8_t  buf[1U + VESC_CAN_MAX_DEVICES];
    uint16_t ind = 0U;
    buf[ind++] = (uint8_t)VESC_BRIDGE_COMM_PING_CAN;

    VESC_Handle_t *h = NULL;
    while ((h = VESC_CAN_IterateBus(br->bus, h)) != NULL)
    {
        if (VESC_CAN_IsAlive(h, VESC_BRIDGE_PING_ALIVE_TIMEOUT_MS))
        {
            buf[ind++] = h->vesc_id;
        }
    }

    vesc_bridge_send_framed(br, buf, ind);
}

/* ========================================================================
 *  Форвардинг на CAN (COMM_FORWARD_CAN) - см. §3-4 BRIDGE_PROTOCOL.md
 * ====================================================================== */

/** Собирает Extended ID из кода команды и forward_target_id и отправляет
 *  через CANMGR_Send() (было VESC_CAN_SendRawFrame() до миграции на
 *  can_manager - у CANMGR_Send() своя программная очередь, поэтому этот
 *  путь теперь успешно ставит кадр в очередь заметно чаще, чем раньше
 *  ставил в аппаратный буфер напрямую, см. vesc_bridge.h). */
static HAL_StatusTypeDef vesc_bridge_send_raw(VESC_Bridge_t *br, VESC_CAN_PacketId_t cmd,
                                               uint8_t len, const uint8_t *data)
{
    uint32_t ext_id = (((uint32_t)cmd) << 8) | (uint32_t)br->forward_target_id;
    return CANMGR_Send(br->bus, ext_id, 1U, data, len);
}

/**
 * @brief  Продвигает автомат форвардинга РОВНО на один кадр (если получилось).
 *
 * @warning ВСЯ функция целиком выполняется под __disable_irq()/__enable_irq() -
 *          не только сама отправка, но и чтение/запись forward_* состояния.
 *          Это НЕ про быстродействие (сама отправка - пара регистровых
 *          операций, микросекунды) - это единственный способ гарантированно
 *          не допустить повреждения данных в конкретно этом сценарии: если
 *          VESC_Bridge_FeedBytes() вызывается из прерывания (это явно
 *          разрешено - см. vesc_bridge.h) и очередной входящий пакет
 *          завершает НОВЫЙ COMM_FORWARD_CAN ровно в тот момент, когда где-то
 *          в другом контексте (например VESC_Bridge_Tick() из основного
 *          цикла) уже выполняется этот же шаг для СТАРОГО форвардинга - без
 *          единой защиты новый vesc_bridge_start_forward() перезаписал бы
 *          forward_src/forward_send_offset ПРЯМО ПОСЕРЕДИНЕ того, как старый
 *          вызов их читает, отправив в итоге кадр с обрывком чужих данных.
 *          НЕ блокируется, если аппаратный буфер прямо сейчас полон - просто
 *          возвращает 0 (интервал в критической секции в этом случае предельно
 *          короткий - до первого же обращения к регистру занятого буфера).
 *
 * @param  br  мост
 * @retval 1, если в этом вызове реально отправили кадр (вызывающему коду
 *         стоит попробовать продолжить - есть шанс, что буфер ещё не
 *         заполнился); 0 - нечего слать (IDLE/WAITING) либо буфер полон.
 */
static uint8_t vesc_bridge_forward_step(VESC_Bridge_t *br)
{
    uint8_t sent = 0U;

    __disable_irq();

    switch (br->forward_phase)
    {
        case VESC_BRIDGE_FWD_SHORT:
        {
            uint8_t frame[8];
            frame[0] = br->own_can_id;
            frame[1] = (uint8_t)VESC_BRIDGE_SEND_FLAG;
            memcpy(&frame[2], br->forward_src, br->forward_src_len);
            if (vesc_bridge_send_raw(br, VESC_CAN_PACKET_PROCESS_SHORT_BUFFER,
                                      (uint8_t)(2U + br->forward_src_len), frame) == HAL_OK)
            {
                br->forward_phase      = VESC_BRIDGE_FWD_WAITING;
                br->forward_start_tick = HAL_GetTick();
                sent = 1U;
            }
            break;
        }

        case VESC_BRIDGE_FWD_CHUNKING:
        {
            uint32_t off = br->forward_send_offset;
            uint32_t remaining = (uint32_t)br->forward_src_len - off;
            uint8_t  frame[8];
            uint32_t chunk;
            HAL_StatusTypeDef st;

            if (off <= 255U)
            {
                chunk = (remaining < 7U) ? remaining : 7U;
                frame[0] = (uint8_t)off;
                memcpy(&frame[1], &br->forward_src[off], chunk);
                st = vesc_bridge_send_raw(br, VESC_CAN_PACKET_FILL_RX_BUFFER, (uint8_t)(1U + chunk), frame);
            }
            else
            {
                chunk = (remaining < 6U) ? remaining : 6U;
                frame[0] = (uint8_t)(off >> 8);
                frame[1] = (uint8_t)(off & 0xFFU);
                memcpy(&frame[2], &br->forward_src[off], chunk);
                st = vesc_bridge_send_raw(br, VESC_CAN_PACKET_FILL_RX_BUFFER_LONG, (uint8_t)(2U + chunk), frame);
            }

            if (st == HAL_OK)
            {
                off += chunk;
                br->forward_send_offset = (uint16_t)off;
                if (off >= br->forward_src_len)
                {
                    br->forward_phase = VESC_BRIDGE_FWD_FINAL;
                }
                sent = 1U;
            }
            /* иначе - буфер полон прямо сейчас, тот же чанк повторим на следующем шаге */
            break;
        }

        case VESC_BRIDGE_FWD_FINAL:
        {
            uint16_t crc = vesc_bridge_crc16(br->forward_src, br->forward_src_len);
            uint8_t  frame[6];
            frame[0] = br->own_can_id;
            frame[1] = (uint8_t)VESC_BRIDGE_SEND_FLAG;
            frame[2] = (uint8_t)(br->forward_src_len >> 8);
            frame[3] = (uint8_t)(br->forward_src_len & 0xFFU);
            frame[4] = (uint8_t)(crc >> 8);
            frame[5] = (uint8_t)(crc & 0xFFU);
            if (vesc_bridge_send_raw(br, VESC_CAN_PACKET_PROCESS_RX_BUFFER, 6U, frame) == HAL_OK)
            {
                br->forward_phase      = VESC_BRIDGE_FWD_WAITING;
                br->forward_start_tick = HAL_GetTick();
                sent = 1U;
            }
            break;
        }

        default:
            break; /* IDLE либо WAITING - активной отправки нет */
    }

    __enable_irq();
    return sent;
}

/**
 * @brief  Запускает новый форвардинг - см. §7 BRIDGE_PROTOCOL.md про
 *         "последний запрос побеждает", если предыдущий форвардинг ещё не
 *         завершился.
 *
 * @warning Установка нового forward_src/forward_send_offset/forward_phase -
 *          под той же защитой __disable_irq()/__enable_irq(), что и
 *          vesc_bridge_forward_step() (см. предупреждение там же) - той же
 *          природы гонка, только с другой стороны: это ЗДЕСЬ происходит
 *          перезапись состояния, от которой защищается forward_step().
 */
static void vesc_bridge_start_forward(VESC_Bridge_t *br, uint8_t target_id,
                                       const uint8_t *inner, uint16_t inner_len)
{
    if (inner_len > VESC_BRIDGE_MAX_PAYLOAD)
    {
        br->rx_error_count++;
        return;
    }

    __disable_irq();
    memcpy(br->forward_src, inner, inner_len);
    br->forward_src_len     = inner_len;
    br->forward_send_offset = 0U;
    br->forward_target_id   = target_id;
    br->forward_start_tick  = HAL_GetTick();
    br->forward_phase = (inner_len <= 6U) ? VESC_BRIDGE_FWD_SHORT : VESC_BRIDGE_FWD_CHUNKING;
    __enable_irq();

    /* Пробуем продвинуть сразу, не дожидаясь следующего VESC_Bridge_Tick() -
     * в типичном случае (буфер свободен) весь форвардинг короткой команды
     * уйдёт без задержки в один тик. Каждый вызов сам себя защищает - см.
     * предупреждение у vesc_bridge_forward_step(). */
    while (vesc_bridge_forward_step(br) != 0U)
    {
    }
}

/* ========================================================================
 *  Разбор входящего внешнего пакета целиком (после успешного CRC)
 * ====================================================================== */

static void vesc_bridge_handle_payload(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    if (len == 0U)
    {
        return;
    }

    if ((payload[0] == (uint8_t)VESC_BRIDGE_COMM_FORWARD_CAN) && (len >= 2U))
    {
        vesc_bridge_start_forward(br, payload[1], &payload[2], (uint16_t)(len - 2U));
    }
    else if (payload[0] == (uint8_t)VESC_BRIDGE_COMM_FW_VERSION)
    {
        vesc_bridge_handle_fw_version(br);
    }
    else if (payload[0] == (uint8_t)VESC_BRIDGE_COMM_PING_CAN)
    {
        vesc_bridge_handle_ping_can(br);
    }
    else
    {
        VESC_Bridge_OnLocalCommand(br, payload, len);
    }
}

/* ========================================================================
 *  Публичный API - создание моста
 * ====================================================================== */

/** Callback CANMGR_RxCallback_t, зарегистрированный в can_manager для 4
 *  точных (exact-match) фильтров моста (см. VESC_Bridge_Init ниже) - тонкая
 *  обёртка, доставляющая кадр в уже существующий VESC_Bridge_OnCanFrame()
 *  (её собственная сигнатура/логика не изменилась, она и раньше принимала
 *  просто ext_id/data/len - см. vesc_bridge.h). user_ctx - сам мост (VESC_Bridge_t*),
 *  передан как есть при регистрации фильтра. */
static void vesc_bridge_canmgr_rx(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                                   const uint8_t *data, uint8_t len, void *user_ctx)
{
    (void)bus; (void)is_extended;
    VESC_Bridge_OnCanFrame((VESC_Bridge_t *)user_ctx, id, data, len);
}

/** Создаёт мост - см. подробности в vesc_bridge.h. Регистрирует в
 *  can_manager 4 точных фильтра приёма (cmd_id 5/6/7/8, каждый на
 *  (cmd_id<<8)|own_can_id - см. обоснование в BRIDGE_PROTOCOL.md), вместо
 *  того чтобы (как до миграции на can_manager) полагаться на
 *  VESC_CAN_OnForeignFrame() motor_vesc.c. */
VESC_Bridge_t *VESC_Bridge_Init(const VESC_Bridge_Config_t *config)
{
    if ((config == NULL) || (config->bus == NULL) || (config->tx_callback == NULL))
    {
        return NULL;
    }

    VESC_Bridge_t *br = NULL;
    for (uint32_t i = 0U; i < VESC_BRIDGE_MAX_INSTANCES; i++)
    {
        if (!s_pool[i].used)
        {
            br = &s_pool[i];
            break;
        }
    }
    if (br == NULL)
    {
        return NULL; /* исчерпан VESC_BRIDGE_MAX_INSTANCES */
    }

    memset(br, 0, sizeof(*br));
    br->used             = 1U;
    br->bus               = config->bus;
    br->own_can_id        = config->own_can_id;
    br->tx_callback       = config->tx_callback;
    br->fw_version_major  = config->fw_version_major;
    br->fw_version_minor  = config->fw_version_minor;
    br->hw_name           = config->hw_name;
    br->rx_state          = VESC_BRIDGE_RX_WAIT_START;
    br->forward_phase     = VESC_BRIDGE_FWD_IDLE;

    /* См. @warning у VESC_Bridge_Init() в vesc_bridge.h - при частичном
     * успехе (часть из 4 фильтров зарегистрирована, следующий отклонён)
     * уже зарегистрированные в can_manager фильтры НЕ отменяются (у
     * can_manager в этой версии нет функции отмены регистрации) - слот
     * пула освобождаем (br->used = 0), функция возвращает NULL. */
    static const VESC_CAN_PacketId_t bridge_cmd_ids[4] = {
        VESC_CAN_PACKET_FILL_RX_BUFFER, VESC_CAN_PACKET_FILL_RX_BUFFER_LONG,
        VESC_CAN_PACKET_PROCESS_RX_BUFFER, VESC_CAN_PACKET_PROCESS_SHORT_BUFFER,
    };
    for (uint32_t i = 0U; i < 4U; i++)
    {
        uint32_t filter_id = (((uint32_t)bridge_cmd_ids[i]) << 8) | (uint32_t)config->own_can_id;
        if (CANMGR_RegisterFilter(config->bus, filter_id, 0x1FFFFFFFU, 1U,
                                   vesc_bridge_canmgr_rx, br) != CANMGR_REG_OK)
        {
            br->used = 0U;
            return NULL;
        }
    }

    return br;
}

/* ========================================================================
 *  Публичный API - приём байт от клиента (VESC Tool)
 * ====================================================================== */

/** Готовит парсер к приёму payload заданной длины, либо сбрасывает его,
 *  если длина некорректна (0 или больше VESC_BRIDGE_MAX_PAYLOAD). */
static void vesc_bridge_rx_begin_payload(VESC_Bridge_t *br)
{
    if ((br->rx_expected_len == 0U) || (br->rx_expected_len > VESC_BRIDGE_MAX_PAYLOAD))
    {
        br->rx_state = VESC_BRIDGE_RX_WAIT_START;
        br->rx_error_count++;
        return;
    }
    br->rx_received_len = 0U;
    br->rx_crc_calc      = 0U;
    br->rx_state         = VESC_BRIDGE_RX_WAIT_PAYLOAD;
}

/** Продвигает парсер внешнего кадрирования на один байт - см. §2 BRIDGE_PROTOCOL.md. */
static void vesc_bridge_feed_one_byte(VESC_Bridge_t *br, uint8_t b)
{
    br->rx_last_activity_tick = HAL_GetTick();

    switch (br->rx_state)
    {
        case VESC_BRIDGE_RX_WAIT_START:
            if (b == (uint8_t)VESC_BRIDGE_PKT_START_SHORT)
            {
                br->rx_is_medium = 0U;
                br->rx_state     = VESC_BRIDGE_RX_WAIT_LEN_B0;
            }
            else if (b == (uint8_t)VESC_BRIDGE_PKT_START_MEDIUM)
            {
                br->rx_is_medium = 1U;
                br->rx_state     = VESC_BRIDGE_RX_WAIT_LEN_B0;
            }
            /* иначе - мусорный байт между пакетами, просто ждём следующий старт-байт */
            break;

        case VESC_BRIDGE_RX_WAIT_LEN_B0:
            if (br->rx_is_medium != 0U)
            {
                br->rx_expected_len = (uint16_t)((uint16_t)b << 8);
                br->rx_state = VESC_BRIDGE_RX_WAIT_LEN_B1;
            }
            else
            {
                br->rx_expected_len = b;
                vesc_bridge_rx_begin_payload(br);
            }
            break;

        case VESC_BRIDGE_RX_WAIT_LEN_B1:
            br->rx_expected_len = (uint16_t)(br->rx_expected_len | b);
            vesc_bridge_rx_begin_payload(br);
            break;

        case VESC_BRIDGE_RX_WAIT_PAYLOAD:
            br->rx_payload[br->rx_received_len] = b;
            br->rx_crc_calc = vesc_bridge_crc16_step(br->rx_crc_calc, b);
            br->rx_received_len++;
            if (br->rx_received_len >= br->rx_expected_len)
            {
                br->rx_state = VESC_BRIDGE_RX_WAIT_CRC_HI;
            }
            break;

        case VESC_BRIDGE_RX_WAIT_CRC_HI:
            br->rx_crc_recv = (uint16_t)((uint16_t)b << 8);
            br->rx_state    = VESC_BRIDGE_RX_WAIT_CRC_LO;
            break;

        case VESC_BRIDGE_RX_WAIT_CRC_LO:
            br->rx_crc_recv = (uint16_t)(br->rx_crc_recv | b);
            br->rx_state    = VESC_BRIDGE_RX_WAIT_STOP;
            break;

        case VESC_BRIDGE_RX_WAIT_STOP:
            br->rx_state = VESC_BRIDGE_RX_WAIT_START; /* в любом случае возвращаемся к ожиданию следующего пакета */
            if ((b == (uint8_t)VESC_BRIDGE_PKT_STOP) && (br->rx_crc_calc == br->rx_crc_recv))
            {
                vesc_bridge_handle_payload(br, br->rx_payload, br->rx_expected_len);
            }
            else
            {
                br->rx_error_count++;
            }
            break;

        default:
            br->rx_state = VESC_BRIDGE_RX_WAIT_START;
            break;
    }
}

/** Скармливает мосту входящие байты - см. подробности в vesc_bridge.h. */
void VESC_Bridge_FeedBytes(VESC_Bridge_t *br, const uint8_t *data, uint16_t len)
{
    if ((br == NULL) || (data == NULL))
    {
        return;
    }
    for (uint16_t i = 0U; i < len; i++)
    {
        vesc_bridge_feed_one_byte(br, data[i]);
    }
}

/** Периодическое обслуживание - см. подробности в vesc_bridge.h. */
void VESC_Bridge_Tick(VESC_Bridge_t *br)
{
    if (br == NULL)
    {
        return;
    }

    /* Дренируем очередь исходящих CAN-чанков форвардинга, если есть что -
     * без блокировки: останавливаемся, как только аппаратный буфер занят
     * прямо сейчас, продолжим на следующем тике (см. vesc_bridge_forward_step). */
    while (vesc_bridge_forward_step(br) != 0U)
    {
    }

    /* Таймаут ожидания ответа от вески-цели. Проверка и запись - под одной
     * защитой (см. комментарии про критическую секцию в vesc_bridge_forward_step
     * выше) - иначе новый форвардинг, стартовавший ИМЕННО в этот момент из
     * другого контекста (VESC_Bridge_FeedBytes), рисковал бы быть тут же
     * затёртым обратно в IDLE. */
    __disable_irq();
    if ((br->forward_phase == VESC_BRIDGE_FWD_WAITING) &&
        ((HAL_GetTick() - br->forward_start_tick) > VESC_BRIDGE_FORWARD_TIMEOUT_MS))
    {
        br->forward_phase = VESC_BRIDGE_FWD_IDLE;
    }
    __enable_irq();

    /* Таймаут незавершённого входящего внешнего пакета (например оборвалась
     * TCP-сессия/потерялся байт по UART посреди пакета) - не ждём остаток
     * вечно, сбрасываем парсер. */
    if ((br->rx_state != VESC_BRIDGE_RX_WAIT_START) &&
        ((HAL_GetTick() - br->rx_last_activity_tick) > VESC_BRIDGE_RX_TIMEOUT_MS))
    {
        br->rx_state = VESC_BRIDGE_RX_WAIT_START;
        br->rx_error_count++;
    }
}

/* ========================================================================
 *  Публичный API - приём ответных кадров от весок по CAN
 * ====================================================================== */

/** Скармливает мосту один "чужой" CAN-кадр - см. подробности в vesc_bridge.h. */
void VESC_Bridge_OnCanFrame(VESC_Bridge_t *br, uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    if ((br == NULL) || (data == NULL))
    {
        return;
    }

    const uint8_t id  = (uint8_t)(ext_id & 0xFFU);
    const uint8_t cmd = (uint8_t)((ext_id >> 8) & 0xFFU);

    if (id != br->own_can_id)
    {
        /* После миграции на can_manager фильтры моста УЖЕ точные
         * (cmd_id<<8)|own_can_id (см. VESC_Bridge_Init) - сюда в норме не
         * должен попасть кадр с чужим id вообще. Проверка оставлена как
         * defense-in-depth (на случай прямого вызова этой функции из
         * ручного/тестового кода, минуя can_manager). */
        return;
    }

    switch ((VESC_CAN_PacketId_t)cmd)
    {
        case VESC_CAN_PACKET_FILL_RX_BUFFER:
        {
            if (len < 1U)
            {
                break;
            }
            uint32_t offset = data[0];
            uint32_t chunk  = (uint32_t)len - 1U;
            if ((offset + chunk) <= (uint32_t)VESC_BRIDGE_MAX_PAYLOAD)
            {
                memcpy(&br->can_rx_buf[offset], &data[1], chunk);
            }
            break;
        }

        case VESC_CAN_PACKET_FILL_RX_BUFFER_LONG:
        {
            if (len < 2U)
            {
                break;
            }
            uint32_t offset = ((uint32_t)data[0] << 8) | (uint32_t)data[1];
            uint32_t chunk  = (uint32_t)len - 2U;
            if ((offset + chunk) <= (uint32_t)VESC_BRIDGE_MAX_PAYLOAD)
            {
                memcpy(&br->can_rx_buf[offset], &data[2], chunk);
            }
            break;
        }

        case VESC_CAN_PACKET_PROCESS_SHORT_BUFFER:
        {
            if (len < 2U)
            {
                break;
            }
            /* Короткий полный ответ - без пересборки, [0]=id отправителя
             * (не нужен нам), [1]=send_flag (не нужен), [2:]=ответ как есть. */
            vesc_bridge_send_framed(br, &data[2], (uint16_t)(len - 2U));
            br->forward_phase = VESC_BRIDGE_FWD_IDLE; /* обмен завершён */
            break;
        }

        case VESC_CAN_PACKET_PROCESS_RX_BUFFER:
        {
            if (len < 6U)
            {
                break;
            }
            uint16_t rxlen     = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
            uint16_t crc_recv  = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);

            if (rxlen <= VESC_BRIDGE_MAX_PAYLOAD)
            {
                uint16_t crc_calc = vesc_bridge_crc16(br->can_rx_buf, rxlen);
                if (crc_calc == crc_recv)
                {
                    vesc_bridge_send_framed(br, br->can_rx_buf, rxlen);
                }
                else
                {
                    br->can_crc_error_count++;
                }
            }
            else
            {
                br->can_crc_error_count++;
            }
            br->forward_phase = VESC_BRIDGE_FWD_IDLE; /* обмен завершён (успешно или нет) */
            break;
        }

        default:
            break; /* прочие кадры (PING/PONG и т.п.) мост не обрабатывает */
    }
}

/* ========================================================================
 *  Публичный API - взаимодействие со штатной отправкой команд
 * ====================================================================== */

/** Правда ли, что прямо сейчас идёт форвардинг именно этой веске - см.
 *  подробности в vesc_bridge.h. */
uint8_t VESC_Bridge_IsTargetActive(VESC_Bridge_t *br, uint8_t vesc_id)
{
    if ((br == NULL) || (br->forward_phase == VESC_BRIDGE_FWD_IDLE))
    {
        return 0U;
    }
    return (br->forward_target_id == vesc_id) ? 1U : 0U;
}

/* ========================================================================
 *  Точка расширения: нераспознанные локальные команды (слабая заглушка)
 * ====================================================================== */

/** Слабая заглушка: по умолчанию нераспознанные локальные команды тихо
 *  игнорируются (см. §5 BRIDGE_PROTOCOL.md - в т.ч. почему это безопасно для
 *  COMM_ALIVE). Переопределите в своём коде, чтобы отвечать на другие. */
__weak void VESC_Bridge_OnLocalCommand(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    (void)br; (void)payload; (void)len;
}

/** Заворачивает payload во внешнее кадрирование и отправляет - см.
 *  подробности в vesc_bridge.h. */
void VESC_Bridge_SendLocalReply(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    if ((br == NULL) || (payload == NULL) || (len > VESC_BRIDGE_MAX_PAYLOAD))
    {
        return;
    }
    vesc_bridge_send_framed(br, payload, len);
}

/* ========================================================================
 *  Диагностика
 * ====================================================================== */

/** Счётчик отвергнутых входящих внешних пакетов - см. подробности в vesc_bridge.h. */
uint32_t VESC_Bridge_GetRxErrorCount(VESC_Bridge_t *br)
{
    return (br != NULL) ? br->rx_error_count : 0U;
}

/** Счётчик отвергнутых по CRC ответов от весок по CAN - см. подробности в vesc_bridge.h. */
uint32_t VESC_Bridge_GetCanCrcErrorCount(VESC_Bridge_t *br)
{
    return (br != NULL) ? br->can_crc_error_count : 0U;
}
