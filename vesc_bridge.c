/**
 ******************************************************************************
 * @file    vesc_bridge.c
 * @brief   Реализация транспорт-независимого моста VESC Tool <-> CAN.
 *          См. vesc_bridge.h и BRIDGE_PROTOCOL.md
 * @author  Mechanic
 * @date    01.10.2026
 * @version 1.10
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

/* Максимальная длина hw_name/fw_name в ответе на COMM_FW_VERSION - см.
 * vesc_bridge_handle_fw_version(). Ограничение чисто по размеру локального
 * буфера сборки ответа, не протокола VESC (там просто null-terminated
 * строка произвольной длины) - при необходимости увеличьте вместе с buf[]
 * там же. */
#define VESC_BRIDGE_FW_VERSION_NAME_MAX_LEN   31U

/* Внешнее кадрирование (packet.c) */
#define VESC_BRIDGE_PKT_START_SHORT      2U
#define VESC_BRIDGE_PKT_START_MEDIUM     3U
#define VESC_BRIDGE_PKT_STOP             3U

/* Наибольший CAN ID, который сканирует активный скан шины для COMM_PING_CAN
 * (0..254 включительно, 255 не сканируется - зарезервирован/широковещательный
 * в протоколе VESC, см. BRIDGE_PROTOCOL.md §5 и обоснование прошивки VESC
 * Express, на которую ориентировались при реализации). */
#define VESC_BRIDGE_SCAN_MAX_ID              254U

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
    VESC_Bridge_HwType_t      hw_type;
    uint8_t                   custom_config_num;
    const uint8_t             *uuid12;
    const char                *fw_name;

    /* ---- активный скан шины для COMM_PING_CAN, см. vesc_bridge_handle_ping_can/
     * VESC_Bridge_Tick - независим от регистрации весок в motor_vesc.c, шлёт
     * CAN_PACKET_PING на все кандидаты и собирает CAN_PACKET_PONG сам ---- */
    uint8_t                  scan_active;         /* идёт скан прямо сейчас */
    uint8_t                  scan_all_sent;        /* все PING (0..VESC_BRIDGE_SCAN_MAX_ID) уже отправлены, ждём окно ответов */
    uint8_t                  scan_next_id;          /* следующий кандидат для PING (0..VESC_BRIDGE_SCAN_MAX_ID+1) */
    uint32_t                 scan_generation;       /* инкрементируется в vesc_bridge_start_scan - см. её же комментарий про гонку с Tick() */
    uint32_t                 scan_all_sent_tick;    /* HAL_GetTick() момента отправки последнего PING скана */
    uint32_t                 scan_found_bitmap[8];  /* 256 бит, по одному на каждый возможный CAN ID 0..255 - см. vesc_bridge_send_scan_result */

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

/**
 * @brief Считает CRC16 (XMODEM) для всего буфера целиком.
 * @param buf Буфер данных
 * @param len Длина буфера, байт
 * @return Значение CRC16
 */
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

/**
 * @brief  Заворачивает payload во внешнее кадрирование и отдаёт через
 *         tx_callback - общая точка выхода для форвардинга и локальных команд.
 * @param  br       хэндл моста
 * @param  payload  данные для отправки
 * @param  len      длина данных
 */
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

/**
 * @brief  Обрабатывает COMM_FW_VERSION - best-effort ответ, см.
 *         BRIDGE_PROTOCOL.md §5.
 * @param  br  хэндл моста
 */
static void vesc_bridge_handle_fw_version(VESC_Bridge_t *br)
{
    /* buf[]: 1(cmd)+1(major)+1(minor)+32(hw_name+null)+12(uuid)+1(pairing)+
     * 1(test)+1(hw_type)+1(custom_config_num)+1(phase_filters)+2(qml_hw+qml_app)+
     * 1(nrf)+32(fw_name+null)+4(hw crc) = 91, с запасом до 96.
     * [v1.9->1.10, найдено тестированием с реальным VESC Tool] Поле QML - ДВА
     * байта (qmlHw, qmlApp - см. commands.cpp COMM_FW_VERSION в vesc_tool), а
     * не один, как было изначально (best-effort по неполным данным) - при
     * одном байте все поля ПОСЛЕ него (nrfFlags, fw_name, hw CRC) сдвигались
     * на байт, nrfFlags получал первый символ fw_name. */
    uint8_t  buf[96];
    uint16_t ind = 0U;

    const char *name = (br->hw_name != NULL) ? br->hw_name : "STM32-BRIDGE";
    uint16_t name_len = (uint16_t)strlen(name);
    if (name_len > VESC_BRIDGE_FW_VERSION_NAME_MAX_LEN)
    {
        name_len = VESC_BRIDGE_FW_VERSION_NAME_MAX_LEN; /* защита от переполнения buf[], см. расчёт выше */
    }

    const char *fw_name = (br->fw_name != NULL) ? br->fw_name : "";
    uint16_t fw_name_len = (uint16_t)strlen(fw_name);
    if (fw_name_len > VESC_BRIDGE_FW_VERSION_NAME_MAX_LEN)
    {
        fw_name_len = VESC_BRIDGE_FW_VERSION_NAME_MAX_LEN;
    }

    buf[ind++] = (uint8_t)VESC_BRIDGE_COMM_FW_VERSION;
    buf[ind++] = br->fw_version_major;
    buf[ind++] = br->fw_version_minor;
    memcpy(&buf[ind], name, name_len);
    ind = (uint16_t)(ind + name_len);
    buf[ind++] = 0U;                          /* null-терминатор HW_NAME */
    if (br->uuid12 != NULL)
    {
        memcpy(&buf[ind], br->uuid12, 12U);   /* см. VESC_Bridge_Config_t.uuid12 */
    }
    else
    {
        memset(&buf[ind], 0, 12U);            /* честно нулевой, см. BRIDGE_PROTOCOL.md/vesc_bridge.h */
    }
    ind = (uint16_t)(ind + 12U);
    buf[ind++] = 0U;                          /* pairing_done */
    buf[ind++] = 0U;                          /* test_version */
    buf[ind++] = (uint8_t)br->hw_type;        /* см. VESC_Bridge_HwType_t - CUSTOM_MODULE для хаба */
    buf[ind++] = br->custom_config_num;
    buf[ind++] = 0U;                          /* has_phase_filters */
    buf[ind++] = 0U;                          /* qml_hw */
    buf[ind++] = 0U;                          /* qml_app - см. @note выше, отдельный байт, не один общий "qmlui флаги" */
    buf[ind++] = 0U;                          /* nrf флаги */
    memcpy(&buf[ind], fw_name, fw_name_len);
    ind = (uint16_t)(ind + fw_name_len);
    buf[ind++] = 0U;                          /* null-терминатор FW_NAME */
    buf[ind++] = 0U; buf[ind++] = 0U; buf[ind++] = 0U; buf[ind++] = 0U; /* hw CRC = 0 */

    vesc_bridge_send_framed(br, buf, ind);
}

/**
 * @brief  Запускает (или перезапускает, если уже идёт - "последний запрос
 *         побеждает", как и у форвардинга) активный скан шины для
 *         COMM_PING_CAN - см. §5 BRIDGE_PROTOCOL.md. Сам скан (отправка
 *         CAN_PACKET_PING кандидатам и ожидание CAN_PACKET_PONG) выполняется
 *         пошагово из VESC_Bridge_Tick()/VESC_Bridge_OnCanFrame() - эта
 *         функция только инициализирует состояние, ответ клиенту уйдёт
 *         позже, асинхронно (см. vesc_bridge_send_scan_result).
 *
 * @warning Может вызываться из контекста, отличного от VESC_Bridge_Tick()
 *          (см. VESC_Bridge_FeedBytes) - состояние скана защищено
 *          критической секцией и счётчиком scan_generation, которым
 *          VESC_Bridge_Tick() проверяет, что скан не перезапустился ровно
 *          посреди её собственного шага (та же гонка и то же решение, что и
 *          у forward_phase в vesc_bridge_forward_step(), см. её @warning).
 * @param  br  хэндл моста
 */
static void vesc_bridge_start_scan(VESC_Bridge_t *br)
{
    __disable_irq();
    br->scan_generation++;
    br->scan_next_id  = 0U;
    br->scan_all_sent = 0U;
    memset(br->scan_found_bitmap, 0, sizeof(br->scan_found_bitmap));
    br->scan_active   = 1U;
    __enable_irq();
}

/**
 * @brief  Отмечает ID отозвавшейся вески в битовой карте активного скана
 *         (см. vesc_bridge_start_scan) - вызывается из VESC_Bridge_OnCanFrame
 *         на приём CAN_PACKET_PONG, адресованного этому мосту.
 * @param  br  хэндл моста
 * @param  id  CAN ID отозвавшейся вески (data[0] кадра PONG)
 */
static void vesc_bridge_scan_mark_found(VESC_Bridge_t *br, uint8_t id)
{
    __disable_irq();
    br->scan_found_bitmap[id >> 5U] |= (1UL << (id & 31U));
    __enable_irq();
}

/**
 * @brief  Собирает итоговый ответ COMM_PING_CAN из битовой карты найденных
 *         ID (см. vesc_bridge_start_scan/vesc_bridge_scan_mark_found) и
 *         отправляет его - вызывается по истечении окна ожидания
 *         VESC_BRIDGE_SCAN_SETTLE_MS из VESC_Bridge_Tick().
 * @param  br  хэндл моста
 */
static void vesc_bridge_send_scan_result(VESC_Bridge_t *br)
{
    uint8_t  buf[1U + VESC_BRIDGE_SCAN_MAX_ID + 1U];
    uint16_t ind = 0U;
    buf[ind++] = (uint8_t)VESC_BRIDGE_COMM_PING_CAN;

    for (uint16_t id = 0U; id <= (uint16_t)VESC_BRIDGE_SCAN_MAX_ID; id++)
    {
        if ((br->scan_found_bitmap[id >> 5U] & (1UL << (id & 31U))) != 0UL)
        {
            buf[ind++] = (uint8_t)id;
        }
    }

    vesc_bridge_send_framed(br, buf, ind);
}

/* ========================================================================
 *  Форвардинг на CAN (COMM_FORWARD_CAN) - см. §3-4 BRIDGE_PROTOCOL.md
 * ====================================================================== */

/**
 * @brief  Собирает Extended ID из кода команды и forward_target_id и
 *         отправляет кадр через CANMGR_Send().
 * @param  br    хэндл моста
 * @param  cmd   код команды VESC
 * @param  len   длина данных
 * @param  data  данные кадра
 * @return HAL_OK при успехе, иначе код ошибки CANMGR_Send
 */
static HAL_StatusTypeDef vesc_bridge_send_raw(VESC_Bridge_t *br, VESC_CAN_PacketId_t cmd,
                                               uint8_t len, const uint8_t *data)
{
    uint32_t ext_id = (((uint32_t)cmd) << 8) | (uint32_t)br->forward_target_id;
    return CANMGR_Send(br->bus, ext_id, 1U, data, len);
}

/**
 * @brief  Продвигает автомат форвардинга РОВНО на один кадр (если получилось).
 *
 * @warning [ИСПРАВЛЕНО] Раньше вся функция целиком (включая сам вызов
 *          vesc_bridge_send_raw()/CANMGR_Send()) была обёрнута ОДНИМ
 *          __disable_irq()/__enable_irq() в расчёте защитить и чтение
 *          forward_src перед отправкой, и запись forward_phase/
 *          forward_send_offset ПОСЛЕ неё. Это не работало: __disable_irq()/
 *          __enable_irq() на Cortex-M не считающие (просто выставляют/сбрасывают
 *          PRIMASK), а CANMGR_Send() САМ внутри себя безусловно вызывает
 *          __enable_irq() перед возвратом - то есть уже на момент возврата
 *          из vesc_bridge_send_raw() прерывания фактически разрешены, что бы
 *          ни делал вызывающий код снаружи. В результате запись
 *          br->forward_phase/forward_send_offset ПОСЛЕ отправки реально
 *          выполнялась БЕЗ защиты - ровно та гонка с
 *          VESC_Bridge_FeedBytes()/vesc_bridge_start_forward() из
 *          прерывания, от которой якобы защищались (новый форвардинг мог
 *          выставить свежие forward_phase/forward_src/forward_send_offset, а
 *          этот вызов, доработав ПОСЛЕ прерывания, тут же затирал их
 *          собственным (устаревшим) результатом).
 *
 *          Правильная защита - ДВЕ раздельные критические секции: первая
 *          вокруг чтения forward_src/forward_send_offset для формирования
 *          кадра (снимок состояния до отправки), вторая - вокруг записи
 *          результата (forward_phase/forward_send_offset) ПОСЛЕ того, как
 *          vesc_bridge_send_raw() уже вернула управление (её собственный
 *          __enable_irq() к этому моменту уже отработал, поэтому мы
 *          заново __disable_irq() перед записью результата, а не полагаемся
 *          на то, что прерывания всё ещё выключены с начала функции).
 *
 * @param  br  мост
 * @retval 1, если в этом вызове реально отправили кадр (вызывающему коду
 *         стоит попробовать продолжить - есть шанс, что буфер ещё не
 *         заполнился); 0 - нечего слать (IDLE/WAITING) либо буфер полон.
 */
static uint8_t vesc_bridge_forward_step(VESC_Bridge_t *br)
{
    uint8_t sent = 0U;
    vesc_bridge_fwd_phase_t phase;

    __disable_irq();
    phase = br->forward_phase;
    __enable_irq();

    switch (phase)
    {
        case VESC_BRIDGE_FWD_SHORT:
        {
            uint8_t frame[8];
            uint8_t src_len;
            HAL_StatusTypeDef st;

            __disable_irq();
            frame[0] = br->own_can_id;
            frame[1] = (uint8_t)VESC_BRIDGE_SEND_FLAG;
            memcpy(&frame[2], br->forward_src, br->forward_src_len);
            src_len = br->forward_src_len;
            __enable_irq();

            st = vesc_bridge_send_raw(br, VESC_CAN_PACKET_PROCESS_SHORT_BUFFER,
                                       (uint8_t)(2U + src_len), frame);

            __disable_irq();
            if ((st == HAL_OK) && (br->forward_phase == VESC_BRIDGE_FWD_SHORT))
            {
                br->forward_phase      = VESC_BRIDGE_FWD_WAITING;
                br->forward_start_tick = HAL_GetTick();
                sent = 1U;
            }
            __enable_irq();
            break;
        }

        case VESC_BRIDGE_FWD_CHUNKING:
        {
            uint32_t off;
            uint32_t remaining;
            uint8_t  frame[8];
            uint32_t chunk;
            HAL_StatusTypeDef st;
            uint16_t src_len_snapshot;

            __disable_irq();
            off = br->forward_send_offset;
            src_len_snapshot = br->forward_src_len;
            remaining = (uint32_t)src_len_snapshot - off;
            if (off <= 255U)
            {
                chunk = (remaining < 7U) ? remaining : 7U;
                frame[0] = (uint8_t)off;
                memcpy(&frame[1], &br->forward_src[off], chunk);
            }
            else
            {
                chunk = (remaining < 6U) ? remaining : 6U;
                frame[0] = (uint8_t)(off >> 8);
                frame[1] = (uint8_t)(off & 0xFFU);
                memcpy(&frame[2], &br->forward_src[off], chunk);
            }
            __enable_irq();

            if (off <= 255U)
            {
                st = vesc_bridge_send_raw(br, VESC_CAN_PACKET_FILL_RX_BUFFER, (uint8_t)(1U + chunk), frame);
            }
            else
            {
                st = vesc_bridge_send_raw(br, VESC_CAN_PACKET_FILL_RX_BUFFER_LONG, (uint8_t)(2U + chunk), frame);
            }

            __disable_irq();
            if ((st == HAL_OK) && (br->forward_phase == VESC_BRIDGE_FWD_CHUNKING) &&
                (br->forward_send_offset == (uint16_t)off))
            {
                off += chunk;
                br->forward_send_offset = (uint16_t)off;
                if (off >= br->forward_src_len)
                {
                    br->forward_phase = VESC_BRIDGE_FWD_FINAL;
                }
                sent = 1U;
            }
            /* иначе - буфер полон прямо сейчас, либо форвардинг успел смениться/
             * перезапуститься параллельно (см. @warning выше) - тот же чанк (или
             * уже новый форвардинг) сам подхватится на следующем шаге */
            __enable_irq();
            break;
        }

        case VESC_BRIDGE_FWD_FINAL:
        {
            uint16_t crc;
            uint8_t  frame[6];
            HAL_StatusTypeDef st;

            __disable_irq();
            crc = vesc_bridge_crc16(br->forward_src, br->forward_src_len);
            frame[0] = br->own_can_id;
            frame[1] = (uint8_t)VESC_BRIDGE_SEND_FLAG;
            frame[2] = (uint8_t)(br->forward_src_len >> 8);
            frame[3] = (uint8_t)(br->forward_src_len & 0xFFU);
            frame[4] = (uint8_t)(crc >> 8);
            frame[5] = (uint8_t)(crc & 0xFFU);
            __enable_irq();

            st = vesc_bridge_send_raw(br, VESC_CAN_PACKET_PROCESS_RX_BUFFER, 6U, frame);

            __disable_irq();
            if ((st == HAL_OK) && (br->forward_phase == VESC_BRIDGE_FWD_FINAL))
            {
                br->forward_phase      = VESC_BRIDGE_FWD_WAITING;
                br->forward_start_tick = HAL_GetTick();
                sent = 1U;
            }
            __enable_irq();
            break;
        }

        default:
            break; /* IDLE либо WAITING - активной отправки нет */
    }

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

/**
 * @brief  Разбирает входящий внешний пакет (после успешного CRC) и
 *         обрабатывает известную COMM-команду.
 * @param  br       хэндл моста
 * @param  payload  данные пакета (первый байт - код команды)
 * @param  len      длина payload
 */
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
        vesc_bridge_start_scan(br);
    }
    else
    {
        VESC_Bridge_OnLocalCommand(br, payload, len);
    }
}

/* ========================================================================
 *  Публичный API - создание моста
 * ====================================================================== */

/**
 * @brief  Callback CANMGR_RxCallback_t для 5 точных фильтров моста - тонкая
 *         обёртка, доставляющая кадр в VESC_Bridge_OnCanFrame().
 * @param  bus         шина (не используется)
 * @param  id          extended CAN ID кадра
 * @param  is_extended признак extended-кадра (не используется)
 * @param  data        данные кадра
 * @param  len         длина данных
 * @param  user_ctx    сам мост (VESC_Bridge_t*), передан при регистрации фильтра
 */
static void vesc_bridge_canmgr_rx(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                                   const uint8_t *data, uint8_t len, void *user_ctx)
{
    (void)bus; (void)is_extended;
    VESC_Bridge_OnCanFrame((VESC_Bridge_t *)user_ctx, id, data, len);
}

/** Создаёт мост - см. подробности в vesc_bridge.h. Регистрирует в
 *  can_manager 5 точных фильтров приёма (cmd_id 5/6/7/8/18, каждый на
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
    br->hw_type           = config->hw_type;
    br->custom_config_num = config->custom_config_num;
    br->uuid12            = config->uuid12;
    br->fw_name           = config->fw_name;
    br->rx_state          = VESC_BRIDGE_RX_WAIT_START;
    br->forward_phase     = VESC_BRIDGE_FWD_IDLE;

    /* См. @warning у VESC_Bridge_Init() в vesc_bridge.h - при частичном
     * успехе (часть из 5 фильтров зарегистрирована, следующий отклонён)
     * уже зарегистрированные в can_manager фильтры НЕ отменяются (у
     * can_manager в этой версии нет функции отмены регистрации) - слот
     * пула освобождаем (br->used = 0), функция возвращает NULL. */
    static const VESC_CAN_PacketId_t bridge_cmd_ids[5] = {
        VESC_CAN_PACKET_FILL_RX_BUFFER, VESC_CAN_PACKET_FILL_RX_BUFFER_LONG,
        VESC_CAN_PACKET_PROCESS_RX_BUFFER, VESC_CAN_PACKET_PROCESS_SHORT_BUFFER,
        VESC_CAN_PACKET_PONG, /* см. активный скан шины у COMM_PING_CAN, VESC_Bridge_Tick */
    };
    for (uint32_t i = 0U; i < 5U; i++)
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

/**
 * @brief  Готовит парсер к приёму payload заданной длины, либо сбрасывает
 *         его, если длина некорректна (0 или больше VESC_BRIDGE_MAX_PAYLOAD).
 * @param  br  хэндл моста
 */
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

/**
 * @brief  Продвигает парсер внешнего кадрирования на один байт - см. §2
 *         BRIDGE_PROTOCOL.md.
 * @param  br  хэндл моста
 * @param  b   очередной принятый байт
 */
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

/**
 * @brief  Скармливает мосту входящие байты - см. подробности в vesc_bridge.h.
 * @param  br    мост, полученный из VESC_Bridge_Init()
 * @param  data  входящие байты
 * @param  len   их количество
 */
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

/**
 * @brief  Периодическое обслуживание - см. подробности в vesc_bridge.h.
 * @param  br  мост, полученный из VESC_Bridge_Init()
 */
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

    /* Активный скан шины для COMM_PING_CAN (см. vesc_bridge_start_scan) -
     * дренируем отправку CAN_PACKET_PING пачками по
     * VESC_BRIDGE_SCAN_PINGS_PER_TICK за тик (как и форвардинг выше), затем
     * ждём VESC_BRIDGE_SCAN_SETTLE_MS после последнего PING и шлём итог. */
    for (;;)
    {
        __disable_irq();
        uint8_t  active   = br->scan_active;
        uint8_t  all_sent = br->scan_all_sent;
        uint8_t  next_id  = br->scan_next_id;
        uint32_t gen      = br->scan_generation;
        __enable_irq();

        if ((active == 0U) || (all_sent != 0U))
        {
            break;
        }

        uint8_t sent_this_tick = 0U;
        while ((sent_this_tick < VESC_BRIDGE_SCAN_PINGS_PER_TICK) && (next_id <= (uint8_t)VESC_BRIDGE_SCAN_MAX_ID))
        {
            if (next_id == br->own_can_id)
            {
                next_id++;
                continue;
            }
            uint32_t ext_id          = (((uint32_t)VESC_CAN_PACKET_PING) << 8) | (uint32_t)next_id;
            uint8_t  ping_payload[1] = { br->own_can_id };
            if (CANMGR_Send(br->bus, ext_id, 1U, ping_payload, 1U) != HAL_OK)
            {
                break; /* очередь can_manager занята прямо сейчас - тот же id повторим на следующем тике */
            }
            next_id++;
            sent_this_tick++;
        }

        /* См. @warning у vesc_bridge_start_scan() - пишем результат этого шага,
         * только если скан не перезапустился параллельно (scan_generation
         * совпадает со снимком выше) - иначе устаревший next_id затёр бы
         * свежесброшенное состояние нового скана (та же гонка, что и с
         * forward_phase в vesc_bridge_forward_step(), см. её @warning). */
        __disable_irq();
        if (br->scan_generation == gen)
        {
            br->scan_next_id = next_id;
            if (next_id > (uint8_t)VESC_BRIDGE_SCAN_MAX_ID)
            {
                br->scan_all_sent      = 1U;
                br->scan_all_sent_tick = HAL_GetTick();
            }
        }
        __enable_irq();

        if (sent_this_tick == 0U)
        {
            break; /* буфер can_manager полон прямо сейчас - остаток отправим на следующем VESC_Bridge_Tick() */
        }
    }

    __disable_irq();
    uint8_t  scan_finished  = (uint8_t)((br->scan_active != 0U) && (br->scan_all_sent != 0U) &&
                                         ((HAL_GetTick() - br->scan_all_sent_tick) > VESC_BRIDGE_SCAN_SETTLE_MS));
    uint32_t gen_for_finish = br->scan_generation;
    __enable_irq();

    if (scan_finished != 0U)
    {
        vesc_bridge_send_scan_result(br); /* вне критической секции - зовёт tx_callback, см. её же предупреждение о контексте */
        __disable_irq();
        if (br->scan_generation == gen_for_finish) /* не перезапустился параллельно, пока слали результат - см. выше */
        {
            br->scan_active = 0U;
        }
        __enable_irq();
    }
}

/* ========================================================================
 *  Публичный API - приём ответных кадров от весок по CAN
 * ====================================================================== */

/**
 * @brief  Скармливает мосту один "чужой" CAN-кадр - см. подробности в vesc_bridge.h.
 * @param  br      мост, полученный из VESC_Bridge_Init()
 * @param  ext_id  extended CAN ID кадра
 * @param  data    данные кадра
 * @param  len     длина данных
 */
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

        case VESC_CAN_PACKET_PONG:
        {
            /* Ответ на активный скан шины (см. vesc_bridge_start_scan/
             * VESC_Bridge_Tick) - data[0] = CAN ID отозвавшейся вески (как и
             * в vesc_pong_dispatch_callback мотор_vesc.c, тот же формат
             * PONG). Кадры PONG, пришедшие БЕЗ активного скана (например от
             * чужого PING, не нашего - но фильтр у нас точный на
             * (PONG<<8)|own_can_id, поэтому реально значит "PONG адресован
             * именно нам, но скан уже завершился/не запускался") - честно
             * игнорируем, отвечать нечем и незачем. */
            if ((len >= 1U) && (br->scan_active != 0U))
            {
                vesc_bridge_scan_mark_found(br, data[0]);
            }
            break;
        }

        default:
            break; /* прочие кадры (PING и т.п.) мост не обрабатывает */
    }
}

/* ========================================================================
 *  Публичный API - взаимодействие со штатной отправкой команд
 * ====================================================================== */

/**
 * @brief  Правда ли, что прямо сейчас идёт форвардинг именно этой веске.
 * @param  br       мост, полученный из VESC_Bridge_Init()
 * @param  vesc_id  проверяемый CAN ID вески
 * @return 1, если форвардинг активен именно к этой веске, иначе 0
 */
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

/**
 * @brief  Слабая заглушка: по умолчанию нераспознанные локальные команды
 *         тихо игнорируются (см. §5 BRIDGE_PROTOCOL.md). Переопределите в
 *         своём коде, чтобы отвечать на другие.
 * @param  br       мост, полученный из VESC_Bridge_Init()
 * @param  payload  данные нераспознанной команды
 * @param  len      длина данных
 */
__weak void VESC_Bridge_OnLocalCommand(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    (void)br; (void)payload; (void)len;
}

/**
 * @brief  Заворачивает payload во внешнее кадрирование и отправляет - см.
 *         подробности в vesc_bridge.h.
 * @param  br       мост, полученный из VESC_Bridge_Init()
 * @param  payload  данные ответа
 * @param  len      длина данных
 */
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

/**
 * @brief  Счётчик отвергнутых входящих внешних пакетов.
 * @param  br  мост, полученный из VESC_Bridge_Init()
 * @return количество отвергнутых пакетов
 */
uint32_t VESC_Bridge_GetRxErrorCount(VESC_Bridge_t *br)
{
    return (br != NULL) ? br->rx_error_count : 0U;
}

/**
 * @brief  Счётчик отвергнутых по CRC ответов от весок по CAN.
 * @param  br  мост, полученный из VESC_Bridge_Init()
 * @return количество отвергнутых по CRC ответов
 */
uint32_t VESC_Bridge_GetCanCrcErrorCount(VESC_Bridge_t *br)
{
    return (br != NULL) ? br->can_crc_error_count : 0U;
}
