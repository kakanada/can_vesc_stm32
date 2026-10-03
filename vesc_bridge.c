/**
 ******************************************************************************
 * @file    vesc_bridge.c
 * @brief   Реализация транспорт-независимого моста VESC Tool <-> CAN.
 *          См. vesc_bridge.h и BRIDGE_PROTOCOL.md
 * @author  Mechanic
 * @date    03.10.2026
 * @version 1.14
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
#define VESC_BRIDGE_COMM_FW_VERSION       0U
#define VESC_BRIDGE_COMM_CAN_FWD_FRAME    33U
#define VESC_BRIDGE_COMM_FORWARD_CAN      34U
#define VESC_BRIDGE_COMM_PING_CAN         62U
#define VESC_BRIDGE_COMM_GET_QML_UI_APP   118U
#define VESC_BRIDGE_COMM_LISP_READ_CODE   130U

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

/* Размер буфера ресинхронизации (см. vesc_bridge_feed_one_byte): поток короткого
 * кадра БЕЗ ложного старт-байта = 1 (len) + 255 (payload) + 2 (crc) + 1 (stop). */
#define VESC_BRIDGE_RESYNC_BUF_SIZE      259U

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

/**
 * @brief  Состояние исходящего форвардинга (VESC Tool -> veska) - пошаговый
 *  автомат, продвигается и сразу при старте (vesc_bridge_start_forward), и
 *  на каждом VESC_Bridge_Tick(), НИКОГДА не блокируясь - см. vesc_bridge_forward_step.
 *
 * @note   [v1.12, fire-and-forget] До этой версии после отправки последнего
 *         кадра команда переходила в VESC_BRIDGE_FWD_WAITING и следующая
 *         команда из очереди стартовала только после ответа вески-цели либо
 *         таймаута - по образцу настоящей прошивки VESC Express это НЕВЕРНО:
 *         comm_can.c на COMM_FORWARD_CAN делает ровно comm_can_send_buffer()
 *         и возвращается немедленно, не дожидаясь ответа - ответы приходят
 *         полностью асинхронно, отдельными CAN-кадрами, не блокируя
 *         следующий форвардинг. VESC_BRIDGE_FWD_WAITING удалена: как только
 *         последний кадр команды фактически ушёл в can_manager (CANMGR_Send
 *         вернул HAL_OK), форвардинг сразу считается завершённым и из
 *         очереди стартует следующий - см. vesc_bridge_forward_step().
 */
typedef enum
{
    VESC_BRIDGE_FWD_IDLE = 0,     /* нет активного форвардинга - можно начать новый */
    VESC_BRIDGE_FWD_SHORT,        /* payload <= 6 байт - один кадр PROCESS_SHORT_BUFFER */
    VESC_BRIDGE_FWD_CHUNKING,     /* идёт отправка FILL_RX_BUFFER[_LONG] */
    VESC_BRIDGE_FWD_FINAL,        /* чанки отправлены, осталось отправить PROCESS_RX_BUFFER */
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
    uint32_t                rx_timeout_ms;       /* [v1.14] из конфига, 0 = таймаут выключен - см. VESC_Bridge_Config_t.rx_timeout_ms */
    /* [v1.14] ресинхронизация после ошибки CRC/STOP КОРОТКОГО кадра (<=255 байт): уже
     * принятые байты (кроме ложного старт-байта) перепрогоняются через парсер, чтобы
     * потерянная граница не "съела" следующий нормальный пакет - см.
     * vesc_bridge_feed_one_byte()/vesc_bridge_drain_resync(). */
    uint8_t                 rx_resync_buf[VESC_BRIDGE_RESYNC_BUF_SIZE];  /* пишется парсером при ошибке */
    uint8_t                 rx_resync_work[VESC_BRIDGE_RESYNC_BUF_SIZE]; /* рабочая копия на время перепрогона */
    uint16_t                rx_resync_len;
    uint8_t                 rx_payload[VESC_BRIDGE_MAX_PAYLOAD];

    /* ---- исходящий форвардинг (VESC Tool -> veska), см. §3 BRIDGE_PROTOCOL.md ----
     * forward_src - СОБСТВЕННАЯ копия вложенной команды (не указатель на
     * rx_payload!) - форвардинг продвигается пошагово через несколько
     * вызовов VESC_Bridge_Tick(), а rx_payload может быть перезаписан новым
     * входящим пакетом раньше, чем текущий форвардинг завершится.
     * Это состояние ТОЛЬКО активного (прямо сейчас выполняемого) форвардинга -
     * см. fwd_queue ниже (с версии 1.11) для запросов, ожидающих очереди. */
    uint8_t                  forward_src[VESC_BRIDGE_MAX_PAYLOAD];
    uint16_t                 forward_src_len;
    uint16_t                 forward_send_offset;
    uint8_t                  forward_target_id;
    uint32_t                 last_forward_tick;    /* [v1.12] HAL_GetTick() последнего УСПЕШНО отправленного кадра форвардинга (любой фазы) - см. VESC_Bridge_IsTargetActive, заменяет forward_start_tick/WAITING-таймаут */
    vesc_bridge_fwd_phase_t  forward_phase;

    /* ---- [v1.12] очередь форвардинга, см. VESC_BRIDGE_FORWARD_QUEUE_BYTES ----
     * Кольцевой байтовый буфер запросов COMM_FORWARD_CAN, пришедших, пока
     * предыдущий ещё не отправлен (VESC Tool конвейерит запросы - см.
     * BRIDGE_PROTOCOL.md §3). Запись в кольце: [target_id:1][len_hi:1][len_lo:1][payload:len].
     * Переполнение - НОВЫЙ запрос отбрасывается целиком (см.
     * vesc_bridge_enqueue_forward/fwd_queue_overflow_count) - НЕ затирает
     * ничего уже стоящее в очереди (в отличие от v1.11, где переполнение
     * перезаписывало последний поставленный запрос). */
    uint8_t   fwd_queue_buf[VESC_BRIDGE_FORWARD_QUEUE_BYTES];
    uint16_t  fwd_queue_wr;               /* индекс записи (куда писать следующий байт) */
    uint16_t  fwd_queue_rd;               /* индекс чтения (откуда читать следующий байт) */
    uint16_t  fwd_queue_used;             /* сколько байт сейчас реально хранится в кольце */
    uint32_t  fwd_queue_overflow_count;   /* см. VESC_Bridge_GetForwardQueueOverflowCount */

    /* ---- пересборка многокадрового CAN-ответа вески-цели, см. §3 BRIDGE_PROTOCOL.md ---- */
    uint8_t                  can_rx_buf[VESC_BRIDGE_MAX_PAYLOAD];

    /* ---- переиспользуемые буферы для сборки исходящего внешнего кадра ----
     * [v1.13] ДВА отдельных буфера, не один - см. @warning у
     * vesc_bridge_send_framed_buf(): tx_frame_buf - для всего, что НЕ
     * VESC_Bridge_OnCanFrame() (форвардинг результата, локальные команды,
     * VESC_Bridge_SendLocalReply), tx_frame_buf_isr - ТОЛЬКО для
     * VESC_Bridge_OnCanFrame() (CAN RX ISR) - без этого разделения ISR мог
     * перезаписать буфер посреди сборки ответа в другом контексте. */
    uint8_t                  tx_frame_buf[VESC_BRIDGE_MAX_PAYLOAD + 8U];
    uint8_t                  tx_frame_buf_isr[VESC_BRIDGE_MAX_PAYLOAD + 8U];

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
 * @brief  [v1.13] Общая реализация - заворачивает payload во внешнее
 *         кадрирование в ПЕРЕДАННЫЙ буфер и отдаёт через tx_callback.
 *
 * @warning Буфер - ПАРАМЕТР, а не всегда br->tx_frame_buf, именно чтобы
 *          избежать гонки, найденной тестированием на реальном железе: до
 *          этой версии ВСЕ вызовы (и из VESC_Bridge_OnCanFrame() - CAN RX
 *          ISR, см. её @warning в vesc_bridge.h, - и из вызовов, идущих от
 *          VESC_Bridge_FeedBytes() - main-контекст в типичной интеграции, но
 *          формально тоже МОЖЕТ быть прерыванием другого источника, см. её
 *          же документацию) использовали ОДИН общий br->tx_frame_buf. Если
 *          CAN-ответ вески прилетал (прерывание) РОВНО посреди сборки
 *          локального ответа (FW_VERSION/PING_CAN/LISP/QML) в другом
 *          контексте - ISR перезаписывал буфер, и уже начатая сборка
 *          достраивалась и отправлялась повреждённой (неверный CRC - VESC
 *          Tool отбрасывает/переспрашивает, не падает, но лишняя задержка и
 *          непредсказуемость). Решение - РАЗДЕЛЬНЫЕ буферы для ISR-пути
 *          (см. vesc_bridge_send_framed_isr) и всех остальных (см.
 *          vesc_bridge_send_framed) - они физически не могут столкнуться,
 *          полноценная критическая секция (которая держала бы прерывания
 *          выключенными на всё время сборки И вызова tx_callback) не нужна.
 *
 * @param  br        хэндл моста
 * @param  frame_buf  буфер сборки (>= VESC_BRIDGE_MAX_PAYLOAD + 8 байт) -
 *                     br->tx_frame_buf либо br->tx_frame_buf_isr, см. выше
 * @param  payload   данные для отправки
 * @param  len       длина данных
 */
static void vesc_bridge_send_framed_buf(VESC_Bridge_t *br, uint8_t *frame_buf,
                                         const uint8_t *payload, uint16_t len)
{
    if (len > VESC_BRIDGE_MAX_PAYLOAD)
    {
        return; /* не должно происходить при штатном использовании - защитная проверка */
    }

    uint16_t ind = 0U;

    if (len <= 255U)
    {
        frame_buf[ind++] = (uint8_t)VESC_BRIDGE_PKT_START_SHORT;
        frame_buf[ind++] = (uint8_t)len;
    }
    else
    {
        frame_buf[ind++] = (uint8_t)VESC_BRIDGE_PKT_START_MEDIUM;
        frame_buf[ind++] = (uint8_t)(len >> 8);
        frame_buf[ind++] = (uint8_t)(len & 0xFFU);
    }

    memcpy(&frame_buf[ind], payload, len);
    ind = (uint16_t)(ind + len);

    uint16_t crc = vesc_bridge_crc16(payload, len);
    frame_buf[ind++] = (uint8_t)(crc >> 8);
    frame_buf[ind++] = (uint8_t)(crc & 0xFFU);
    frame_buf[ind++] = (uint8_t)VESC_BRIDGE_PKT_STOP;

    br->tx_callback(br, frame_buf, ind);
}

/**
 * @brief  Заворачивает payload во внешнее кадрирование и отдаёт через
 *         tx_callback - общая точка выхода для форвардинга результата,
 *         локальных команд (FW_VERSION/PING_CAN/LISP/QML) и
 *         VESC_Bridge_SendLocalReply(). НЕ вызывайте из
 *         VESC_Bridge_OnCanFrame() - см. vesc_bridge_send_framed_isr() и
 *         @warning у vesc_bridge_send_framed_buf().
 * @param  br       хэндл моста
 * @param  payload  данные для отправки
 * @param  len      длина данных
 */
static void vesc_bridge_send_framed(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    vesc_bridge_send_framed_buf(br, br->tx_frame_buf, payload, len);
}

/**
 * @brief  То же самое, что vesc_bridge_send_framed(), но через ОТДЕЛЬНЫЙ
 *         буфер (br->tx_frame_buf_isr) - используйте ТОЛЬКО из
 *         VESC_Bridge_OnCanFrame() (CAN RX ISR) - см. @warning у
 *         vesc_bridge_send_framed_buf().
 * @param  br       хэндл моста
 * @param  payload  данные для отправки
 * @param  len      длина данных
 */
static void vesc_bridge_send_framed_isr(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    vesc_bridge_send_framed_buf(br, br->tx_frame_buf_isr, payload, len);
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

/**
 * @brief  [v1.12] Обрабатывает COMM_LISP_READ_CODE (130) и COMM_GET_QML_UI_APP
 *         (118) - у настоящего VESC Express (vedderb/vesc_express,
 *         commands.c) оба читаются из одного блока кода: если скрипт/QMLUI
 *         не загружен, ответ - ровно 9 байт [packet_id][total_size:int32 BE
 *         = 0][offset:int32 BE = 0] (запрошенные offset+len при этом
 *         превышают нулевой размер, что по общей логике таких ответов и
 *         означает "ничего нет" - без этого VESC Tool ждёт таймаут на
 *         любое открытие вкладки Lisp/QMLUI, даже когда они не используются
 *         вовсе). Байты сверены дословно с исходником Express (сообщены
 *         тестовой сессией) - НЕ эвристика.
 * @param  br       хэндл моста
 * @param  cmd_id   VESC_BRIDGE_COMM_LISP_READ_CODE либо VESC_BRIDGE_COMM_GET_QML_UI_APP (эхом в ответ)
 */
static void vesc_bridge_handle_empty_code_reply(VESC_Bridge_t *br, uint8_t cmd_id)
{
    uint8_t buf[9] = { cmd_id, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
    vesc_bridge_send_framed(br, buf, (uint16_t)sizeof(buf));
}

/**
 * @brief  [v1.12] Обрабатывает COMM_CAN_FWD_FRAME (33) - "сырая" отправка
 *         одного CAN-кадра по явно заданным ID/is_extended, в обход
 *         (cmd_id<<8)|vesc_id адресации остального протокола - использует
 *         VESC Tool для отладки/CAN-анализатора. Формат payload (сверен
 *         дословно с comm_can_transmit_eid/sid в прошивке VESC Express):
 *         [0..3] CAN ID, big-endian (29 бит extended либо 11 бит standard -
 *                какая форма, определяет byte [4])
 *         [4]    is_extended (0/1)
 *         [5:]   данные кадра, 0..8 байт
 *         Без ответа - как и в прошивке (чистая отправка, никакого
 *         подтверждения по этому локальному каналу).
 * @param  br       хэндл моста
 * @param  payload  весь payload (включая COMM_CAN_FWD_FRAME первым байтом)
 * @param  len      его длина
 */
static void vesc_bridge_handle_can_fwd_frame(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)
{
    if (len < 6U)
    {
        return; /* короче минимального (cmd + id[4] + is_ext[1], без данных) - игнорируем молча, как и неизвестные команды */
    }

    uint32_t can_id = (((uint32_t)payload[1] << 24) | ((uint32_t)payload[2] << 16) |
                        ((uint32_t)payload[3] << 8) | (uint32_t)payload[4]);
    uint8_t  is_extended = payload[5];
    uint8_t  data_len    = (uint8_t)(len - 6U);

    if (data_len > 8U)
    {
        return; /* длиннее классического CAN-кадра - некорректный запрос, молча игнорируем */
    }

    (void)CANMGR_Send(br->bus, can_id, is_extended, &payload[6], data_len);
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

static void vesc_bridge_try_start_next_forward(VESC_Bridge_t *br); /* см. определение ниже, нужна уже здесь */

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
 *         заполнился); 0 - нечего слать (IDLE) либо буфер can_manager полон.
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
                /* [v1.12] fire-and-forget - кадр ушёл, команда считается
                 * завершённой НЕМЕДЛЕННО, ответа вески не ждём (см. @note у
                 * vesc_bridge_fwd_phase_t). Сразу освобождаем слот под
                 * следующий запрос из очереди. */
                br->forward_phase  = VESC_BRIDGE_FWD_IDLE;
                br->last_forward_tick = HAL_GetTick();
                sent = 1U;
            }
            __enable_irq();
            if (sent != 0U)
            {
                vesc_bridge_try_start_next_forward(br);
            }
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
                br->last_forward_tick   = HAL_GetTick();
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
                /* [v1.12] fire-and-forget - см. комментарий у FWD_SHORT выше */
                br->forward_phase     = VESC_BRIDGE_FWD_IDLE;
                br->last_forward_tick = HAL_GetTick();
                sent = 1U;
            }
            __enable_irq();
            if (sent != 0U)
            {
                vesc_bridge_try_start_next_forward(br);
            }
            break;
        }

        default:
            break; /* IDLE - активной отправки нет */
    }

    return sent;
}

/**
 * @brief  [v1.12] Записывает n байт в кольцевой буфер очереди форвардинга,
 *         с переносом через границу буфера - НЕ проверяет наличие места
 *         (это обязанность вызывающего кода, см. vesc_bridge_enqueue_forward)
 *         и НЕ защищена критической секцией сама по себе (вызывайте под
 *         __disable_irq(), как и везде в этом файле с общим состоянием).
 * @param  br    хэндл моста
 * @param  data  данные для записи
 * @param  n     их длина, байт
 */
static void vesc_bridge_ring_write(VESC_Bridge_t *br, const uint8_t *data, uint16_t n)
{
    uint16_t first = (uint16_t)(VESC_BRIDGE_FORWARD_QUEUE_BYTES - br->fwd_queue_wr);
    if (first > n)
    {
        first = n;
    }
    memcpy(&br->fwd_queue_buf[br->fwd_queue_wr], data, first);
    if (n > first)
    {
        memcpy(&br->fwd_queue_buf[0], &data[first], (uint16_t)(n - first));
    }
    br->fwd_queue_wr   = (uint16_t)((br->fwd_queue_wr + n) % VESC_BRIDGE_FORWARD_QUEUE_BYTES);
    br->fwd_queue_used = (uint16_t)(br->fwd_queue_used + n);
}

/**
 * @brief  [v1.12] Читает n байт из кольцевого буфера очереди форвардинга (с
 *         продвижением позиции чтения) - см. vesc_bridge_ring_write за
 *         симметричными предупреждениями (вызывающий код гарантирует, что в
 *         кольце реально есть n байт, и сам держит критическую секцию).
 * @param  br   хэндл моста
 * @param  dst  куда скопировать прочитанное
 * @param  n    сколько байт прочитать
 */
static void vesc_bridge_ring_read(VESC_Bridge_t *br, uint8_t *dst, uint16_t n)
{
    uint16_t first = (uint16_t)(VESC_BRIDGE_FORWARD_QUEUE_BYTES - br->fwd_queue_rd);
    if (first > n)
    {
        first = n;
    }
    memcpy(dst, &br->fwd_queue_buf[br->fwd_queue_rd], first);
    if (n > first)
    {
        memcpy(&dst[first], &br->fwd_queue_buf[0], (uint16_t)(n - first));
    }
    br->fwd_queue_rd   = (uint16_t)((br->fwd_queue_rd + n) % VESC_BRIDGE_FORWARD_QUEUE_BYTES);
    br->fwd_queue_used = (uint16_t)(br->fwd_queue_used - n);
}

/**
 * @brief  [v1.12] Ставит запрос форвардинга в очередь (кольцевой буфер, см.
 *         VESC_BRIDGE_FORWARD_QUEUE_BYTES) - НЕ запускает его (см.
 *         vesc_bridge_try_start_next_forward), вызывающий код зовёт обе
 *         функции по порядку (см. vesc_bridge_start_forward). Формат записи
 *         в кольце: [target_id:1][len_hi:1][len_lo:1][payload:len].
 *
 * @warning При нехватке места в кольце - см. VESC_BRIDGE_FORWARD_QUEUE_BYTES:
 *          этот НОВЫЙ запрос целиком ОТБРАСЫВАЕТСЯ (см.
 *          fwd_queue_overflow_count) - в отличие от версии 1.11, уже
 *          стоящие в очереди запросы не трогаются совсем.
 *
 * @param  br         хэндл моста
 * @param  target_id  CAN ID вески-цели
 * @param  inner      вложенная команда (без обёртки COMM_FORWARD_CAN)
 * @param  inner_len  её длина, байт
 */
static void vesc_bridge_enqueue_forward(VESC_Bridge_t *br, uint8_t target_id,
                                         const uint8_t *inner, uint16_t inner_len)
{
    uint16_t record_len = (uint16_t)(3U + inner_len); /* [target_id][len_hi][len_lo] + payload */

    __disable_irq();
    if ((uint32_t)br->fwd_queue_used + (uint32_t)record_len > (uint32_t)VESC_BRIDGE_FORWARD_QUEUE_BYTES)
    {
        br->fwd_queue_overflow_count++; /* см. @warning выше - НОВЫЙ запрос просто теряется */
    }
    else
    {
        uint8_t header[3] = { target_id, (uint8_t)(inner_len >> 8), (uint8_t)(inner_len & 0xFFU) };
        vesc_bridge_ring_write(br, header, 3U);
        vesc_bridge_ring_write(br, inner, inner_len);
    }
    __enable_irq();
}

/**
 * @brief  [v1.12] Если форвардинг сейчас IDLE и в очереди есть хотя бы один
 *         полный запрос (см. vesc_bridge_enqueue_forward - в кольце ВСЕГДА
 *         либо 0, либо целое число полных записей, частичных не бывает, т.к.
 *         enqueue пишет заголовок+payload одной критической секцией) -
 *         снимает голову очереди и запускает её как активный форвардинг
 *         (заполняет forward_src/forward_phase и т.п., как раньше делала
 *         vesc_bridge_start_forward напрямую). Саму отправку НЕ продвигает
 *         (см. vesc_bridge_forward_step - вызывающий код сам решает, звать
 *         ли его сразу или дождаться VESC_Bridge_Tick()). Вызывается после
 *         постановки нового запроса в очередь и сразу же, как только
 *         предыдущий форвардинг фактически ушёл на шину (см. @note у
 *         vesc_bridge_fwd_phase_t - fire-and-forget, не после ответа
 *         вески) - в любом из этих мест форвардинг мог быть уже занят
 *         кем-то другим, поэтому проверка IDLE обязательна.
 * @param  br  хэндл моста
 */
static void vesc_bridge_try_start_next_forward(VESC_Bridge_t *br)
{
    __disable_irq();
    if ((br->forward_phase == VESC_BRIDGE_FWD_IDLE) && (br->fwd_queue_used >= 3U))
    {
        uint8_t header[3];
        vesc_bridge_ring_read(br, header, 3U);
        uint8_t  target_id = header[0];
        uint16_t item_len  = (uint16_t)(((uint16_t)header[1] << 8) | header[2]);
        vesc_bridge_ring_read(br, br->forward_src, item_len);

        br->forward_src_len     = item_len;
        br->forward_send_offset = 0U;
        br->forward_target_id   = target_id;
        br->last_forward_tick   = HAL_GetTick();
        br->forward_phase = (item_len <= 6U) ? VESC_BRIDGE_FWD_SHORT : VESC_BRIDGE_FWD_CHUNKING;
    }
    __enable_irq();
}

/**
 * @brief  Запускает новый форвардинг - ставит его в очередь (см.
 *         vesc_bridge_enqueue_forward) и, если прямо сейчас ничего не
 *         выполняется, сразу же переводит его в активный (см.
 *         vesc_bridge_try_start_next_forward) и пробует продвинуть отправку,
 *         не дожидаясь следующего VESC_Bridge_Tick().
 *
 * @warning [v1.11->1.12] До 1.11 при занятом форвардинге новый запрос
 *          безусловно ЗАТИРАЛ ещё не отправленный старый ("последний
 *          побеждает") - конвейерный VESC Tool (например SET_MCCONF, следом
 *          GET_MCCONF для read-back, не дожидаясь ответа на SET) из-за этого
 *          реально никогда не записывал SET в веску. 1.11 завёл очередь, но
 *          всё ещё ждал ОТВЕТА вески перед стартом следующего запроса - это
 *          НЕ соответствует настоящему VESC Express (comm_can.c на
 *          COMM_FORWARD_CAN не ждёт ответа вообще, см. @note у
 *          vesc_bridge_fwd_phase_t) и на практике душит темп (VESC Tool
 *          шлёт COMM_ALIVE/SET_CURRENT и т.п. намного чаще, чем успевают
 *          приходить ответы). С 1.12 - честный fire-and-forget: запрос
 *          считается выполненным, как только его кадры фактически ушли в
 *          can_manager, следующий стартует сразу же.
 */
static void vesc_bridge_start_forward(VESC_Bridge_t *br, uint8_t target_id,
                                       const uint8_t *inner, uint16_t inner_len)
{
    if (inner_len > VESC_BRIDGE_MAX_PAYLOAD)
    {
        br->rx_error_count++;
        return;
    }

    vesc_bridge_enqueue_forward(br, target_id, inner, inner_len);
    vesc_bridge_try_start_next_forward(br);

    /* Пробуем продвинуть сразу, не дожидаясь следующего VESC_Bridge_Tick() -
     * в типичном случае (буфер свободен, очередь до этого была пуста) весь
     * форвардинг короткой команды уйдёт без задержки в один тик. Каждый
     * вызов сам себя защищает - см. предупреждение у vesc_bridge_forward_step().
     * Если форвардинг занят чем-то другим (только что поставили в очередь,
     * не в активный слот) - forward_step() увидит фазу, отличную от
     * SHORT/CHUNKING/FINAL этого запроса, и не отправит ничего, что и нужно. */
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
    else if ((payload[0] == (uint8_t)VESC_BRIDGE_COMM_LISP_READ_CODE) ||
             (payload[0] == (uint8_t)VESC_BRIDGE_COMM_GET_QML_UI_APP))
    {
        vesc_bridge_handle_empty_code_reply(br, payload[0]); /* см. @brief - байты сверены с Express */
    }
    else if (payload[0] == (uint8_t)VESC_BRIDGE_COMM_CAN_FWD_FRAME)
    {
        vesc_bridge_handle_can_fwd_frame(br, payload, len);
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
    br->rx_timeout_ms     = config->rx_timeout_ms;
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
                if (br->rx_is_medium == 0U)
                {
                    /* [v1.14] Ресинхронизация: этот "кадр" мог быть ложным стартом
                     * (старт-байт 0x02 внутри мусора/хвоста оборванного пакета) - тогда
                     * настоящий старт следующего пакета лежит ВНУТРИ уже принятых байт.
                     * Сохраняем весь поток после ложного старт-байта для перепрогона. */
                    uint16_t n = 0U;
                    br->rx_resync_buf[n++] = (uint8_t)br->rx_expected_len;
                    memcpy(&br->rx_resync_buf[n], br->rx_payload, br->rx_expected_len);
                    n = (uint16_t)(n + br->rx_expected_len);
                    br->rx_resync_buf[n++] = (uint8_t)(br->rx_crc_recv >> 8);
                    br->rx_resync_buf[n++] = (uint8_t)(br->rx_crc_recv & 0xFFU);
                    br->rx_resync_buf[n++] = b;
                    br->rx_resync_len = n;
                }
            }
            break;

        default:
            br->rx_state = VESC_BRIDGE_RX_WAIT_START;
            break;
    }
}

/**
 * @brief  [v1.14] Перепрогоняет через парсер байты, сохранённые после ошибки
 *         CRC/STOP (см. vesc_bridge_feed_one_byte). Итеративно, без рекурсии:
 *         каждая новая ошибка внутри перепрогона кладёт в rx_resync_buf
 *         СТРОГО более короткий поток (минимум первый байт отброшен), поэтому
 *         цикл конечен.
 * @param  br  хэндл моста
 */
static void vesc_bridge_drain_resync(VESC_Bridge_t *br)
{
    while (br->rx_resync_len != 0U)
    {
        uint16_t n = br->rx_resync_len;
        memcpy(br->rx_resync_work, br->rx_resync_buf, n);
        br->rx_resync_len = 0U;
        for (uint16_t i = 0U; i < n; i++)
        {
            vesc_bridge_feed_one_byte(br, br->rx_resync_work[i]);
        }
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
        if (br->rx_resync_len != 0U)
        {
            vesc_bridge_drain_resync(br);
        }
    }
}

/**
 * @brief  Сброс парсера входящего потока - см. подробности в vesc_bridge.h.
 * @param  br  мост, полученный из VESC_Bridge_Init()
 */
void VESC_Bridge_ResetRx(VESC_Bridge_t *br)
{
    if (br == NULL)
    {
        return;
    }
    br->rx_state      = VESC_BRIDGE_RX_WAIT_START;
    br->rx_resync_len = 0U;
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

    /* [v1.12] Дренируем очередь исходящих CAN-кадров форвардинга, если есть
     * что - без блокировки: останавливаемся, как только аппаратный буфер
     * занят прямо сейчас, продолжим на следующем тике (см.
     * vesc_bridge_forward_step). Fire-and-forget (см. @note у
     * vesc_bridge_fwd_phase_t) - здесь больше НЕТ ожидания ответа вески
     * перед стартом следующего запроса из очереди: как только текущий
     * фактически уходит на шину, vesc_bridge_forward_step() сам вызывает
     * vesc_bridge_try_start_next_forward() и этот же while-цикл сразу
     * подхватывает следующий, без лишнего тика ожидания. */
    while (vesc_bridge_forward_step(br) != 0U)
    {
    }

    /* Таймаут незавершённого входящего внешнего пакета (например оборвалась
     * TCP-сессия/потерялся байт по UART посреди пакета) - не ждём остаток
     * вечно, сбрасываем парсер. */
    if ((br->rx_timeout_ms != 0U) && (br->rx_state != VESC_BRIDGE_RX_WAIT_START) &&
        ((HAL_GetTick() - br->rx_last_activity_tick) > br->rx_timeout_ms))
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
             * (не нужен нам), [1]=send_flag (не нужен), [2:]=ответ как есть.
             *
             * [v1.12] НЕ трогаем forward_phase/очередь здесь - приём ответа
             * полностью развязан с отправкой (fire-and-forget, см. @note у
             * vesc_bridge_fwd_phase_t): следующий форвардинг из очереди уже
             * мог начаться и даже завершиться к моменту, когда придёт этот
             * ответ, он ни на что не влияет и ни от чего не зависит. */
            vesc_bridge_send_framed_isr(br, &data[2], (uint16_t)(len - 2U)); /* [v1.13] отдельный буфер - см. её @warning */
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
                    vesc_bridge_send_framed_isr(br, br->can_rx_buf, rxlen); /* [v1.13] отдельный буфер - см. её @warning */
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
            /* [v1.12] forward_phase/очередь не трогаем - см. комментарий в
             * PROCESS_SHORT_BUFFER выше. */
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
 * @brief  Правда ли, что эта веска недавно была (или прямо сейчас является)
 *         целью форвардинга - см. VESC_BRIDGE_FORWARD_TIMEOUT_MS.
 *
 * @note   [v1.12] С версии 1.12 форвардинг fire-and-forget (см. @note у
 *         vesc_bridge_fwd_phase_t) - "идёт форвардинг" в прежнем смысле
 *         (ждём ответа) больше не существует как состояние. Эта функция
 *         теперь означает "forward_target_id == vesc_id И последний кадр в
 *         её сторону отправлен не позже VESC_BRIDGE_FORWARD_TIMEOUT_MS назад
 *         (forward_phase != IDLE ИЛИ last_forward_tick ещё свежий)" -
 *         практический смысл для вызывающего кода не изменился: веска,
 *         вероятно, ещё обрабатывает форвардированный запрос и может
 *         прислать ответ - стоит ненадолго придержать штатные команды
 *         (VESC_CAN_SendXxx) именно ей.
 * @param  br       мост, полученный из VESC_Bridge_Init()
 * @param  vesc_id  проверяемый CAN ID вески
 * @return 1, если условие выше верно, иначе 0
 */
uint8_t VESC_Bridge_IsTargetActive(VESC_Bridge_t *br, uint8_t vesc_id)
{
    if ((br == NULL) || (br->forward_target_id != vesc_id))
    {
        return 0U;
    }
    if (br->forward_phase != VESC_BRIDGE_FWD_IDLE)
    {
        return 1U; /* прямо сейчас отправляем ей очередной кадр */
    }
    return ((HAL_GetTick() - br->last_forward_tick) <= VESC_BRIDGE_FORWARD_TIMEOUT_MS) ? 1U : 0U;
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

/**
 * @brief  [v1.11] Счётчик переполнений очереди форвардинга.
 * @param  br  мост, полученный из VESC_Bridge_Init()
 * @return количество переполнений очереди
 */
uint32_t VESC_Bridge_GetForwardQueueOverflowCount(VESC_Bridge_t *br)
{
    return (br != NULL) ? br->fwd_queue_overflow_count : 0U;
}
