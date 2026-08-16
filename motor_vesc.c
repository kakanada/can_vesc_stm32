/**
 ******************************************************************************
 * @file    motor_vesc.c
 * @brief   Реализация портируемой библиотеки обмена с VESC по CAN/FDCAN.
 *          См. motor_vesc.h
 * @author  Mechanic
 * @date    12.08.2026
 * @version 1.1
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include "motor_vesc.h"
#include <string.h>
#include <math.h>

/* ========================================================================
 *  Внутреннее состояние модуля
 * ====================================================================== */

/** Контекст одной физической CAN-шины (CAN1, CAN2, FDCAN1, ...). */
typedef struct
{
    VESC_CAN_HandleTypeDef *hcan;   /* NULL - слот свободен */
    uint8_t                 configured;    /* фильтр/нотификации/старт уже выполнены */
    uint8_t                 flush_cursor;  /* round-robin индекс досылки ДЛЯ ЭТОЙ шины */
} VESC_Bus_t;

/* VESC_Handle_t объявлен целиком в motor_vesc.h (см. пояснение там же) -
 * здесь просто статический пул хэндлов, на которые модуль отдаёт указатели. */
static VESC_Handle_t s_pool[VESC_CAN_MAX_DEVICES];
static VESC_Bus_t    s_buses[VESC_CAN_MAX_BUSES];

/* ========================================================================
 *  Слой портируемости (port_*) - тут и только тут отличаются FDCAN и bxCAN
 * ====================================================================== */

#if defined(VESC_CAN_BACKEND_FDCAN)

/** Переводит длину данных в байтах в код DLC, который понимает регистр FDCAN. */
static uint32_t port_len_to_dlc(uint8_t len)
{
    switch (len)
    {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

/** Обратное преобразование: код DLC регистра FDCAN -> длина данных в байтах. */
static uint8_t port_dlc_to_len(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0: return 0U;
        case FDCAN_DLC_BYTES_1: return 1U;
        case FDCAN_DLC_BYTES_2: return 2U;
        case FDCAN_DLC_BYTES_3: return 3U;
        case FDCAN_DLC_BYTES_4: return 4U;
        case FDCAN_DLC_BYTES_5: return 5U;
        case FDCAN_DLC_BYTES_6: return 6U;
        case FDCAN_DLC_BYTES_7: return 7U;
        default:                return 8U;
    }
}

/** Настраивает единственный фильтр Extended ID, пропускающий ЛЮБОЙ
 *  расширенный ID в RxFIFO0 (маска 0 = "не важно, какой именно ID"). */
static HAL_StatusTypeDef port_config_filter(VESC_CAN_HandleTypeDef *hcan)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = VESC_CAN_FDCAN_FILTER_INDEX;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x00000000U;
    filter.FilterID2    = 0x00000000U;
    return HAL_FDCAN_ConfigFilter(hcan, &filter);
}

/** Включает нотификации о новом сообщении в RxFIFO0 и об опустошении Tx FIFO. */
static HAL_StatusTypeDef port_activate_notifications(VESC_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_ActivateNotification(hcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0U);
}

/** Запускает периферию (переводит из режима конфигурации в рабочий). */
static HAL_StatusTypeDef port_start(VESC_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_Start(hcan);
}

/** Сколько свободных мест в аппаратном передающем буфере прямо сейчас. */
static uint32_t port_get_tx_free_level(VESC_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(hcan);
}

/** Кладёт один кадр (расширенный ID, классический CAN, без BRS) в Tx FIFO/Queue. */
static HAL_StatusTypeDef port_send(VESC_CAN_HandleTypeDef *hcan, uint32_t ext_id,
                                    const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier          = ext_id;
    h.IdType               = FDCAN_EXTENDED_ID;
    h.TxFrameType          = FDCAN_DATA_FRAME;
    h.DataLength            = port_len_to_dlc(len);
    h.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch        = FDCAN_BRS_OFF;   /* VESC не понимает CAN FD/BRS */
    h.FDFormat              = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl   = FDCAN_NO_TX_EVENTS;
    h.MessageMarker        = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &h, (uint8_t *)data);
}

/** Забирает одно сообщение из RxFIFO0, если оно там есть. */
static HAL_StatusTypeDef port_receive(VESC_CAN_HandleTypeDef *hcan, uint32_t *ext_id,
                                       uint8_t *is_ext, uint8_t *data, uint8_t *len)
{
    FDCAN_RxHeaderTypeDef rh;
    HAL_StatusTypeDef st = HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rh, data);
    if (st == HAL_OK)
    {
        *ext_id = rh.Identifier;
        *is_ext = (rh.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
        *len    = port_dlc_to_len(rh.DataLength);
    }
    return st;
}

#elif defined(VESC_CAN_BACKEND_BXCAN)

/** Настраивает единственный фильтр (32-бит, режим маски), пропускающий ЛЮБОЙ
 *  расширенный ID в RxFIFO0: проверяется только бит IDE=1, сам ID не важен.
 *  bank/slave_start учитывают, что банки 0..27 физически общие на CAN1+CAN2. */
static HAL_StatusTypeDef port_config_filter(VESC_CAN_HandleTypeDef *hcan, uint32_t bank, uint32_t slave_start)
{
    CAN_FilterTypeDef f = {0};
    f.FilterIdHigh         = 0x0000U;
    f.FilterIdLow          = CAN_ID_EXT;   /* требуем IDE=1 (расширенный кадр)   */
    f.FilterMaskIdHigh     = 0x0000U;
    f.FilterMaskIdLow      = CAN_ID_EXT;   /* маскируем ТОЛЬКО бит IDE, ID любой */
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterBank           = bank;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = slave_start;
    return HAL_CAN_ConfigFilter(hcan, &f);
}

/** Включает нотификации о новом сообщении в RxFIFO0 и об освобождении mailbox-ов. */
static HAL_StatusTypeDef port_activate_notifications(VESC_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_ActivateNotification(hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}

/** Запускает периферию (переводит из режима конфигурации в рабочий). */
static HAL_StatusTypeDef port_start(VESC_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_Start(hcan);
}

/** Сколько из 3 передающих mailbox-ов свободны прямо сейчас. */
static uint32_t port_get_tx_free_level(VESC_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_GetTxMailboxesFreeLevel(hcan);
}

/** Кладёт один кадр (расширенный ID, данные, не remote) в свободный mailbox. */
static HAL_StatusTypeDef port_send(VESC_CAN_HandleTypeDef *hcan, uint32_t ext_id,
                                    const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef h = {0};
    h.ExtId = ext_id;
    h.IDE   = CAN_ID_EXT;
    h.RTR   = CAN_RTR_DATA;
    h.DLC   = len;
    h.TransmitGlobalTime = DISABLE;
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(hcan, &h, (uint8_t *)data, &mailbox);
}

/** Забирает одно сообщение из RxFIFO0, если оно там есть. */
static HAL_StatusTypeDef port_receive(VESC_CAN_HandleTypeDef *hcan, uint32_t *ext_id,
                                       uint8_t *is_ext, uint8_t *data, uint8_t *len)
{
    CAN_RxHeaderTypeDef rh;
    HAL_StatusTypeDef st = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rh, data);
    if (st == HAL_OK)
    {
        *is_ext = (rh.IDE == CAN_ID_EXT) ? 1U : 0U;
        *ext_id = (*is_ext != 0U) ? rh.ExtId : rh.StdId;
        *len    = (uint8_t)rh.DLC;
    }
    return st;
}

#endif /* backend selection */

/* ========================================================================
 *  Общие (не зависящие от бэкенда) вспомогательные функции
 * ====================================================================== */

/** Ищет уже зарегистрированный хэндл по паре (шина, CAN ID) - используется
 *  ТОЛЬКО внутри модуля для демультиплексирования входящих кадров (RX) и
 *  обхода пула на досылке (TX); "снаружи" модуль адресуется через указатель
 *  VESC_Handle_t*, полученный из VESC_CAN_Init(), повторный поиск по ID на
 *  каждый вызов команды не нужен и не делается. */
static VESC_Handle_t *vesc_find(VESC_CAN_HandleTypeDef *hcan, uint8_t vesc_id)
{
    for (uint32_t i = 0U; i < VESC_CAN_MAX_DEVICES; i++)
    {
        if (s_pool[i].used && (s_pool[i].hcan == hcan) && (s_pool[i].vesc_id == vesc_id))
        {
            return &s_pool[i];
        }
    }
    return NULL;
}

/** Ищет первый свободный (ещё не занятый) слот в общем пуле весок. */
static VESC_Handle_t *vesc_find_free_slot(void)
{
    for (uint32_t i = 0U; i < VESC_CAN_MAX_DEVICES; i++)
    {
        if (!s_pool[i].used)
        {
            return &s_pool[i];
        }
    }
    return NULL;
}

/** Ищет контекст уже зарегистрированной (сконфигурированной) шины по hcan. */
static VESC_Bus_t *bus_find(VESC_CAN_HandleTypeDef *hcan)
{
    for (uint32_t i = 0U; i < VESC_CAN_MAX_BUSES; i++)
    {
        if (s_buses[i].hcan == hcan)
        {
            return &s_buses[i];
        }
    }
    return NULL;
}

/** Возвращает контекст шины по hcan, создавая новый (в первом свободном
 *  слоте s_buses[]), если такая шина видится впервые. NULL, если исчерпан
 *  VESC_CAN_MAX_BUSES. Возвращает и индекс шины (нужен для расчёта банка
 *  фильтра на bxCAN) через out_index. */
static VESC_Bus_t *bus_find_or_alloc(VESC_CAN_HandleTypeDef *hcan, uint32_t *out_index)
{
    VESC_Bus_t *existing = bus_find(hcan);
    if (existing != NULL)
    {
        if (out_index != NULL) { *out_index = (uint32_t)(existing - s_buses); }
        return existing;
    }

    for (uint32_t i = 0U; i < VESC_CAN_MAX_BUSES; i++)
    {
        if (s_buses[i].hcan == NULL)
        {
            s_buses[i].hcan = hcan;
            s_buses[i].configured = 0U;
            s_buses[i].flush_cursor = 0U;
            if (out_index != NULL) { *out_index = i; }
            return &s_buses[i];
        }
    }
    return NULL;
}

/** Собирает 29-битный Extended ID из кода команды и CAN ID вески (см.
 *  формат кадра VESC: биты 15-8 = команда, биты 7-0 = ID вески). */
static inline uint32_t make_ext_id(VESC_CAN_PacketId_t cmd, uint8_t vesc_id)
{
    return (((uint32_t)cmd) << 8) | (uint32_t)vesc_id;
}

/** Упаковывает 32-битное знаковое число в 4 байта big-endian (старший байт
 *  первый) - так VESC ожидает аргумент любой "простой" команды. */
static void pack_i32_be(uint8_t *out, int32_t v)
{
    out[0] = (uint8_t)((uint32_t)v >> 24);
    out[1] = (uint8_t)((uint32_t)v >> 16);
    out[2] = (uint8_t)((uint32_t)v >> 8);
    out[3] = (uint8_t)((uint32_t)v);
}

/** Извлекает 32-битное знаковое big-endian поле из буфера (для статусов). */
static inline int32_t be_to_i32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                      ((uint32_t)p[2] << 8)  |  (uint32_t)p[3]);
}

/** Извлекает 16-битное знаковое big-endian поле из буфера (для статусов). */
static inline int16_t be_to_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/** Ограничивает |v| значением lim (симметрично, для программных лимитов
 *  скорости/тока - см. VESC_CAN_SetSpeedLimit/SetCurrentLimit). */
static float clampf(float v, float lim)
{
    if (v >  lim) { return  lim; }
    if (v < -lim) { return -lim; }
    return v;
}

/** "Губернатор" по телеметрии: коэффициент 0..1, на который надо придушить
 *  запрошенное значение при приближении ИЗМЕРЕННОЙ величины measured_abs к
 *  пределу limit в пределах полосы margin. 1.0 - далеко от предела, ничего
 *  не трогаем; 0.0 - на пределе или уже за ним. Общая для обоих
 *  перекрёстных губернаторов (по скорости внутри SendCurrent и по току
 *  внутри SendSpeed) - см. подробное объяснение в motor_vesc.h. */
static float vesc_governor_scale(float measured_abs, float limit, float margin)
{
    if (margin <= 0.0f) { margin = 1.0f; } /* защита от деления на 0 при некорректно заданной полосе */
    if (measured_abs >= limit) { return 0.0f; }
    if (measured_abs <= (limit - margin)) { return 1.0f; }
    return (limit - measured_abs) / margin;
}

/* ========================================================================
 *  Программная очередь отложенных команд (по одной веске)
 * ====================================================================== */

/** Кладёт (или обновляет, если такая команда уже ждёт своей очереди)
 *  значение команды cmd_id в очередь отложенных команд вески h. Возвращает
 *  HAL_BUSY при успехе, HAL_ERROR если все VESC_CAN_MAX_PENDING_PER_VESC
 *  слотов заняты РАЗНЫМИ командами (в реальной работе почти невозможно). */
static HAL_StatusTypeDef vesc_enqueue_pending(VESC_Handle_t *h, uint8_t cmd_id, int32_t value)
{
    __disable_irq();

    for (uint32_t i = 0U; i < VESC_CAN_MAX_PENDING_PER_VESC; i++)
    {
        if (h->pending[i].valid && (h->pending[i].cmd_id == cmd_id))
        {
            h->pending[i].value = value; /* устаревшее значение никому не нужно - просто заменяем */
            __enable_irq();
            return HAL_BUSY;
        }
    }
    for (uint32_t i = 0U; i < VESC_CAN_MAX_PENDING_PER_VESC; i++)
    {
        if (!h->pending[i].valid)
        {
            h->pending[i].cmd_id = cmd_id;
            h->pending[i].value  = value;
            h->pending[i].valid  = 1U;
            __enable_irq();
            return HAL_BUSY;
        }
    }

    __enable_irq();
    return HAL_ERROR; /* все слоты заняты разными командами одновременно */
}

/** Если для вески h в очереди отложенных команд ждёт устаревшее значение
 *  команды cmd_id - убирает его (используется, когда свежее значение той же
 *  команды только что ушло НАПРЯМУЮ в периферию, минуя очередь - иначе
 *  устаревшее значение потом досослалось бы ПОСЛЕ свежего). */
static void vesc_cancel_pending(VESC_Handle_t *h, uint8_t cmd_id)
{
    __disable_irq();
    for (uint32_t i = 0U; i < VESC_CAN_MAX_PENDING_PER_VESC; i++)
    {
        if (h->pending[i].valid && (h->pending[i].cmd_id == cmd_id))
        {
            h->pending[i].valid = 0U;
            break;
        }
    }
    __enable_irq();
}

/** Пытается протолкнуть в аппаратный буфер ВСЕ отложенные команды вески h.
 *  Возвращает 1, если очередь этой вески полностью опустела (либо изначально
 *  была пуста), 0 - если периферия заполнилась раньше, чем управились (тогда
 *  оставшиеся команды остаются в очереди до следующего вызова). */
static uint8_t vesc_flush_one(VESC_CAN_HandleTypeDef *hcan, VESC_Handle_t *h)
{
    for (uint32_t i = 0U; i < VESC_CAN_MAX_PENDING_PER_VESC; i++)
    {
        if (!h->pending[i].valid)
        {
            continue;
        }
        if (port_get_tx_free_level(hcan) == 0U)
        {
            return 0U;
        }

        __disable_irq();
        uint8_t cmd  = h->pending[i].cmd_id;
        int32_t val  = h->pending[i].value;
        h->pending[i].valid = 0U;
        __enable_irq();

        uint8_t payload[4];
        pack_i32_be(payload, val);

        if (port_send(hcan, make_ext_id((VESC_CAN_PacketId_t)cmd, h->vesc_id), payload, 4U) != HAL_OK)
        {
            /* не получилось (редкая гонка) - вернуть значение обратно в очередь */
            __disable_irq();
            h->pending[i].cmd_id = cmd;
            h->pending[i].value  = val;
            h->pending[i].valid  = 1U;
            __enable_irq();
            return 0U;
        }
    }
    return 1U;
}

/* ========================================================================
 *  Публичный API - регистрация и телеметрия
 * ====================================================================== */

/** Регистрирует веску по заполненной конфигурации и (один раз на шину)
 *  настраивает фильтр, нотификации и запускает периферию. Подробности -
 *  см. motor_vesc.h. */
VESC_Handle_t *VESC_CAN_Init(const VESC_Config_t *config)
{
    if ((config == NULL) || (config->hcan == NULL))
    {
        return NULL;
    }
    if ((config->pole_count == 0U) || ((config->pole_count % 2U) != 0U))
    {
        return NULL; /* число полюсов должно быть чётным и ненулевым */
    }

    /* Идемпотентность: если такая (hcan, vesc_id) уже зарегистрирована -
     * просто вернуть существующий хэндл, ничего заново не настраивая. Но
     * если pole_count во втором вызове ОТЛИЧАЕТСЯ от того, что было при
     * первой регистрации - это, скорее всего, ошибка в вызывающем коде
     * (например, конфиг для этой вески в двух местах отличается) - лучше
     * вернуть NULL явной ошибкой, чем молча использовать первое значение
     * и получить незаметно неверный mech_rpm. */
    VESC_Handle_t *existing = vesc_find(config->hcan, config->vesc_id);
    if (existing != NULL)
    {
        if (existing->pole_pairs != (config->pole_count / 2U))
        {
            return NULL; /* pole_count разошёлся между повторными вызовами Init для этой же вески */
        }
        return existing;
    }

    uint32_t bus_index = 0U;
    VESC_Bus_t *bus = bus_find_or_alloc(config->hcan, &bus_index);
    if (bus == NULL)
    {
        return NULL; /* исчерпан VESC_CAN_MAX_BUSES */
    }

    if (!bus->configured)
    {
#if defined(VESC_CAN_BACKEND_FDCAN)
        if (port_config_filter(config->hcan) != HAL_OK) { return NULL; }
#else
        uint32_t bank        = bus_index * VESC_CAN_BXCAN_BANKS_PER_BUS;
        uint32_t slave_start = VESC_CAN_BXCAN_BANKS_PER_BUS;
        if (port_config_filter(config->hcan, bank, slave_start) != HAL_OK) { return NULL; }
#endif
        if (port_activate_notifications(config->hcan) != HAL_OK) { return NULL; }
        if (port_start(config->hcan) != HAL_OK) { return NULL; }
        bus->configured = 1U;
    }

    VESC_Handle_t *h = vesc_find_free_slot();
    if (h == NULL)
    {
        return NULL; /* исчерпан VESC_CAN_MAX_DEVICES */
    }

    memset(h, 0, sizeof(*h));
    h->used       = 1U;
    h->vesc_id    = config->vesc_id;
    h->hcan       = config->hcan;
    h->pole_pairs = config->pole_count / 2U;

#if defined(HAL_RTC_MODULE_ENABLED)
    h->position_memory_hrtc         = config->position_memory_hrtc;
    h->position_memory_backup_index = config->position_memory_backup_index;
#endif

    return h;
}

/** Возвращает &h->telemetry (см. пояснение в motor_vesc.h) - можно и нужно
 *  обращаться к h->telemetry напрямую, эта функция для удобства/симметрии API. */
const VESC_Telemetry_t *VESC_CAN_GetTelemetry(VESC_Handle_t *h)
{
    return (h != NULL) ? &h->telemetry : NULL;
}

/** Проверяет, не "протухла" ли телеметрия вески дольше timeout_ms. */
uint8_t VESC_CAN_IsAlive(VESC_Handle_t *h, uint32_t timeout_ms)
{
    if (h == NULL)
    {
        return 0U;
    }
    return ((HAL_GetTick() - h->telemetry.last_rx_tick) <= timeout_ms) ? 1U : 0U;
}

/* ========================================================================
 *  Команды на веску - общий отправитель + тонкие обёртки
 * ====================================================================== */

/** Общая реализация для всех "простых" (однокадровых, 4 байта) команд:
 *  быстрый путь напрямую в периферию, иначе - в программную очередь этой
 *  вески. Для имитируемых весок реальная передача пропускается (см.
 *  VESC_CAN_SetSimulated). */
static HAL_StatusTypeDef vesc_send_simple(VESC_Handle_t *h, VESC_CAN_PacketId_t cmd, int32_t scaled)
{
#if VESC_CAN_SIM_ENABLE
    if (cmd == VESC_CAN_PACKET_SET_RPM)
    {
        h->last_commanded_rpm = scaled; /* вход модели имитации, нужен независимо от simulated */
    }
    if (h->simulated)
    {
        return HAL_OK; /* реального обмена не будет - см. VESC_CAN_SimulateTick() */
    }
#endif

    uint8_t payload[4];
    pack_i32_be(payload, scaled);

    if (port_get_tx_free_level(h->hcan) > 0U)
    {
        if (port_send(h->hcan, make_ext_id(cmd, h->vesc_id), payload, 4U) == HAL_OK)
        {
            vesc_cancel_pending(h, (uint8_t)cmd); /* свежее значение уже ушло - старое отложенное не нужно */
            return HAL_OK;
        }
        /* иначе (редкая гонка) - падаем в программный путь ниже */
    }

    return vesc_enqueue_pending(h, (uint8_t)cmd, scaled);
}

/** Duty Cycle напрямую. Масштаб 100000, диапазон -1.0..1.0. */
HAL_StatusTypeDef VESC_CAN_SendDuty(VESC_Handle_t *h, float duty)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_DUTY, (int32_t)(duty * 100000.0f));
}

/** Ток мотора, А. Масштаб 1000. Ограничивается VESC_CAN_SetCurrentLimit, если включён. */
HAL_StatusTypeDef VESC_CAN_SendCurrent(VESC_Handle_t *h, float current)
{
    if (h == NULL) { return HAL_ERROR; }

    if (h->current_limit_enabled) { current = clampf(current, h->current_limit); }

    /* Перекрёстный губернатор: управляем током, но не даём веске разогнаться
     * быстрее speed_limit - придушиваем запрошенный ток по мере приближения
     * ИЗМЕРЕННОЙ скорости (последняя телеметрия) к пределу. Важно: глушим
     * ТОЛЬКО если ток толкает В ТУ ЖЕ сторону, куда уже крутится вал (т.е.
     * реально может разогнать его ещё дальше за предел) - если знак тока
     * противоположен erpm (торможение/реверс), это не разгон, а замедление -
     * губернатор не должен в это вмешиваться. См. motor_vesc.h. */
    if (h->speed_limit_enabled)
    {
        float erpm = h->telemetry.erpm;
        uint8_t same_direction = (uint8_t)((current >= 0.0f) == (erpm >= 0.0f));
        if (same_direction)
        {
            float scale = vesc_governor_scale(fabsf(erpm), h->speed_limit, h->speed_limit_margin);
            current *= scale;
        }
    }

    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT, (int32_t)(current * 1000.0f));
}

/** Тормозной ток ("тормоз мотором"), А. Масштаб 1000. Ограничивается VESC_CAN_SetCurrentLimit. */
HAL_StatusTypeDef VESC_CAN_SendCurrentBrake(VESC_Handle_t *h, float brake_current)
{
    if (h == NULL) { return HAL_ERROR; }
    if (h->current_limit_enabled) { brake_current = clampf(brake_current, h->current_limit); }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_BRAKE, (int32_t)(brake_current * 1000.0f));
}

/** Целевая скорость, эл. RPM. Масштаб 1. Ограничивается VESC_CAN_SetSpeedLimit. */
HAL_StatusTypeDef VESC_CAN_SendSpeed(VESC_Handle_t *h, float pid_speed)
{
    if (h == NULL) { return HAL_ERROR; }

    if (h->speed_limit_enabled) { pid_speed = clampf(pid_speed, h->speed_limit); }

    /* Перекрёстный губернатор: управляем скоростью, но не даём току на веске
     * превысить current_limit - перестаём НАРАЩИВАТЬ запрошенную скорость
     * сверх уже ИЗМЕРЕННОЙ (последняя телеметрия), по мере приближения
     * измеренного тока к пределу. Важно: throttle применяется ТОЛЬКО когда
     * pid_speed реально требует ускориться В ТУ ЖЕ сторону, куда вал уже
     * крутится (|pid_speed| > |erpm| и то же направление) - снижение
     * скорости или смена направления губернатором не тормозится, это не
     * тот случай, который может разогнать ток дальше предела. См. motor_vesc.h. */
    if (h->current_limit_enabled)
    {
        float erpm = h->telemetry.erpm;
        uint8_t same_direction = (uint8_t)((pid_speed >= 0.0f) == (erpm >= 0.0f));
        uint8_t increasing     = (uint8_t)(same_direction && (fabsf(pid_speed) > fabsf(erpm)));
        if (increasing)
        {
            float scale = vesc_governor_scale(fabsf(h->telemetry.current), h->current_limit, h->current_limit_margin);
            float increment = (pid_speed - erpm) * scale;
            pid_speed = erpm + increment;
        }
    }

    return vesc_send_simple(h, VESC_CAN_PACKET_SET_RPM, (int32_t)pid_speed);
}

/** Пересчитывает механические RPM в электрические (через pole_count из
 *  конфига) и отправляет тем же путём, что и VESC_CAN_SendSpeed. */
HAL_StatusTypeDef VESC_CAN_SendMechanicalSpeed(VESC_Handle_t *h, float mech_rpm)
{
    if (h == NULL) { return HAL_ERROR; }
    return VESC_CAN_SendSpeed(h, mech_rpm * (float)h->pole_pairs);
}

/** Целевая позиция, градусы 0..360. Масштаб 1000000. */
HAL_StatusTypeDef VESC_CAN_SendPosition(VESC_Handle_t *h, float position_deg)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_POS, (int32_t)(position_deg * 1000000.0f));
}

/** Ток относительно максимального. Масштаб 100000, диапазон -1.0..1.0. Не клэмпится VESC_CAN_SetCurrentLimit (unit mismatch, см. motor_vesc.h). */
HAL_StatusTypeDef VESC_CAN_SendCurrentRel(VESC_Handle_t *h, float current_rel)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_REL, (int32_t)(current_rel * 100000.0f));
}

/** Тормозной ток относительно максимального. Масштаб 100000, диапазон -1.0..1.0. */
HAL_StatusTypeDef VESC_CAN_SendCurrentBrakeRel(VESC_Handle_t *h, float brake_current_rel)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_BRAKE_REL, (int32_t)(brake_current_rel * 100000.0f));
}

/** "Handbrake"-ток, А. Масштаб 1000. Ограничивается VESC_CAN_SetCurrentLimit. См. предупреждение в motor_vesc.h про неподтверждённую точную семантику. */
HAL_StatusTypeDef VESC_CAN_SendHandbrakeCurrent(VESC_Handle_t *h, float handbrake_current)
{
    if (h == NULL) { return HAL_ERROR; }
    if (h->current_limit_enabled) { handbrake_current = clampf(handbrake_current, h->current_limit); }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE, (int32_t)(handbrake_current * 1000.0f));
}

/** "Handbrake"-ток относительно максимального. Масштаб 100000, диапазон -1.0..1.0. */
HAL_StatusTypeDef VESC_CAN_SendHandbrakeCurrentRel(VESC_Handle_t *h, float handbrake_current_rel)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE_REL, (int32_t)(handbrake_current_rel * 100000.0f));
}

/* ========================================================================
 *  Кастомная команда: принудительное отпускание тормоза
 * ====================================================================== */

/** Отправляет 1-байтовую кастомную команду "отпустить тормоз" один раз.
 *  Намеренно НЕ использует vesc_send_simple()/программную очередь (см.
 *  обоснование в motor_vesc.h) - при занятом аппаратном буфере просто
 *  сообщает об этом вызывающей стороне (HAL_BUSY), без досылки. */
HAL_StatusTypeDef VESC_CAN_SendReleaseBrake(VESC_Handle_t *h)
{
    if (h == NULL) { return HAL_ERROR; }

#if VESC_CAN_SIM_ENABLE
    if (h->simulated)
    {
        return HAL_OK; /* реального обмена не будет - имитируемая веска */
    }
#endif

    uint8_t payload[1] = { 0x01U };

    if (port_get_tx_free_level(h->hcan) == 0U)
    {
        return HAL_BUSY; /* буфер полон прямо сейчас - вызовите функцию ещё раз чуть позже */
    }

    return port_send(h->hcan, make_ext_id(VESC_CAN_PACKET_CUSTOM_BRAKE_CMD, h->vesc_id), payload, 1U);
}

/* ========================================================================
 *  Программные ограничения скорости/тока
 * ====================================================================== */

/** Включает губернатор скорости (прямой clamp для SendSpeed + перекрёстный
 *  потолок для SendCurrent) - см. подробности в motor_vesc.h. */
HAL_StatusTypeDef VESC_CAN_SetSpeedLimit(VESC_Handle_t *h, float max_abs_erpm, float governor_margin_erpm)
{
    if (h == NULL) { return HAL_ERROR; }
    if (governor_margin_erpm <= 0.0f) { governor_margin_erpm = 1.0f; }        /* защита от деления на 0 в губернаторе */
    if (governor_margin_erpm > max_abs_erpm) { governor_margin_erpm = max_abs_erpm; } /* полоса шире предела бессмысленна */
    h->speed_limit = max_abs_erpm;
    h->speed_limit_margin = governor_margin_erpm;
    h->speed_limit_enabled = 1U;
    return HAL_OK;
}

/** Снимает ограничение скорости, включённое VESC_CAN_SetSpeedLimit(). */
HAL_StatusTypeDef VESC_CAN_ClearSpeedLimit(VESC_Handle_t *h)
{
    if (h == NULL) { return HAL_ERROR; }
    h->speed_limit_enabled = 0U;
    return HAL_OK;
}

/** Включает губернатор тока (прямой clamp для SendCurrent/SendCurrentBrake/
 *  SendHandbrakeCurrent + перекрёстный потолок для SendSpeed). */
HAL_StatusTypeDef VESC_CAN_SetCurrentLimit(VESC_Handle_t *h, float max_abs_current, float governor_margin_current)
{
    if (h == NULL) { return HAL_ERROR; }
    if (governor_margin_current <= 0.0f) { governor_margin_current = 1.0f; }
    if (governor_margin_current > max_abs_current) { governor_margin_current = max_abs_current; }
    h->current_limit = max_abs_current;
    h->current_limit_margin = governor_margin_current;
    h->current_limit_enabled = 1U;
    return HAL_OK;
}

/** Снимает ограничение тока, включённое VESC_CAN_SetCurrentLimit(). */
HAL_StatusTypeDef VESC_CAN_ClearCurrentLimit(VESC_Handle_t *h)
{
    if (h == NULL) { return HAL_ERROR; }
    h->current_limit_enabled = 0U;
    return HAL_OK;
}

/* ========================================================================
 *  Память положения (backup-регистры RTC) - см. подробное honest-объяснение
 *  в motor_vesc.h. Гейтится HAL_RTC_MODULE_ENABLED, как и имитация гейтится
 *  VESC_CAN_SIM_ENABLE - при отсутствии RTC в проекте компилируются
 *  функции-заглушки в самом низу этой секции.
 * ====================================================================== */

#if defined(HAL_RTC_MODULE_ENABLED)

/** Маркер-"магия" во втором backup-регистре - отличает "тут когда-то уже
 *  писали валидные данные" от "регистр после сброса/первого включения,
 *  содержимое случайное/нулевое". */
#define VESC_POSMEM_MAGIC   0x56455343UL /* ASCII 'VESC' */

/** Читает последнее сохранённое положение (градусы) из backup-регистров
 *  вески h. Если магия не совпала (данных ещё не было) - возвращает 0.0. */
static float vesc_posmem_read(VESC_Handle_t *h)
{
    uint32_t magic = HAL_RTCEx_BKUPRead(h->position_memory_hrtc, h->position_memory_backup_index + 1U);
    if (magic != VESC_POSMEM_MAGIC)
    {
        return 0.0f; /* валидных данных ещё не сохранялось */
    }
    uint32_t raw = HAL_RTCEx_BKUPRead(h->position_memory_hrtc, h->position_memory_backup_index);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/** Сохраняет положение (градусы) в backup-регистры вески h, вместе с
 *  маркером "магии", подтверждающим валидность при следующем чтении. */
static void vesc_posmem_write(VESC_Handle_t *h, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    HAL_RTCEx_BKUPWrite(h->position_memory_hrtc, h->position_memory_backup_index, raw);
    HAL_RTCEx_BKUPWrite(h->position_memory_hrtc, h->position_memory_backup_index + 1U, VESC_POSMEM_MAGIC);
}

/** Заворачивает угол в диапазон [0, 360). Общая для всех мест, где
 *  считается скорректированное офсетом положение. */
static float vesc_wrap360(float deg)
{
    float w = fmodf(deg, 360.0f);
    if (w < 0.0f) { w += 360.0f; }
    return w;
}

#endif /* HAL_RTC_MODULE_ENABLED */

/* ========================================================================
 *  Память положения - публичный API (см. подробности в motor_vesc.h)
 * ====================================================================== */

#if defined(HAL_RTC_MODULE_ENABLED)

/** Включает/выключает память положения (реальная реализация, есть RTC). */
HAL_StatusTypeDef VESC_CAN_SetPositionMemoryEnabled(VESC_Handle_t *h, uint8_t enabled)
{
    if ((h == NULL) || (h->position_memory_hrtc == NULL)) { return HAL_ERROR; }

    if (enabled)
    {
        HAL_PWR_EnableBkUpAccess();
        if ((h->telemetry.rx_mask & VESC_CAN_RXMASK_STATUS_4) != 0U)
        {
            /* Уже есть свежая телеметрия - можно посчитать офсет немедленно. */
            h->position_offset_deg = vesc_posmem_read(h) - h->last_raw_pid_pos;
            h->position_memory_pending_restore = 0U;
        }
        else
        {
            h->position_memory_pending_restore = 1U; /* посчитаем на первом же STATUS_4 */
        }
        h->position_memory_enabled = 1U;
    }
    else
    {
        h->position_memory_enabled = 0U;
    }
    return HAL_OK;
}

/** Ручная калибровка текущего положения (реальная реализация, есть RTC). */
HAL_StatusTypeDef VESC_CAN_SetCurrentPosition(VESC_Handle_t *h, float actual_position_deg)
{
    if ((h == NULL) || (h->position_memory_hrtc == NULL)) { return HAL_ERROR; }
    if ((h->telemetry.rx_mask & VESC_CAN_RXMASK_STATUS_4) == 0U)
    {
        return HAL_ERROR; /* ещё ни разу не было STATUS_4 - не от чего считать офсет */
    }

    HAL_PWR_EnableBkUpAccess();
    h->position_offset_deg = actual_position_deg - h->last_raw_pid_pos;
    h->position_memory_pending_restore = 0U;
    h->position_memory_enabled = 1U;

    float wrapped = vesc_wrap360(actual_position_deg);
    h->telemetry.pid_pos = wrapped;
    vesc_posmem_write(h, wrapped);
    return HAL_OK;
}

#else /* !HAL_RTC_MODULE_ENABLED - заглушки, чтобы не пришлось убирать вызовы из остального кода */

/** Заглушка: в проекте не включён HAL_RTC_MODULE_ENABLED, функция недоступна. */
HAL_StatusTypeDef VESC_CAN_SetPositionMemoryEnabled(VESC_Handle_t *h, uint8_t enabled)
{
    (void)h; (void)enabled;
    return HAL_ERROR;
}

/** Заглушка: в проекте не включён HAL_RTC_MODULE_ENABLED, функция недоступна. */
HAL_StatusTypeDef VESC_CAN_SetCurrentPosition(VESC_Handle_t *h, float actual_position_deg)
{
    (void)h; (void)actual_position_deg;
    return HAL_ERROR;
}

#endif /* HAL_RTC_MODULE_ENABLED */

/* ========================================================================
 *  Колбэк приёма телеметрии - публичный API
 * ====================================================================== */

/** Задаёт (или снимает, если callback == NULL) обработчик приёма любого
 *  распознанного статусного пакета. Сама реализация - только присвоение
 *  указателя, сам вызов происходит в конце vesc_decode_status() после
 *  разбора очередного статуса. */
HAL_StatusTypeDef VESC_CAN_SetTelemetryCallback(VESC_Handle_t *h, VESC_TelemetryCallback_t callback)
{
    if (h == NULL) { return HAL_ERROR; }
    h->telemetry_callback = callback;
    return HAL_OK;
}

/* ========================================================================
 *  Кастомный датчик - публичный API
 * ====================================================================== */

/** Задаёт (или снимает, если callback == NULL) обработчик события
 *  кастомного датчика. Сама реализация - только присвоение указателя,
 *  сам вызов происходит в vesc_decode_status() при разборе STATUS_7. */
HAL_StatusTypeDef VESC_CAN_SetCustomSensorCallback(VESC_Handle_t *h, VESC_CustomSensorCallback_t callback)
{
    if (h == NULL) { return HAL_ERROR; }
    h->custom_sensor_callback = callback;
    return HAL_OK;
}

/* ========================================================================
 *  Точки расширения (слабые функции по умолчанию - ничего не делают)
 * ====================================================================== */

/** Слабая заглушка: по умолчанию чужие кадры просто отбрасываются.
 *  Переопределите в своём коде, если на шине есть другие устройства. */
#if defined(VESC_CAN_BACKEND_FDCAN)
__weak void VESC_CAN_OnForeignFrame(VESC_CAN_HandleTypeDef *hcan,
                                     const FDCAN_RxHeaderTypeDef *rxHeader,
                                     const uint8_t *data)
{
    (void)hcan; (void)rxHeader; (void)data;
}
#else
__weak void VESC_CAN_OnForeignFrame(VESC_CAN_HandleTypeDef *hcan,
                                     const CAN_RxHeaderTypeDef *rxHeader,
                                     const uint8_t *data)
{
    (void)hcan; (void)rxHeader; (void)data;
}
#endif

/** Слабая заглушка: по умолчанию ничего не делает при освобождении буфера.
 *  Переопределите, если другому коду тоже нужно "дослать" что-то своё. */
__weak void VESC_CAN_OnTxComplete(VESC_CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}

/* ========================================================================
 *  Обработчики, вызываемые ИЗ ВАШИХ HAL callback-ов
 * ====================================================================== */

static void vesc_decode_status(VESC_Handle_t *h, uint8_t cmd_id, const uint8_t *data)
{
    VESC_Telemetry_t *t = &h->telemetry;

    switch ((VESC_CAN_PacketId_t)cmd_id)
    {
        case VESC_CAN_PACKET_STATUS:
            t->erpm     = (float)be_to_i32(&data[0]);
            t->mech_rpm = t->erpm / (float)h->pole_pairs;
            t->current  = (float)be_to_i16(&data[4]) / 10.0f;
            t->duty     = (float)be_to_i16(&data[6]) / 1000.0f;
            t->rx_mask |= VESC_CAN_RXMASK_STATUS;
            break;

        case VESC_CAN_PACKET_STATUS_2:
            t->amp_hours         = (float)be_to_i32(&data[0]) / 10000.0f;
            t->amp_hours_charged = (float)be_to_i32(&data[4]) / 10000.0f;
            t->rx_mask |= VESC_CAN_RXMASK_STATUS_2;
            break;

        case VESC_CAN_PACKET_STATUS_3:
            t->watt_hours         = (float)be_to_i32(&data[0]) / 10000.0f;
            t->watt_hours_charged = (float)be_to_i32(&data[4]) / 10000.0f;
            t->rx_mask |= VESC_CAN_RXMASK_STATUS_3;
            break;

        case VESC_CAN_PACKET_STATUS_4:
        {
            float raw_pid_pos = (float)be_to_i16(&data[6]) / 50.0f;

            t->temp_fet   = (float)be_to_i16(&data[0]) / 10.0f;
            t->temp_motor = (float)be_to_i16(&data[2]) / 10.0f;
            t->current_in = (float)be_to_i16(&data[4]) / 10.0f;

#if defined(HAL_RTC_MODULE_ENABLED)
            h->last_raw_pid_pos = raw_pid_pos; /* обновляем всегда, даже если функция сейчас выключена -
                                                 * пригодится при последующем включении/ручной калибровке */
            if (h->position_memory_enabled)
            {
                if (h->position_memory_pending_restore)
                {
                    /* Первый кадр после включения функции - подхватываем
                     * офсет так, чтобы совпасть с последним сохранённым
                     * значением ровно в этот момент. */
                    h->position_offset_deg = vesc_posmem_read(h) - raw_pid_pos;
                    h->position_memory_pending_restore = 0U;
                }
                t->pid_pos = vesc_wrap360(raw_pid_pos + h->position_offset_deg);
                vesc_posmem_write(h, t->pid_pos); /* постоянно освежаем сохранённое значение */
            }
            else
            {
                t->pid_pos = raw_pid_pos;
            }
#else
            t->pid_pos = raw_pid_pos;
#endif
            t->rx_mask |= VESC_CAN_RXMASK_STATUS_4;
            break;
        }

        case VESC_CAN_PACKET_STATUS_5:
            t->tachometer = (float)be_to_i32(&data[0]) / 6.0f;
            t->v_in       = (float)be_to_i16(&data[4]) / 10.0f;
            t->rx_mask |= VESC_CAN_RXMASK_STATUS_5;
            break;

        case VESC_CAN_PACKET_STATUS_6:
            t->adc1 = (float)be_to_i16(&data[0]) / 1000.0f;
            t->adc2 = (float)be_to_i16(&data[2]) / 1000.0f;
            t->adc3 = (float)be_to_i16(&data[4]) / 1000.0f;
            t->ppm  = (float)be_to_i16(&data[6]) / 1000.0f;
            t->rx_mask |= VESC_CAN_RXMASK_STATUS_6;
            break;

        case VESC_CAN_PACKET_STATUS_7:
        {
            /* Кастомный статус: data[0] - флаг тормоза (int8_t), data[1] -
             * флаг кастомного датчика (int8_t), data[2:7] - резерв. */
            int8_t brake_flag  = (int8_t)data[0];
            int8_t sensor_flag = (int8_t)data[1];

            if (brake_flag == 0x01)      { t->brake_state = VESC_BRAKE_STATE_ENGAGED; }
            else if (brake_flag == 0x02) { t->brake_state = VESC_BRAKE_STATE_RELEASED; }
            else                         { t->brake_state = VESC_BRAKE_STATE_NONE; }

            if (sensor_flag == 0x01)      { t->custom_sensor_state = VESC_CUSTOM_SENSOR_PIN_SET; }
            else if (sensor_flag == 0x02) { t->custom_sensor_state = VESC_CUSTOM_SENSOR_PIN_RESET; }
            else                          { t->custom_sensor_state = VESC_CUSTOM_SENSOR_NONE; }

            t->rx_mask |= VESC_CAN_RXMASK_STATUS_7;

            /* Колбэк кастомного датчика - см. предупреждение про вызов из
             * прерывания в motor_vesc.h у VESC_CustomSensorCallback_t. */
            if ((t->custom_sensor_state != VESC_CUSTOM_SENSOR_NONE) &&
                (h->custom_sensor_callback != NULL))
            {
                h->custom_sensor_callback(h);
            }
            break;
        }

        default:
            return; /* незнакомая команда - не телеметрия, не трогаем last_rx_tick */
    }

    t->last_rx_tick = HAL_GetTick();

    /* Колбэк приёма телеметрии - см. предупреждение про вызов из прерывания
     * в motor_vesc.h у VESC_TelemetryCallback_t. Вызывается только для
     * реально распознанных пакетов (default выше уже вышел из функции). */
    if (h->telemetry_callback != NULL)
    {
        h->telemetry_callback(h, (VESC_CAN_PacketId_t)cmd_id);
    }
}

/** Обработчик приёма кадров RxFIFO0. Сначала проверяет, что событие вообще
 *  "для него" (та ли шина, и - только на FDCAN - установлен ли бит нового
 *  сообщения), затем вычитывает все накопившиеся кадры и разбирает их. */
void VESC_CAN_RxFifo0_Handler(VESC_CAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
    if ((hcan == NULL) || (bus_find(hcan) == NULL))
    {
        return; /* не наша шина - выходим, не мешаем другим обработчикам */
    }

#if defined(VESC_CAN_BACKEND_FDCAN)
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return; /* сработало другое событие FIFO0, не новое сообщение */
    }
#else
    (void)RxFifo0ITs; /* на bxCAN этот параметр не используется, см. motor_vesc.h */
#endif

    uint32_t ext_id;
    uint8_t  is_ext;
    uint8_t  rxData[8];
    uint8_t  len;

    while (port_receive(hcan, &ext_id, &is_ext, rxData, &len) == HAL_OK)
    {
        if (!is_ext)
        {
            continue; /* нас интересуют только расширенные ID, как у VESC */
        }

        const uint8_t vesc_id = (uint8_t)(ext_id & 0xFFU);
        const uint8_t cmd_id  = (uint8_t)((ext_id >> 8) & 0xFFU);

        VESC_Handle_t *h = vesc_find(hcan, vesc_id);
        if (h == NULL)
        {
            /* Чужой (незарегистрированный) кадр - отдаём наружу с ПОДЛИННЫМ заголовком. */
#if defined(VESC_CAN_BACKEND_FDCAN)
            FDCAN_RxHeaderTypeDef hdr = {0};
            hdr.Identifier = ext_id;
            hdr.IdType     = FDCAN_EXTENDED_ID;
            hdr.DataLength  = port_len_to_dlc(len);
            VESC_CAN_OnForeignFrame(hcan, &hdr, rxData);
#else
            CAN_RxHeaderTypeDef hdr = {0};
            hdr.ExtId = ext_id;
            hdr.IDE   = CAN_ID_EXT;
            hdr.DLC   = len;
            VESC_CAN_OnForeignFrame(hcan, &hdr, rxData);
#endif
            continue;
        }

        vesc_decode_status(h, cmd_id, rxData);
    }
}

/** Обработчик освобождения передающего буфера периферии. Проверяет, что
 *  шина наша, и обходит пул весок ЭТОЙ шины по кругу начиная с
 *  bus->flush_cursor (а не с индекса 0 каждый раз), досылая отложенные
 *  команды. Если место закончилось раньше, чем обошли всех - курсор
 *  запоминает, на ком остановились, чтобы следующий вызов продолжил
 *  именно оттуда - так ни одна веска не будет обделена навсегда. */
void VESC_CAN_TxComplete_Handler(VESC_CAN_HandleTypeDef *hcan)
{
    VESC_Bus_t *bus = bus_find(hcan);
    if (bus == NULL)
    {
        return; /* не наша шина */
    }

    for (uint32_t n = 0U; n < VESC_CAN_MAX_DEVICES; n++)
    {
        uint8_t idx = (uint8_t)((bus->flush_cursor + n) % VESC_CAN_MAX_DEVICES);
        VESC_Handle_t *h = &s_pool[idx];

        if (!h->used || (h->hcan != hcan))
        {
            continue; /* не занят либо веска другой шины */
        }

        if (!vesc_flush_one(hcan, h))
        {
            bus->flush_cursor = idx; /* остановились тут - со следующего вызова продолжим ровно отсюда */
            VESC_CAN_OnTxComplete(hcan);
            return;
        }
    }

    bus->flush_cursor = 0U; /* обошли и обслужили всех - в следующий раз можно начинать сначала */
    VESC_CAN_OnTxComplete(hcan);
}

/* ========================================================================
 *  Имитация весок для отладки без физической CAN-шины
 * ====================================================================== */

#if VESC_CAN_SIM_ENABLE

/* Параметры модели "двигатель без нагрузки" - подобраны только для того,
 * чтобы данные выглядели правдоподобно на стенде/в отладчике, физической
 * точности не имеют и легко правятся под свою задачу. */
#define VESC_SIM_MAX_ERPM           60000.0f  /* эрпм при duty = 1.0                              */
#define VESC_SIM_ACCEL_ERPM_PER_MS    400.0f  /* ограничение скорости разгона/торможения, эрпм/мс */
#define VESC_SIM_IDLE_CURRENT           0.6f  /* ток холостого хода, А                            */
#define VESC_SIM_VIN                   24.0f  /* имитируемое напряжение питания, В                */
#define VESC_SIM_TEMP_AMBIENT           25.0f /* имитируемая температура окружения, °C            */
#define VESC_SIM_BRAKE_ERPM_THRESH      50.0f /* порог |erpm|, ниже которого при нулевой команде считаем, что мотор стоит */

/* Момент последнего вызова VESC_CAN_SimulateTick() - нужен, чтобы посчитать
 * реальную дельту времени между вызовами и не привязываться к тому, из
 * какого именно таймера/с какой частотой её дёргают. Один на всю систему,
 * т.к. шаг физики применяется одинаково ко всем имитируемым вескам сразу. */
static uint32_t s_sim_last_tick = 0U;

/** Считает один шаг физики "двигателя без нагрузки" для ОДНОЙ имитируемой
 *  вески и сразу заполняет всю телеметрию (все 7 статусов) так, как будто
 *  они пришли по CAN одновременно. */
static void vesc_sim_step(VESC_Handle_t *h, float dt_ms)
{
    VESC_Telemetry_t *t = &h->telemetry;
    const float target = (float)h->last_commanded_rpm;

    float max_step = VESC_SIM_ACCEL_ERPM_PER_MS * dt_ms;
    float d = target - t->erpm;
    if (d >  max_step) { d =  max_step; }
    if (d < -max_step) { d = -max_step; }
    t->erpm += d;

    t->duty       = t->erpm / VESC_SIM_MAX_ERPM;
    t->current    = VESC_SIM_IDLE_CURRENT + (0.002f * fabsf(d) / ((dt_ms > 0.0f) ? dt_ms : 1.0f));
    t->current_in = t->current * fabsf(t->duty);
    t->v_in       = VESC_SIM_VIN;

    float temp_target = VESC_SIM_TEMP_AMBIENT + (t->current * 1.5f);
    t->temp_fet   += (temp_target - t->temp_fet)   * 0.01f;
    t->temp_motor += (temp_target - t->temp_motor) * 0.01f;

    float dt_hours = dt_ms / 3600000.0f;
    t->amp_hours  += t->current_in * dt_hours;
    t->watt_hours += t->current_in * VESC_SIM_VIN * dt_hours;

    t->tachometer += t->erpm * (dt_ms / 60000.0f);
    t->pid_pos = fmodf(t->tachometer * 360.0f, 360.0f);
    if (t->pid_pos < 0.0f) { t->pid_pos += 360.0f; }

    t->adc1 = 0.0f;
    t->adc2 = 0.0f;
    t->adc3 = 0.0f;
    t->ppm  = 0.0f;

    if ((fabsf(target) < 1.0f) && (fabsf(t->erpm) < VESC_SIM_BRAKE_ERPM_THRESH))
    {
        t->brake_state = VESC_BRAKE_STATE_ENGAGED;
    }
    else
    {
        t->brake_state = VESC_BRAKE_STATE_RELEASED;
    }

    t->rx_mask |= VESC_CAN_RXMASK_ALL;
    t->last_rx_tick = HAL_GetTick();

    /* Консистентность с реальным приёмом: вызывающему коду не нужно
     * различать имитацию и реальную веску - колбэк телеметрии срабатывает
     * так же, как если бы пришли 7 отдельных кадров (см. VESC_TelemetryCallback_t
     * и предупреждение там про вызов из прерывания - здесь вызывается из
     * VESC_CAN_SimulateTick(), т.е. из того контекста, откуда её вызывает
     * прикладной код). */
    if (h->telemetry_callback != NULL)
    {
        h->telemetry_callback(h, VESC_CAN_PACKET_STATUS);
        h->telemetry_callback(h, VESC_CAN_PACKET_STATUS_2);
        h->telemetry_callback(h, VESC_CAN_PACKET_STATUS_3);
        h->telemetry_callback(h, VESC_CAN_PACKET_STATUS_4);
        h->telemetry_callback(h, VESC_CAN_PACKET_STATUS_5);
        h->telemetry_callback(h, VESC_CAN_PACKET_STATUS_6);
        h->telemetry_callback(h, VESC_CAN_PACKET_STATUS_7);
    }
}

/** Помечает веску как имитируемую/реальную (реальная реализация). */
void VESC_CAN_SetSimulated(VESC_Handle_t *h, uint8_t is_simulated)
{
    if (h != NULL)
    {
        h->simulated = is_simulated ? 1U : 0U;
    }
}

/** Проходит по всем зарегистрированным вескам (на любой шине) и для тех,
 *  что помечены как имитируемые, генерирует свежую телеметрию (реальная
 *  реализация). */
void VESC_CAN_SimulateTick(void)
{
    uint32_t now = HAL_GetTick();
    float dt_ms = (s_sim_last_tick == 0U) ? 1.0f : (float)(now - s_sim_last_tick);
    if (dt_ms <= 0.0f) { dt_ms = 1.0f; }
    s_sim_last_tick = now;

    for (uint32_t i = 0U; i < VESC_CAN_MAX_DEVICES; i++)
    {
        VESC_Handle_t *h = &s_pool[i];
        if (h->used && h->simulated)
        {
            vesc_sim_step(h, dt_ms);
        }
    }
}

#else /* VESC_CAN_SIM_ENABLE == 0 */

/** Заглушка: имитация выключена сборочным define-ом, вызов ничего не делает. */
void VESC_CAN_SetSimulated(VESC_Handle_t *h, uint8_t is_simulated)
{
    (void)h; (void)is_simulated;
}

/** Заглушка: имитация выключена сборочным define-ом, вызов ничего не делает. */
void VESC_CAN_SimulateTick(void)
{
}

#endif /* VESC_CAN_SIM_ENABLE */
