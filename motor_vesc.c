/**
 ******************************************************************************
 * @file    motor_vesc.c
 * @brief   Реализация портируемой библиотеки обмена с VESC по CAN/FDCAN.
 *          См. motor_vesc.h
 * @author  Mechanic
 * @date    12.08.2026
 * @version 1.6
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

/** Контекст одной физической CAN-шины (can_manager) - заменяет старый
 *  VESC_Bus_t после миграции на can_manager. bus_off_count/rx_overflow_count
 *  больше не дублируются здесь - читайте их напрямую с
 *  CANMGR_GetBusOffCount()/GetRxOverflowCount() на самом хэндле шины. */
typedef struct
{
    CANMGR_Handle_t *bus;                 /* NULL - слот свободен */
    uint8_t          built_ins_registered; /* фильтры штатных статусов уже зарегистрированы в can_manager */
    uint8_t          flush_cursor;         /* round-robin индекс досылки ДЛЯ ЭТОЙ шины */
    uint8_t          local_id;             /* см. VESC_CAN_SetLocalId/RequestExists */
    uint8_t          local_id_configured;  /* 0 - VESC_CAN_SetLocalId ещё не звали */

    /* Разные local_id, под которые уже зарегистрирован PONG-фильтр в
     * can_manager (см. vesc_register_pong_filter) - can_manager не даёт
     * снять регистрацию, поэтому при смене local_id старое значение тут
     * остаётся навсегда (безвредно, просто больше никогда не совпадёт). */
    uint8_t          pong_local_ids[VESC_CAN_MAX_LOCAL_IDS_PER_BUS];
    uint8_t          pong_local_id_count;

    /* Кастомные cmd_id (см. VESC_CAN_RegisterCustomStatus), под которые уже
     * зарегистрирован широкий фильтр в can_manager на этой шине - чтобы не
     * пытаться зарегистрировать тот же cmd_id повторно (self-overlap),
     * когда вторая веска на той же шине регистрирует тот же кастомный статус. */
    uint8_t          custom_filter_cmd_ids[VESC_CAN_MAX_CUSTOM_FILTERS_PER_BUS];
    uint8_t          custom_filter_cmd_id_count;
} VESC_BusCtx_t;

/* VESC_Handle_t объявлен целиком в motor_vesc.h (см. пояснение там же) -
 * здесь просто статический пул хэндлов, на которые модуль отдаёт указатели. */
static VESC_Handle_t s_pool[VESC_CAN_MAX_DEVICES];
static VESC_BusCtx_t s_buses[VESC_CAN_MAX_BUSES];

/* ========================================================================
 *  Общие вспомогательные функции
 * ====================================================================== */

/** Ищет уже зарегистрированный хэндл по паре (шина, CAN ID) - используется
 *  ТОЛЬКО внутри модуля для демультиплексирования входящих кадров (RX,
 *  диспетчеризуются can_manager-ом в vesc_dispatch_callback/vesc_pong_dispatch_callback
 *  ниже) и обхода пула на досылке (TX); "снаружи" модуль адресуется через
 *  указатель VESC_Handle_t*, полученный из VESC_CAN_Init(), повторный поиск
 *  по ID на каждый вызов команды не нужен и не делается. */
static VESC_Handle_t *vesc_find(CANMGR_Handle_t *bus, uint8_t vesc_id)
{
    for (uint32_t i = 0U; i < VESC_CAN_MAX_DEVICES; i++)
    {
        if (s_pool[i].used && (s_pool[i].bus == bus) && (s_pool[i].vesc_id == vesc_id))
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

/** Ищет контекст уже известной модулю шины по указателю can_manager. */
static VESC_BusCtx_t *bus_find(CANMGR_Handle_t *bus)
{
    for (uint32_t i = 0U; i < VESC_CAN_MAX_BUSES; i++)
    {
        if (s_buses[i].bus == bus)
        {
            return &s_buses[i];
        }
    }
    return NULL;
}

/** Возвращает контекст шины по bus, создавая новый (в первом свободном
 *  слоте s_buses[]), если такая шина видится впервые. NULL, если исчерпан
 *  VESC_CAN_MAX_BUSES. */
static VESC_BusCtx_t *bus_find_or_alloc(CANMGR_Handle_t *bus)
{
    VESC_BusCtx_t *existing = bus_find(bus);
    if (existing != NULL)
    {
        return existing;
    }

    for (uint32_t i = 0U; i < VESC_CAN_MAX_BUSES; i++)
    {
        if (s_buses[i].bus == NULL)
        {
            memset(&s_buses[i], 0, sizeof(s_buses[i]));
            s_buses[i].bus = bus;
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

/** Безопасно приводит float к int32_t перед отправкой по CAN. Прямое
 *  (int32_t)v в языке Си - undefined behavior, если v не влезает в диапазон
 *  int32_t (переполнение) или равно NaN/бесконечности - а входные параметры
 *  VESC_CAN_SendXxx приходят из кода вызывающей стороны без гарантии, что
 *  они всегда конечны и в разумных пределах (например NaN может возникнуть
 *  где-то выше по стеку из-за деления на 0 в чужом коде и молча дойти сюда).
 *  Здесь - явное насыщение по границам диапазона и явный перевод NaN в 0
 *  (безопасное значение по умолчанию - "останов", а не что-то случайное) -
 *  после этой функции обычный (int32_t) уже гарантированно определённое
 *  поведение, т.к. вход всегда в допустимом диапазоне. */
static int32_t safe_f2i32(float v)
{
    if (isnan(v))            { return 0; }
    if (v >=  2147483648.0f) { return INT32_MAX; } /* 2^31 - ближайшее представимое float сверху от INT32_MAX */
    if (v <= -2147483648.0f) { return INT32_MIN; }
    return (int32_t)v;
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
 *  Разбор статуса (телеметрия) - НЕ изменилось миграцией на can_manager,
 *  см. предварительное объявление ниже (нужно раньше, чем dispatch-колбэки)
 * ====================================================================== */

static void vesc_decode_status(VESC_Handle_t *h, uint8_t cmd_id, const uint8_t *data, uint8_t len);

/* ========================================================================
 *  Приём - callback-и, зарегистрированные в can_manager (CANMGR_RxCallback_t)
 * ====================================================================== */

/** Общий диспетчер приёма для ВСЕХ штатных статусов и всех кастомных
 *  статусов, зарегистрированных через VESC_CAN_RegisterCustomStatus() -
 *  один и тот же callback подходит для обоих случаев, т.к. вся логика
 *  "какой это статус и что с ним делать" уже реализована в
 *  vesc_decode_status() (её собственный default-case делает то же самое,
 *  что раньше делала ветка "код не входит в штатные статусы" в
 *  VESC_CAN_RxFifo0_Handler). Регистрируется с маской 0xFF00 (любой
 *  vesc_id, конкретный cmd_id в фильтре) - см. VESC_CAN_Init()/
 *  RegisterCustomStatus(). Сигнатура - точно CANMGR_RxCallback_t. */
static void vesc_dispatch_callback(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                                    const uint8_t *data, uint8_t len, void *user_ctx)
{
    (void)is_extended; /* фильтр зарегистрирован с is_extended=1 - can_manager это уже гарантировал */
    (void)user_ctx;    /* контекст (VESC_BusCtx_t*) не нужен - поиск ведём по (bus, vesc_id), как раньше */

    const uint8_t vesc_id = (uint8_t)(id & 0xFFU);
    const uint8_t cmd_id  = (uint8_t)((id >> 8) & 0xFFU);

    VESC_Handle_t *h = vesc_find(bus, vesc_id);
    if (h == NULL)
    {
        /* Кадр подошёл под наш широкий (по cmd_id) фильтр, но конкретный
         * vesc_id не наш - тихо отбрасываем. Прямая замена старого пути
         * "незнакомый vesc_id -> VESC_CAN_OnForeignFrame()": теперь такие
         * кадры просто не наши, у can_manager своя философия "не подошло
         * ни под один фильтр - молча отбросить" (см. can_manager.h), это
         * тот же принцип на уровень выше. */
        return;
    }

    vesc_decode_status(h, cmd_id, data, len);
}

/** Диспетчер PONG (VESC_CAN_PACKET_PONG) - зарегистрирован ТОЧНЫМ (exact-
 *  match) фильтром на (PONG<<8)|local_id, см. VESC_CAN_SetLocalId(). PONG
 *  адресуется НЕ по ID ответившей вески (она названа в payload[0]), а по
 *  "нашему" local_id - см. @warning у VESC_CAN_PACKET_PING/PONG в
 *  motor_vesc.h. Прямая замена старой инлайновой проверки в начале
 *  VESC_CAN_RxFifo0_Handler(). */
static void vesc_pong_dispatch_callback(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                                    const uint8_t *data, uint8_t len, void *user_ctx)
{
    (void)id; (void)is_extended; (void)user_ctx;

    if (len < 1U)
    {
        return;
    }

    VESC_Handle_t *ponged = vesc_find(bus, data[0]);
    if (ponged != NULL)
    {
        ponged->exist_status           = VESC_EXIST_CONFIRMED;
        ponged->telemetry.last_rx_tick = HAL_GetTick(); /* реальное доказательство жизни на шине */
    }
}

/* ========================================================================
 *  Регистрация фильтров в can_manager (замена старой port_config_filter)
 * ====================================================================== */

/** Регистрирует (один раз на шину) широкие фильтры (маска 0xFF00, любой
 *  vesc_id) для всех 7 штатных статусов VESC - см. VESC_CAN_Init().
 *  Возвращает HAL_ERROR при первой неудачной регистрации (см. @warning у
 *  VESC_CAN_Init() в motor_vesc.h про честное ограничение - откат уже
 *  зарегистрированных фильтров не реализован, у can_manager для этого нет
 *  API). */
static HAL_StatusTypeDef vesc_register_builtin_filters(CANMGR_Handle_t *bus, VESC_BusCtx_t *bus_ctx)
{
    static const VESC_CAN_PacketId_t builtin_cmds[] = {
        VESC_CAN_PACKET_STATUS,   VESC_CAN_PACKET_STATUS_2, VESC_CAN_PACKET_STATUS_3,
        VESC_CAN_PACKET_STATUS_4, VESC_CAN_PACKET_STATUS_5, VESC_CAN_PACKET_STATUS_6,
        VESC_CAN_PACKET_STATUS_7,
    };

    for (uint32_t i = 0U; i < (sizeof(builtin_cmds) / sizeof(builtin_cmds[0])); i++)
    {
        uint32_t filter_id = ((uint32_t)builtin_cmds[i]) << 8;
        if (CANMGR_RegisterFilter(bus, filter_id, 0xFF00U, 1U, vesc_dispatch_callback, bus_ctx) != CANMGR_REG_OK)
        {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/** Регистрирует (лениво, идемпотентно на конкретный local_id) точный
 *  фильтр приёма PONG под (bus, local_id) - см. VESC_CAN_SetLocalId(). */
static HAL_StatusTypeDef vesc_register_pong_filter(CANMGR_Handle_t *bus, VESC_BusCtx_t *bus_ctx,
                                                     uint8_t local_id)
{
    for (uint32_t i = 0U; i < bus_ctx->pong_local_id_count; i++)
    {
        if (bus_ctx->pong_local_ids[i] == local_id)
        {
            return HAL_OK; /* уже зарегистрирован под этот local_id - идемпотентно */
        }
    }

    if (bus_ctx->pong_local_id_count >= VESC_CAN_MAX_LOCAL_IDS_PER_BUS)
    {
        return HAL_ERROR; /* см. честное ограничение у VESC_CAN_MAX_LOCAL_IDS_PER_BUS в motor_vesc.h */
    }

    uint32_t filter_id = (((uint32_t)VESC_CAN_PACKET_PONG) << 8) | (uint32_t)local_id;
    if (CANMGR_RegisterFilter(bus, filter_id, 0x1FFFFFFFU, 1U, vesc_pong_dispatch_callback, bus_ctx) != CANMGR_REG_OK)
    {
        return HAL_ERROR;
    }

    bus_ctx->pong_local_ids[bus_ctx->pong_local_id_count] = local_id;
    bus_ctx->pong_local_id_count++;
    return HAL_OK;
}

/** Регистрирует (один раз на конкретный кастомный cmd_id на шине) широкий
 *  фильтр под этот cmd_id - см. VESC_CAN_RegisterCustomStatus(). */
static HAL_StatusTypeDef vesc_register_custom_filter(CANMGR_Handle_t *bus, VESC_BusCtx_t *bus_ctx,
                                                        uint8_t cmd_id)
{
    for (uint32_t i = 0U; i < bus_ctx->custom_filter_cmd_id_count; i++)
    {
        if (bus_ctx->custom_filter_cmd_ids[i] == cmd_id)
        {
            return HAL_OK; /* уже зарегистрирован этим или другим vesc-ом на той же шине */
        }
    }

    if (bus_ctx->custom_filter_cmd_id_count >= VESC_CAN_MAX_CUSTOM_FILTERS_PER_BUS)
    {
        return HAL_ERROR; /* см. VESC_CAN_MAX_CUSTOM_FILTERS_PER_BUS в motor_vesc.h */
    }

    uint32_t filter_id = ((uint32_t)cmd_id) << 8;
    if (CANMGR_RegisterFilter(bus, filter_id, 0xFF00U, 1U, vesc_dispatch_callback, bus_ctx) != CANMGR_REG_OK)
    {
        return HAL_ERROR;
    }

    bus_ctx->custom_filter_cmd_ids[bus_ctx->custom_filter_cmd_id_count] = cmd_id;
    bus_ctx->custom_filter_cmd_id_count++;
    return HAL_OK;
}

/* ========================================================================
 *  Программная очередь отложенных команд (по одной веске) - структурно не
 *  изменилась миграцией на can_manager, см. motor_vesc.h
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
 *  команды только что ушло НАПРЯМУЮ в CANMGR_Send(), минуя очередь - иначе
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

/** Пытается протолкнуть в CANMGR_Send() ВСЕ отложенные команды вески h.
 *  Возвращает 1, если очередь этой вески полностью опустела (либо изначально
 *  была пуста), 0 - если CANMGR_Send() отклонил пакет раньше, чем управились
 *  (его собственная программная очередь на шину переполнена - тогда
 *  оставшиеся команды остаются в очереди до следующего вызова). */
static uint8_t vesc_flush_one(CANMGR_Handle_t *bus, VESC_Handle_t *h)
{
    for (uint32_t i = 0U; i < VESC_CAN_MAX_PENDING_PER_VESC; i++)
    {
        if (!h->pending[i].valid)
        {
            continue;
        }

        __disable_irq();
        uint8_t cmd  = h->pending[i].cmd_id;
        int32_t val  = h->pending[i].value;
        h->pending[i].valid = 0U;
        __enable_irq();

        uint8_t payload[4];
        pack_i32_be(payload, val);

        if (CANMGR_Send(bus, make_ext_id((VESC_CAN_PacketId_t)cmd, h->vesc_id), 1U, payload, 4U) != HAL_OK)
        {
            /* CANMGR_Send() отклонил (его программная очередь переполнена,
             * редкий случай под очень высокой нагрузкой) - вернуть значение
             * обратно в очередь, попробуем на следующем opportunистическом flush */
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

/** Общее тело обхода всех весок ОДНОЙ шины по кругу (round-robin), начиная с
 *  bus_ctx->flush_cursor, с попыткой опустошить программную очередь каждой -
 *  вызывается ОПОРТУНИСТИЧЕСКИ из vesc_send_simple() на каждый вызов
 *  VESC_CAN_SendXxx (см. там же). После миграции на can_manager это
 *  ЕДИНСТВЕННЫЙ путь досылки - выделенного "буфер освободился"-обработчика
 *  у этой библиотеки больше нет (это теперь дело can_manager для его СВОЕЙ
 *  очереди, не для нашего дедупликатора - см. motor_vesc.h). Возвращает 1,
 *  если обошли и обслужили всех весок этой шины (курсор сброшен на начало),
 *  0 - если остановились раньше (курсор запомнил, на ком остановились). */
static uint8_t vesc_bus_flush_pending(CANMGR_Handle_t *bus, VESC_BusCtx_t *bus_ctx)
{
    for (uint32_t n = 0U; n < VESC_CAN_MAX_DEVICES; n++)
    {
        uint8_t idx = (uint8_t)((bus_ctx->flush_cursor + n) % VESC_CAN_MAX_DEVICES);
        VESC_Handle_t *h = &s_pool[idx];

        if (!h->used || (h->bus != bus))
        {
            continue; /* не занят либо веска другой шины */
        }

        if (!vesc_flush_one(bus, h))
        {
            bus_ctx->flush_cursor = idx; /* остановились тут - со следующего вызова продолжим ровно отсюда */
            return 0U;
        }
    }

    bus_ctx->flush_cursor = 0U; /* обошли и обслужили всех - в следующий раз можно начинать сначала */
    return 1U;
}

/* ========================================================================
 *  Публичный API - регистрация и телеметрия
 * ====================================================================== */

/** Регистрирует веску по заполненной конфигурации и (один раз на шину)
 *  регистрирует в can_manager фильтры штатных статусов. Подробности -
 *  см. motor_vesc.h. */
VESC_Handle_t *VESC_CAN_Init(const VESC_Config_t *config)
{
    if ((config == NULL) || (config->bus == NULL))
    {
        return NULL;
    }
    if ((config->pole_count == 0U) || ((config->pole_count % 2U) != 0U))
    {
        return NULL; /* число полюсов должно быть чётным и ненулевым */
    }

    /* Идемпотентность: если такая (bus, vesc_id) уже зарегистрирована -
     * просто вернуть существующий хэндл, ничего заново не настраивая. Но
     * если pole_count во втором вызове ОТЛИЧАЕТСЯ от того, что было при
     * первой регистрации - это, скорее всего, ошибка в вызывающем коде
     * (например, конфиг для этой вески в двух местах отличается) - лучше
     * вернуть NULL явной ошибкой, чем молча использовать первое значение
     * и получить незаметно неверный mech_rpm. */
    VESC_Handle_t *existing = vesc_find(config->bus, config->vesc_id);
    if (existing != NULL)
    {
        if (existing->pole_pairs != (config->pole_count / 2U))
        {
            return NULL; /* pole_count разошёлся между повторными вызовами Init для этой же вески */
        }
        return existing;
    }

    VESC_BusCtx_t *bus_ctx = bus_find_or_alloc(config->bus);
    if (bus_ctx == NULL)
    {
        return NULL; /* исчерпан VESC_CAN_MAX_BUSES */
    }

    if (!bus_ctx->built_ins_registered)
    {
        /* См. @warning у VESC_CAN_Init() в motor_vesc.h - при частичном
         * успехе (часть фильтров зарегистрирована, следующий отклонён)
         * фильтры, которые уже встали в can_manager, НЕ отменяются (у
         * can_manager нет такого API в этой версии) - функция всё равно
         * возвращает NULL, честно, не оставляя bus_ctx в "наполовину
         * готовом" состоянии, которое выглядело бы как готовое. */
        if (vesc_register_builtin_filters(config->bus, bus_ctx) != HAL_OK)
        {
            return NULL;
        }
        bus_ctx->built_ins_registered = 1U;
    }

    VESC_Handle_t *h = vesc_find_free_slot();
    if (h == NULL)
    {
        return NULL; /* исчерпан VESC_CAN_MAX_DEVICES */
    }

    memset(h, 0, sizeof(*h));
    /* "used" выставляется ПОСЛЕДНИМ, после всех остальных полей - это слот
     * статического пула, к которому обращается и RX-диспетчер can_manager
     * (см. vesc_find(), может выполняться из прерывания приёма ПАРАЛЛЕЛЬНО
     * этому вызову Init на другом ядре/контексте). Пока used == 0, vesc_find()
     * этот слот не рассматривает вообще - значит vesc_id/bus не могут быть
     * увидены наполовину заполненными. При обратном порядке (used=1 первым)
     * такого реального совпадения по факту не было бы (bus только что
     * обнулён memset-ом и не совпал бы ни с одним настоящим указателем шины),
     * но этот порядок не оставляет такого окна вообще - без учёта того,
     * останется ли это верно после будущих правок структуры. */
    h->vesc_id    = config->vesc_id;
    h->bus        = config->bus;
    h->pole_pairs = config->pole_count / 2U;

#if defined(HAL_RTC_MODULE_ENABLED)
    h->position_memory_hrtc         = config->position_memory_hrtc;
    h->position_memory_backup_index = config->position_memory_backup_index;
#endif

    h->used = 1U; /* публикуем слот ПОСЛЕДНИМ - см. комментарий выше */
    return h;
}

/** Перебор зарегистрированных весок шины bus - см. подробности в motor_vesc.h. */
VESC_Handle_t *VESC_CAN_IterateBus(CANMGR_Handle_t *bus, VESC_Handle_t *prev)
{
    if (bus == NULL)
    {
        return NULL;
    }

    uint32_t start = 0U;
    if (prev != NULL)
    {
        /* prev должен быть NULL либо предыдущим результатом ЭТОЙ ЖЕ функции
         * (см. motor_vesc.h) - если вызывающий код всё же передал указатель
         * не из s_pool (ошибка использования API), защитный bounds-check
         * ниже не даёт вычислить неопределённый индекс/уйти в чужую память -
         * просто считаем, что перебор закончен. */
        if ((prev < &s_pool[0]) || (prev >= &s_pool[VESC_CAN_MAX_DEVICES]))
        {
            return NULL;
        }
        start = (uint32_t)(prev - s_pool) + 1U; /* следующий слот пула ПОСЛЕ prev */
    }

    for (uint32_t i = start; i < VESC_CAN_MAX_DEVICES; i++)
    {
        if (s_pool[i].used && (s_pool[i].bus == bus))
        {
            return &s_pool[i];
        }
    }
    return NULL;
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
 *  Активная проверка присутствия на шине (PING/PONG) - см. motor_vesc.h
 * ====================================================================== */

/** Задаёт "наш" CAN ID для VESC_CAN_RequestExists() на данной шине и лениво
 *  регистрирует точный (exact-match) фильтр приёма PONG под этот local_id. */
HAL_StatusTypeDef VESC_CAN_SetLocalId(CANMGR_Handle_t *bus, uint8_t local_id)
{
    VESC_BusCtx_t *bus_ctx = bus_find(bus);
    if (bus_ctx == NULL)
    {
        return HAL_ERROR; /* шина ещё не зарегистрирована ни одним VESC_CAN_Init() */
    }

    if (vesc_register_pong_filter(bus, bus_ctx, local_id) != HAL_OK)
    {
        return HAL_ERROR;
    }

    bus_ctx->local_id           = local_id;
    bus_ctx->local_id_configured = 1U;
    return HAL_OK;
}

/** Отправляет PING конкретной веске через CANMGR_Send() - см. подробности и
 *  формат payload у VESC_CAN_PACKET_PING в motor_vesc.h. Без программной
 *  очереди-дедупликатора этой библиотеки, как и
 *  VESC_CAN_SendReleaseBrake/SendCustomCommand. */
HAL_StatusTypeDef VESC_CAN_RequestExists(VESC_Handle_t *h)
{
    if (h == NULL)
    {
        return HAL_ERROR;
    }

    VESC_BusCtx_t *bus_ctx = bus_find(h->bus);
    if ((bus_ctx == NULL) || (bus_ctx->local_id_configured == 0U))
    {
        return HAL_ERROR; /* VESC_CAN_SetLocalId() ещё не вызывался для этой шины */
    }

#if VESC_CAN_SIM_ENABLE
    if (h->simulated)
    {
        h->exist_status   = VESC_EXIST_CONFIRMED; /* имитируемая веска "существует" по определению */
        h->ping_sent_tick = HAL_GetTick();
        return HAL_OK;
    }
#endif

    uint8_t payload[1] = { bus_ctx->local_id };

    h->exist_status   = VESC_EXIST_PENDING;
    h->ping_sent_tick  = HAL_GetTick();

    return CANMGR_Send(h->bus, make_ext_id(VESC_CAN_PACKET_PING, h->vesc_id), 1U, payload, 1U);
}

/** Неблокирующий опрос результата последнего VESC_CAN_RequestExists() -
 *  таймаут пересчитывается лениво, здесь же (нет отдельного тика/таймера). */
VESC_ExistStatus_t VESC_CAN_GetExistStatus(VESC_Handle_t *h)
{
    if (h == NULL)
    {
        return VESC_EXIST_UNKNOWN;
    }
    if ((h->exist_status == VESC_EXIST_PENDING) &&
        ((HAL_GetTick() - h->ping_sent_tick) > VESC_CAN_EXIST_TIMEOUT_MS))
    {
        h->exist_status = VESC_EXIST_TIMEOUT;
    }
    return h->exist_status;
}

/* ========================================================================
 *  Команды на веску - общий отправитель + тонкие обёртки
 * ====================================================================== */

/** Общая реализация для всех "простых" (однокадровых, 4 байта) команд:
 *  прямой вызов CANMGR_Send(), иначе - в программную очередь-дедупликатор
 *  этой вески (см. motor_vesc.h - структурно то же, что было до миграции,
 *  только "положить на шину" теперь всегда через CANMGR_Send(), а не через
 *  ручную проверку свободного места в аппаратном буфере + port_send). Для
 *  имитируемых весок реальная передача пропускается (см. VESC_CAN_SetSimulated). */
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

    /* Опортунистическая досылка отложенных команд ЭТОЙ ШИНЫ (не только этой
     * вески) перед своей отправкой - см. подробное обоснование (почему это
     * ЕДИНСТВЕННЫЙ путь досылки после миграции на can_manager) в motor_vesc.h
     * у vesc_bus_flush_pending. */
    VESC_BusCtx_t *bus_ctx = bus_find(h->bus);
    if (bus_ctx != NULL)
    {
        (void)vesc_bus_flush_pending(h->bus, bus_ctx);
    }

    uint8_t payload[4];
    pack_i32_be(payload, scaled);

    /* "Положить на шину" теперь ВСЕГДА - безусловный вызов CANMGR_Send():
     * его HAL_OK трактуем как "доставлено" (очищаем дедуп-слот, даже если
     * физически кадр всё ещё в программной очереди can_manager - это её
     * забота его оттуда вытолкнуть, не наша); его HAL_ERROR трактуем как
     * "программная очередь can_manager прямо сейчас переполнена" - падаем в
     * СВОЙ дедуп-слот, следующий opportunистический flush (из любого
     * VESC_CAN_SendXxx на этой шине) попробует снова. Раньше здесь была
     * ручная проверка port_get_tx_free_level()+port_send() - теперь этим
     * занимается can_manager сам, эта функция про его аппаратный буфер
     * больше ничего не знает и не обязана знать. */
    if (CANMGR_Send(h->bus, make_ext_id(cmd, h->vesc_id), 1U, payload, 4U) == HAL_OK)
    {
        vesc_cancel_pending(h, (uint8_t)cmd); /* свежее значение уже передано - старое отложенное не нужно */
        return HAL_OK;
    }

    return vesc_enqueue_pending(h, (uint8_t)cmd, scaled);
}

/** Duty Cycle напрямую. Масштаб 100000, диапазон -1.0..1.0. */
HAL_StatusTypeDef VESC_CAN_SendDuty(VESC_Handle_t *h, float duty)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_DUTY, safe_f2i32(duty * 100000.0f));
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

    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT, safe_f2i32(current * 1000.0f));
}

/** Тормозной ток ("тормоз мотором"), А. Масштаб 1000. Ограничивается VESC_CAN_SetCurrentLimit. */
HAL_StatusTypeDef VESC_CAN_SendCurrentBrake(VESC_Handle_t *h, float brake_current)
{
    if (h == NULL) { return HAL_ERROR; }
    if (h->current_limit_enabled) { brake_current = clampf(brake_current, h->current_limit); }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_BRAKE, safe_f2i32(brake_current * 1000.0f));
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

    return vesc_send_simple(h, VESC_CAN_PACKET_SET_RPM, safe_f2i32(pid_speed));
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
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_POS, safe_f2i32(position_deg * 1000000.0f));
}

/** Ток относительно максимального. Масштаб 100000, диапазон -1.0..1.0. Не клэмпится VESC_CAN_SetCurrentLimit (unit mismatch, см. motor_vesc.h). */
HAL_StatusTypeDef VESC_CAN_SendCurrentRel(VESC_Handle_t *h, float current_rel)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_REL, safe_f2i32(current_rel * 100000.0f));
}

/** Тормозной ток относительно максимального. Масштаб 100000, диапазон -1.0..1.0. */
HAL_StatusTypeDef VESC_CAN_SendCurrentBrakeRel(VESC_Handle_t *h, float brake_current_rel)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_BRAKE_REL, safe_f2i32(brake_current_rel * 100000.0f));
}

/** "Handbrake"-ток, А. Масштаб 1000. Ограничивается VESC_CAN_SetCurrentLimit. См. предупреждение в motor_vesc.h про неподтверждённую точную семантику. */
HAL_StatusTypeDef VESC_CAN_SendHandbrakeCurrent(VESC_Handle_t *h, float handbrake_current)
{
    if (h == NULL) { return HAL_ERROR; }
    if (h->current_limit_enabled) { handbrake_current = clampf(handbrake_current, h->current_limit); }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE, safe_f2i32(handbrake_current * 1000.0f));
}

/** "Handbrake"-ток относительно максимального. Масштаб 100000, диапазон -1.0..1.0. */
HAL_StatusTypeDef VESC_CAN_SendHandbrakeCurrentRel(VESC_Handle_t *h, float handbrake_current_rel)
{
    if (h == NULL) { return HAL_ERROR; }
    return vesc_send_simple(h, VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE_REL, safe_f2i32(handbrake_current_rel * 100000.0f));
}

/* ========================================================================
 *  Кастомная команда: принудительное отпускание тормоза
 * ====================================================================== */

/** Отправляет 1-байтовую кастомную команду "отпустить тормоз" один раз через
 *  CANMGR_Send() - намеренно НЕ использует vesc_send_simple()/программную
 *  очередь-дедупликатор (см. обоснование в motor_vesc.h). */
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
    return CANMGR_Send(h->bus, make_ext_id(VESC_CAN_PACKET_CUSTOM_BRAKE_CMD, h->vesc_id), 1U, payload, 1U);
}

/* ========================================================================
 *  Произвольные кастомные команды (свой формат сверх протокола VESC)
 * ====================================================================== */

/** Отправляет один кадр с произвольным кодом команды и произвольными
 *  данными через CANMGR_Send() - см. предупреждение про выбор custom_cmd_id
 *  в motor_vesc.h. Как и VESC_CAN_SendReleaseBrake() - без программной
 *  очереди-дедупликатора. */
HAL_StatusTypeDef VESC_CAN_SendCustomCommand(VESC_Handle_t *h, uint8_t custom_cmd_id,
                                              const uint8_t *data, uint8_t len)
{
    if ((h == NULL) || (len > 8U) || ((len > 0U) && (data == NULL)))
    {
        return HAL_ERROR;
    }

#if VESC_CAN_SIM_ENABLE
    if (h->simulated)
    {
        return HAL_OK; /* реального обмена не будет - имитируемая веска */
    }
#endif

    return CANMGR_Send(h->bus, make_ext_id((VESC_CAN_PacketId_t)custom_cmd_id, h->vesc_id), 1U, data, len);
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
 *  функции-заглушки в самом низу этой секции. НЕ ИЗМЕНИЛОСЬ миграцией на
 *  can_manager - работает с RTC_HandleTypeDef, а не с шиной CAN.
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
 *  Произвольные кастомные команды - публичный API (приём)
 * ====================================================================== */

/** Задаёт (или снимает, если callback == NULL) обработчик приёма
 *  произвольного (нераспознанного) кадра от вески. Сама реализация - только
 *  присвоение указателя, сам вызов происходит в vesc_decode_status() в
 *  ветке default (код команды не входит в набор штатных статусов). */
HAL_StatusTypeDef VESC_CAN_SetCustomCommandCallback(VESC_Handle_t *h, VESC_CustomCommandCallback_t callback)
{
    if (h == NULL) { return HAL_ERROR; }
    h->custom_command_callback = callback;
    return HAL_OK;
}

/* ========================================================================
 *  Произвольные кастомные СТАТУСЫ - публичный API
 * ====================================================================== */

/** Регистрирует/переиспользует слот реестра кастомных статусов для одного
 *  cmd_id - сам вызов колбэка происходит в vesc_decode_status() в ветке
 *  default, ПЕРЕД тем как код упадёт в custom_command_callback. После
 *  миграции на can_manager - также лениво регистрирует широкий фильтр
 *  приёма под этот cmd_id на шине вески (см. vesc_register_custom_filter),
 *  если он ещё не был зарегистрирован (этой либо другой веской на той же
 *  шине). */
HAL_StatusTypeDef VESC_CAN_RegisterCustomStatus(VESC_Handle_t *h, uint8_t cmd_id,
                                                 VESC_CustomStatusCallback_t callback)
{
    if ((h == NULL) || (callback == NULL))
    {
        return HAL_ERROR;
    }

    switch ((VESC_CAN_PacketId_t)cmd_id)
    {
        case VESC_CAN_PACKET_STATUS:
        case VESC_CAN_PACKET_STATUS_2:
        case VESC_CAN_PACKET_STATUS_3:
        case VESC_CAN_PACKET_STATUS_4:
        case VESC_CAN_PACKET_STATUS_5:
        case VESC_CAN_PACKET_STATUS_6:
        case VESC_CAN_PACKET_STATUS_7:
            return HAL_ERROR; /* уже штатный статус, регистрировать поверх него нельзя */
        default:
            break;
    }

    VESC_BusCtx_t *bus_ctx = bus_find(h->bus);
    if (bus_ctx == NULL)
    {
        return HAL_ERROR; /* не должно происходить - h->bus всегда известен модулю после VESC_CAN_Init() */
    }
    if (vesc_register_custom_filter(h->bus, bus_ctx, cmd_id) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Тот же cmd_id уже зарегистрирован НА ЭТОЙ ВЕСКЕ - просто заменяем колбэк, слот тот же. */
    for (uint32_t i = 0U; i < VESC_CAN_MAX_CUSTOM_STATUSES; i++)
    {
        if ((h->custom_statuses[i].used != 0U) && (h->custom_statuses[i].cmd_id == cmd_id))
        {
            h->custom_statuses[i].callback = callback;
            return HAL_OK;
        }
    }

    for (uint32_t i = 0U; i < VESC_CAN_MAX_CUSTOM_STATUSES; i++)
    {
        if (h->custom_statuses[i].used == 0U)
        {
            h->custom_statuses[i].used     = 1U;
            h->custom_statuses[i].cmd_id   = cmd_id;
            h->custom_statuses[i].callback = callback;
            return HAL_OK;
        }
    }

    return HAL_ERROR; /* исчерпан VESC_CAN_MAX_CUSTOM_STATUSES */
}

/** "Пришли мне кастомный статус cmd_id прямо сейчас" - тонкая обёртка над
 *  VESC_CAN_SendCustomCommand(), см. предупреждение про штатные статусы
 *  1-6 в motor_vesc.h. */
HAL_StatusTypeDef VESC_CAN_RequestCustomStatus(VESC_Handle_t *h, uint8_t cmd_id)
{
    uint8_t payload[1] = { cmd_id };
    return VESC_CAN_SendCustomCommand(h, VESC_CAN_PACKET_CUSTOM_STATUS_REQUEST, payload, 1U);
}

/* ========================================================================
 *  Разбор статуса (телеметрия) - НЕ изменилось миграцией на can_manager:
 *  вызывается из vesc_dispatch_callback() выше, а не из ветки RxFifo0_Handler,
 *  но сама логика (парсинг, губернаторы косвенно через telemetry, память
 *  положения, реестр кастомных статусов, последний default-случай) осталась
 *  ровно той же, что и до миграции - ей не важно, как физически пришёл кадр.
 * ====================================================================== */

static void vesc_decode_status(VESC_Handle_t *h, uint8_t cmd_id, const uint8_t *data, uint8_t len)
{
    VESC_Telemetry_t *t = &h->telemetry;

    /* Штатные статусы протокола VESC (и наш кастомный №7) - ВСЕГДА ровно
     * 8 байт payload. Кадр с ТЕМ ЖЕ cmd_id, но короче - не настоящий
     * статус (ошибка на шине, либо чужой протокол, случайно совпавший
     * кодом команды) - обязательно проверяем ДО того, как разборы ниже
     * начнут читать data[] до смещения 6-7: буфер, который нам передают
     * (см. can_manager.h/CANMGR_RxCallback_t), физически всегда 8 байт
     * (не UB, не выход за границы памяти), но байты ЗА реальной длиной
     * кадра НЕ инициализированы ЭТИМ приёмом - это могут быть остатки
     * ПРЕДЫДУЩЕГО кадра из того же буфера. Без этой проверки короткий/битый
     * кадр тихо подмешал бы в телеметрию (включая erpm/current, которые
     * читают губернаторы в VESC_CAN_SendCurrent/SendSpeed) чужие устаревшие
     * значения вместо явной ошибки. */
    switch ((VESC_CAN_PacketId_t)cmd_id)
    {
        case VESC_CAN_PACKET_STATUS:
        case VESC_CAN_PACKET_STATUS_2:
        case VESC_CAN_PACKET_STATUS_3:
        case VESC_CAN_PACKET_STATUS_4:
        case VESC_CAN_PACKET_STATUS_5:
        case VESC_CAN_PACKET_STATUS_6:
        case VESC_CAN_PACKET_STATUS_7:
            if (len < 8U)
            {
                return; /* короче штатного пакета - молча игнорируем, telemetry не трогаем */
            }
            break;
        default:
            break; /* кастомные статусы/команды - произвольная длина, см. ветку default ниже */
    }

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
        {
            /* Сначала - реестр кастомных статусов (VESC_CAN_RegisterCustomStatus).
             * Это ТЕЛЕМЕТРИЯ (в отличие от произвольной команды ниже) - падаем
             * дальше на last_rx_tick/telemetry_callback как штатные статусы. */
            uint8_t handled = 0U;
            for (uint32_t i = 0U; i < VESC_CAN_MAX_CUSTOM_STATUSES; i++)
            {
                if ((h->custom_statuses[i].used != 0U) && (h->custom_statuses[i].cmd_id == cmd_id))
                {
                    if (h->custom_statuses[i].callback != NULL)
                    {
                        h->custom_statuses[i].callback(h, cmd_id, data, len);
                    }
                    handled = 1U;
                    break;
                }
            }
            if (handled)
            {
                break;
            }

            /* Не входит ни в штатные статусы, ни в реестр кастомных статусов -
             * это намеренно произвольная кастомная команда пользователя - см.
             * VESC_CAN_SetCustomCommandCallback/VESC_CustomCommandCallback_t
             * в motor_vesc.h. */
            if (h->custom_command_callback != NULL)
            {
                h->custom_command_callback(h, cmd_id, data, len);
            }
            return;
        }
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
