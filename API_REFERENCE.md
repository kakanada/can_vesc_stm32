# motor_vesc — справочник по API

Полный список публичных функций и типов библиотеки `motor_vesc` (+ модуля моста `vesc_bridge`) с
описанием. Для быстрого повторения архитектуры — см. `README.md`, для честных ограничений и
обоснований конкретных решений — комментарии в `motor_vesc.h` и, для моста, в
`vesc_bridge.h`/`BRIDGE_PROTOCOL.md` (справочник ниже — их сжатый и структурированный пересказ, при
расхождениях ориентируйтесь на .h-файлы, они первичны).

## Оглавление

- [Типы](#типы)
- [Регистрация и телеметрия](#регистрация-и-телеметрия)
- [Проверка присутствия на шине (PING/PONG)](#проверка-присутствия-на-шине-pingpong)
- [Команды на веску](#команды-на-веску)
- [Кастомная команда: отпускание тормоза](#кастомная-команда-отпускание-тормоза)
- [Произвольные кастомные команды](#произвольные-кастомные-команды)
- [Произвольные кастомные статусы](#произвольные-кастомные-статусы)
- [Программные ограничения (губернаторы)](#программные-ограничения-губернаторы)
- [Память положения](#память-положения)
- [Обработчики прерываний](#обработчики-прерываний)
- [Диагностика и восстановление шины](#диагностика-и-восстановление-шины)
- [Точки расширения и колбэки](#точки-расширения-и-колбэки)
- [Имитация](#имитация)
- [Мост VESC Tool ↔ CAN (vesc_bridge.h)](#мост-vesc-tool--can-vesc_bridgeh)

---

## Типы

### `VESC_CAN_HandleTypeDef`
Универсальный тип хэндла CAN-шины, автоматически равен `FDCAN_HandleTypeDef` (бэкенд FDCAN) либо
`CAN_HandleTypeDef` (бэкенд bxCAN) в зависимости от того, какой HAL-модуль включён в проекте.

### `VESC_Config_t`
Структура конфигурации для регистрации вески — заполняется перед вызовом `VESC_CAN_Init()`.

| Поле | Тип | Описание |
|---|---|---|
| `hcan` | `VESC_CAN_HandleTypeDef*` | шина, на которой сидит веска |
| `vesc_id` | `uint8_t` | CAN ID вески (0..255, задаётся в VESC Tool) |
| `pole_count` | `uint8_t` | число полюсов мотора (чётное), для пересчёта erpm↔mech_rpm; укажите 2, если механические RPM не нужны |
| `position_memory_hrtc` | `RTC_HandleTypeDef*` | (только если `HAL_RTC_MODULE_ENABLED`) хэндл RTC для памяти положения, NULL если не используется |
| `position_memory_backup_index` | `uint32_t` | (только если `HAL_RTC_MODULE_ENABLED`) индекс backup-регистра RTC (резервируются индекс и индекс+1) |

### `VESC_Handle_t`
Хэндл вески, который `VESC_CAN_Init()` возвращает по указателю. Публично для чтения: `hcan`,
`vesc_id`, `telemetry`. Остальные поля — внутреннее состояние модуля, менять напрямую не нужно (для
этого есть соответствующие функции).

### `VESC_Telemetry_t`
Актуальная телеметрия вески (поле `h->telemetry`), обновляется библиотекой автоматически при приёме
статусных пакетов.

| Поле | Пакет | Единица |
|---|---|---|
| `erpm` | STATUS | эл. RPM |
| `mech_rpm` | STATUS (расчётное) | RPM на валу |
| `current` | STATUS | А |
| `duty` | STATUS | 0..1 |
| `amp_hours`, `amp_hours_charged` | STATUS_2 | Ач |
| `watt_hours`, `watt_hours_charged` | STATUS_3 | Втч |
| `temp_fet`, `temp_motor` | STATUS_4 | °C |
| `current_in` | STATUS_4 | А |
| `pid_pos` | STATUS_4 | градусы (с учётом офсета памяти положения, если включена) |
| `tachometer` | STATUS_5 | EREV |
| `v_in` | STATUS_5 | В |
| `adc1`, `adc2`, `adc3`, `ppm` | STATUS_6 | — |
| `brake_state` | STATUS_7 (кастомный) | `VESC_BrakeState_t` |
| `custom_sensor_state` | STATUS_7 (кастомный) | `VESC_CustomSensorState_t` |
| `last_rx_tick` | — | `HAL_GetTick()` последнего любого статуса |
| `rx_mask` | — | битовая маска пришедших пакетов, см. `VESC_CAN_RXMASK_*` |

### `VESC_BrakeState_t`
`VESC_BRAKE_STATE_NONE` (0) / `VESC_BRAKE_STATE_ENGAGED` (1, зажат) / `VESC_BRAKE_STATE_RELEASED`
(2, разжат).

### `VESC_CustomSensorState_t`
`VESC_CUSTOM_SENSOR_NONE` (0) / `VESC_CUSTOM_SENSOR_PIN_SET` (1) / `VESC_CUSTOM_SENSOR_PIN_RESET`
(2).

### `VESC_TelemetryCallback_t` / `VESC_CustomSensorCallback_t`
Типы функций-колбэков — см. секцию [Точки расширения и колбэки](#точки-расширения-и-колбэки).

### `VESC_CustomCommandCallback_t`
Тип колбэка для приёма произвольных кастомных команд — см. [Произвольные кастомные
команды](#произвольные-кастомные-команды).

### `VESC_ExistStatus_t`
Результат проверки физического присутствия на шине — см. [Проверка присутствия на шине
(PING/PONG)](#проверка-присутствия-на-шине-pingpong). `VESC_EXIST_UNKNOWN` (0, запрос не
отправлялся) / `VESC_EXIST_PENDING` (1, ждём ответа) / `VESC_EXIST_CONFIRMED` (2, PONG получен) /
`VESC_EXIST_TIMEOUT` (3, не ответила за отведённое время).

### `VESC_CustomStatusCallback_t`
Тип колбэка для приёма ОДНОГО зарегистрированного кастомного статуса — см. [Произвольные кастомные
статусы](#произвольные-кастомные-статусы).

---

## Регистрация и телеметрия

### `VESC_Handle_t *VESC_CAN_Init(const VESC_Config_t *config)`
Регистрирует веску, при первой регистрации на данной шине настраивает фильтр/нотификации/запуск
периферии. Возвращает указатель на хэндл (хранить у себя и использовать во всех остальных вызовах)
либо `NULL` при ошибке (некорректный `config`, исчерпаны лимиты `VESC_CAN_MAX_DEVICES`/
`VESC_CAN_MAX_BUSES`, либо при повторном вызове для уже известной пары hcan+vesc_id — несовпадение
`pole_count`). Повторный вызов с теми же параметрами идемпотентен.

### `const VESC_Telemetry_t *VESC_CAN_GetTelemetry(VESC_Handle_t *h)`
Возвращает `&h->telemetry`. Чисто для удобства — эквивалентно прямому обращению к полю.

### `uint8_t VESC_CAN_IsAlive(VESC_Handle_t *h, uint32_t timeout_ms)`
1, если хоть один статусный пакет пришёл за последние `timeout_ms`, иначе 0.

### `VESC_Handle_t *VESC_CAN_IterateBus(VESC_CAN_HandleTypeDef *hcan, VESC_Handle_t *prev)`
Перебор зарегистрированных весок конкретной шины — итератор (`prev == NULL` для начала, `NULL` в
ответ — весок больше нет). Для расширений вроде `vesc_bridge.h`, которым нужен весь список, а не
одна веска.

---

## Проверка присутствия на шине (PING/PONG)

В отличие от `VESC_CAN_IsAlive()` (пассивная — смотрит, приходили ли статусы сами, по расписанию
вески), эта пара функций АКТИВНО запрашивает конкретную веску через официальный PING/PONG протокола
VESC. Полезно, если у вески отключена периодическая отправка статусов в VESC Tool.

### `HAL_StatusTypeDef VESC_CAN_SetLocalId(VESC_CAN_HandleTypeDef *hcan, uint8_t local_id)`
Задаёт "наш" CAN ID для проверки присутствия на данной шине. Вызывается один раз на шину, до первого
`VESC_CAN_RequestExists()`, после хотя бы одного `VESC_CAN_Init()` на этой шине. Обязателен, так как
PONG в протоколе VESC адресуется не по ID запрошенной вески, а по ID, вложенному в тело самого
PING-а. `local_id` не должен совпадать ни с одной веской/мостом на шине. `HAL_ERROR`, если шина ещё
не зарегистрирована ни одним `VESC_CAN_Init()`.

### `HAL_StatusTypeDef VESC_CAN_RequestExists(VESC_Handle_t *h)`
Неблокирующая отправка PING конкретной веске. Требует предварительного `VESC_CAN_SetLocalId()` для
этой шины (иначе `HAL_ERROR`). `HAL_BUSY`, если аппаратный буфер занят прямо сейчас (без программной
очереди — как и `VESC_CAN_SendReleaseBrake`/`SendCustomCommand`).

### `VESC_ExistStatus_t VESC_CAN_GetExistStatus(VESC_Handle_t *h)`
Неблокирующий опрос результата последнего `VESC_CAN_RequestExists()`. Таймаут —
`VESC_CAN_EXIST_TIMEOUT_MS` (конфигурация модуля, по умолчанию 200 мс). Подтверждённый PONG также
обновляет `h->telemetry.last_rx_tick` — `VESC_CAN_IsAlive()` увидит веску живой сразу после
PING/PONG, даже если статусы у неё полностью отключены.

---

## Команды на веску

Общий принцип для всех функций ниже: неблокирующие; если аппаратный буфер периферии полон — значение
уходит в программную очередь этой конкретной команды этой конкретной вески и досылается по
освобождению буфера, по кругу между вескими на шине; повторный вызов до отправки просто заменяет
значение. Для имитируемых весок реальная отправка не выполняется. Возвращают `HAL_OK` (ушло),
`HAL_BUSY` (отложено), `HAL_ERROR` (`h == NULL` либо переполнена очередь конкретной команды).

Гарантия по float-параметру: NaN/Inf/значение вне диапазона `int32_t` не приводят к падению или
undefined behavior — NaN превращается в 0, выход за диапазон насыщается до соответствующей границы.
Подробности — в `motor_vesc.h`, раздел "Команды на веску".

| Функция | Параметр | Диапазон/масштаб | Ограничивается губернатором |
|---|---|---|---|
| `VESC_CAN_SendDuty(h, duty)` | скважность | -1.0..1.0 | нет |
| `VESC_CAN_SendCurrent(h, current)` | ток, А | — | `current_limit` (прямо) + `speed_limit` (перекрёстно) |
| `VESC_CAN_SendCurrentBrake(h, brake_current)` | тормозной ток, А ("тормоз мотором") | — | `current_limit` |
| `VESC_CAN_SendSpeed(h, pid_speed)` | скорость, эл. RPM | — | `speed_limit` (прямо) + `current_limit` (перекрёстно) |
| `VESC_CAN_SendMechanicalSpeed(h, mech_rpm)` | скорость, мех. RPM | пересчитывается в erpm через `pole_count` | те же, что у SendSpeed (в erpm!) |
| `VESC_CAN_SendPosition(h, position_deg)` | позиция, градусы | 0..360 | нет |
| `VESC_CAN_SendCurrentRel(h, current_rel)` | ток относительно максимума | -1.0..1.0 | нет (несовместимые единицы) |
| `VESC_CAN_SendCurrentBrakeRel(h, brake_current_rel)` | тормозной ток относительно максимума | -1.0..1.0 | нет |
| `VESC_CAN_SendHandbrakeCurrent(h, handbrake_current)` | handbrake-ток, А (точная семантика не подтверждена) | — | `current_limit` |
| `VESC_CAN_SendHandbrakeCurrentRel(h, handbrake_current_rel)` | handbrake-ток относительно максимума | -1.0..1.0 | нет |

Для "не дать валу провернуться от вибрации" рекомендуется именно `VESC_CAN_SendCurrentBrake()` — её
семантика подтверждена официальной таблицей команд протокола VESC.

> **Мотор не реагирует на `SendSpeed`/`SendMechanicalSpeed`, хотя на `SendCurrent` реагирует
> нормально?** Пакет отправляется корректно (сверено построчно с прошивкой VESC) — почти наверняка
> дело в ненастроенном Speed PID и/или пороге Minimum ERPM (~900 по умолчанию) в VESC Tool → Motor
> Settings → PID Controllers → Speed Controller. Подробности — у объявления `VESC_CAN_SendSpeed` в
> `motor_vesc.h`.

---

## Кастомная команда: отпускание тормоза

### `HAL_StatusTypeDef VESC_CAN_SendReleaseBrake(VESC_Handle_t *h)`
Кастомная (не входит в официальный протокол VESC) 1-байтовая команда, парная к Lisp-скрипту
`vesc_ppm_universal.lisp`. Отправляет "отпустить тормоз" **один раз** — библиотека НЕ повторяет её
сама. На стороне вески действует watchdog (по умолчанию 500 мс): чтобы тормоз оставался отпущенным
дольше, вызывать функцию нужно периодически самому. В отличие от команд выше, НЕ проходит через
программную очередь — при занятом буфере просто возвращает `HAL_BUSY` без отложенной досылки.

---

## Произвольные кастомные команды

Для собственного протокола сверх официального VESC (например обмен со своим Lisp-скриптом), когда
заранее готовых `VESC_CAN_SendReleaseBrake`/ `VESC_CAN_PACKET_STATUS_7` недостаточно — полностью
произвольный код команды и произвольные данные (0..8 байт) в обе стороны. Как и `SendReleaseBrake`,
не проходит через программную очередь.

> **Внимание**: код команды (`custom_cmd_id`) — то же 8-битное поле, что и у штатных команд
> протокола VESC (`VESC_CAN_PacketId_t`). Если случайно выбрать значение, уже занятое официальной
> командой (например `SET_RPM`) — настоящая прошивка вески применит присланные байты именно как эту
> настоящую команду. Выбирайте значение, не входящее в `VESC_CAN_PacketId_t`.

### `HAL_StatusTypeDef VESC_CAN_SendCustomCommand(VESC_Handle_t *h, uint8_t custom_cmd_id, const uint8_t *data, uint8_t len)`
Отправляет один кадр с произвольным кодом команды и данными веске. `len` — 0..8 (классический
CAN-кадр), `HAL_ERROR` при превышении. `HAL_BUSY` при занятом аппаратном буфере, без досылки (как у
`SendReleaseBrake`).

### `HAL_StatusTypeDef VESC_CAN_SetCustomCommandCallback(VESC_Handle_t *h, VESC_CustomCommandCallback_t callback)`
Задаёт обработчик, вызываемый при получении от вески кадра, код команды которого НЕ входит в набор
штатных статусов (`VESC_CAN_PACKET_STATUS` .. `STATUS_7`) и НЕ зарегистрирован через
`VESC_CAN_RegisterCustomStatus()` (см. ниже) — сигнатура `void(*)(VESC_Handle_t *h, uint8_t
custom_cmd_id, const uint8_t *data, uint8_t len)`. `NULL` отключает. **Вызывается из прерывания** —
быстрый, неблокирующий код; `data`/`len` валидны только на время вызова.

---

## Произвольные кастомные статусы

`VESC_CAN_PACKET_STATUS_7` (`brake_state`/`custom_sensor_state`) — один конкретный, заранее
оформленный кастомный статус со своими полями в `VESC_Telemetry_t`. Если нужно несколько СВОИХ
кастомных статусов (разных ID, разных payload) — регистрируйте каждый под своим `cmd_id` со своим
отдельным колбэком-парсером; парсинг конкретного payload можно оформить в отдельном файле проекта.

В отличие от `VESC_CAN_SetCustomCommandCallback()` (один общий колбэк на все незнакомые коды) — тут
у каждого статуса свой колбэк, и приём считается ТЕЛЕМЕТРИЕЙ: обновляет `last_rx_tick` и вызывает
`VESC_TelemetryCallback_t`, как штатные статусы 1-7.

> **Внимание про выбор `cmd_id`**: то же 8-битное поле, что у официальных команд протокола VESC.
> Библиотека уже занимает 200-202 (`VESC_CAN_PACKET_STATUS_7`/`CUSTOM_BRAKE_CMD`/
> `CUSTOM_STATUS_REQUEST`) — для своих статусов используйте диапазон 210+.

### `HAL_StatusTypeDef VESC_CAN_RegisterCustomStatus(VESC_Handle_t *h, uint8_t cmd_id, VESC_CustomStatusCallback_t callback)`
Регистрирует обработчик для одного кастомного статуса. Повторная регистрация того же `cmd_id`
заменяет колбэк (не расходует новый слот). `HAL_ERROR`, если `callback == NULL`, `cmd_id` совпал со
штатным статусом (`STATUS`..`STATUS_7`), либо свободных слотов не осталось (см.
`VESC_CAN_MAX_CUSTOM_STATUSES`, конфигурация модуля, по умолчанию 4 на веску).

### `HAL_StatusTypeDef VESC_CAN_RequestCustomStatus(VESC_Handle_t *h, uint8_t cmd_id)`
"Пришли мне кастомный статус `cmd_id` прямо сейчас", не дожидаясь периодического цикла на стороне
вески (`VESC_CAN_PACKET_CUSTOM_STATUS_REQUEST`). Обёртка над `VESC_CAN_SendCustomCommand()`.
Работает только если Lisp-скрипт на стороне вески явно реализует приём этого запроса (см. раздел 6
`vesc_ppm_universal.lisp`) — **для штатных статусов 1-6 такого механизма в самой прошивке VESC
нет**, они шлются только периодически (частота настраивается в VESC Tool).

---

## Программные ограничения (губернаторы)

Честно: в протоколе VESC нет команды "временно ограничить максимум скорости/тока", поэтому обе
функции ниже реализованы как обратная связь по телеметрии на стороне STM32, а не в прошивке вески —
со всеми вытекающими оговорками (задержка реакции по частоте статусов, возможен небольшой заброс за
предел, прошитые в самой веске пределы остаются абсолютным потолком). Подробности — в
`motor_vesc.h`.

### `HAL_StatusTypeDef VESC_CAN_SetSpeedLimit(VESC_Handle_t *h, float max_abs_erpm, float governor_margin_erpm)`
Включает губернатор скорости: прямой clamp для `SendSpeed`, перекрёстный потолок для `SendCurrent`.
`governor_margin_erpm` — полоса сглаживания перекрёстного губернатора (автоматически ограничивается
сверху пределом).

### `HAL_StatusTypeDef VESC_CAN_ClearSpeedLimit(VESC_Handle_t *h)`
Снимает ограничение.

### `HAL_StatusTypeDef VESC_CAN_SetCurrentLimit(VESC_Handle_t *h, float max_abs_current, float governor_margin_current)`
Включает губернатор тока: прямой clamp для `SendCurrent`/`SendCurrentBrake`/ `SendHandbrakeCurrent`,
перекрёстный потолок для `SendSpeed`.

### `HAL_StatusTypeDef VESC_CAN_ClearCurrentLimit(VESC_Handle_t *h)`
Снимает ограничение.

Оба перекрёстных губернатора учитывают направление: throttle срабатывает только когда запрошенное
значение реально толкает вал/ток дальше за предел — торможение, реверс и снижение скорости никогда
не придушиваются.

---

## Память положения

Доступно только если в проекте включён `HAL_RTC_MODULE_ENABLED` (иначе обе функции — заглушки,
всегда возвращающие `HAL_ERROR`, вызовы можно не убирать из кода). Сохраняет скорректированный
`pid_pos` в backup-регистры RTC, чтобы после выключения/включения вески (датчики Холла могут выдать
другое значение при физически неподвижном вале) система координат не съезжала. Для сохранения через
полное обесточивание платы нужна батарея на VBAT — подробности в `motor_vesc.h`.

### `HAL_StatusTypeDef VESC_CAN_SetPositionMemoryEnabled(VESC_Handle_t *h, uint8_t enabled)`
Включает/выключает функцию. При включении подхватывает офсет от последнего сохранённого значения
(сразу, если уже есть телеметрия, либо на первом же следующем `STATUS_4`).

### `HAL_StatusTypeDef VESC_CAN_SetCurrentPosition(VESC_Handle_t *h, float actual_position_deg)`
Ручная калибровка "на лету" — говорит модулю, какой угол сейчас на самом деле, пересчитывает офсет и
сразу сохраняет. Требует, чтобы уже приходил хотя бы один `STATUS_4`. Неявно включает память
положения.

---

## Обработчики прерываний

Вызываются не сами по себе, а из **ваших** HAL callback-ов — см. таблицу в `motor_vesc.h` (шпаргалка
по FDCAN/bxCAN). Каждый сам проверяет, относится ли событие к нему, и тихо выходит, если нет.

### `void VESC_CAN_RxFifo0_Handler(VESC_CAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)`
Разбирает принятые кадры RxFIFO0, обновляет телеметрию соответствующей вески. Второй параметр значим
только для FDCAN, на bxCAN передавайте 0.

### `void VESC_CAN_TxComplete_Handler(VESC_CAN_HandleTypeDef *hcan)`
Досылает отложенные команды весок этой шины по кругу (round-robin). Не единственный путь досылки —
каждый `VESC_CAN_SendXxx` тоже опортунистически досылает очередь при каждом вызове (см.
`motor_vesc.h`), поэтому очередь не зависает, даже если это прерывание по какой-то причине не
пришло.

---

## Диагностика и восстановление шины

Честно: сама по себе загрузка шины корректными кадрами (даже на пределе пропускной способности) не
может "уронить" CAN — единственный механизм, из-за которого шина полностью отключает ВСЕХ узлов
сразу — это Bus-Off, следствие реальных ошибок кадров (несовпадающий битовый тайминг у узлов,
неисправный/не тот узел на шине, отсутствующий/лишний терминатор, помехи). Обработчик ниже
**обнаруживает и автоматически восстанавливает** периферию после Bus-Off — FDCAN не выходит из него
сам никогда, bxCAN без опции ABOM в CubeMX тоже. До появления этой секции восстановление после
Bus-Off требовало перезагрузки МК. Подробности — в `motor_vesc.h`.

### `void VESC_CAN_ErrorStatus_Handler(hcan[, ErrorStatusITs])` — вызывается из вашего HAL callback-а
Bus-Off: инкрементирует счётчик и немедленно восстанавливает периферию. Сигнатура для FDCAN —
`(hcan, ErrorStatusITs)`, вызывать из `HAL_FDCAN_ErrorStatusCallback`; для bxCAN — `(hcan)`,
вызывать из `HAL_CAN_ErrorCallback`. Также считает переполнения Rx FIFO0.

### `uint32_t VESC_CAN_GetBusOffCount(VESC_CAN_HandleTypeDef *hcan)`
Сколько раз шина уходила в Bus-Off (и была восстановлена) с момента старта. Растущее значение —
признак физической/конфигурационной проблемы на шине, не в библиотеке.

### `uint32_t VESC_CAN_GetRxOverflowCount(VESC_CAN_HandleTypeDef *hcan)`
Сколько раз переполнялся аппаратный Rx FIFO0 (кадры потеряны, программа не успела их вычитать). В
штатной работе должно быть 0.

---

## Точки расширения и колбэки

### `void VESC_CAN_OnForeignFrame(VESC_CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t len)` — слабая функция
Вызывается для кадров, не опознанных как статус зарегистрированной вески (в т.ч. кадры моста
`vesc_bridge.h` — см. `VESC_Bridge_OnCanFrame`). Переопределите в своём коде для других устройств на
этой же шине. **[Ломающее изменение, версия 1.3]** раньше вторым параметром был указатель на
backend-специфичный заголовок (`FDCAN_RxHeaderTypeDef*`/`CAN_RxHeaderTypeDef*`) — теперь простые
`ext_id`+`len`, см. `motor_vesc.h`.

### `void VESC_CAN_OnTxComplete(hcan)` — слабая функция
Вызывается в конце `VESC_CAN_TxComplete_Handler()`. Переопределите, если другому коду тоже нужно
"дослать" что-то в освободившийся буфер.

### `HAL_StatusTypeDef VESC_CAN_SendRawFrame(VESC_CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t len)`
Отправляет один сырой CAN-кадр напрямую в периферию, в обход программной очереди/round-robin весок —
примитив для расширений (используется `vesc_bridge.h` для форвардинга многокадровых команд). Для
обычных команд на веску используйте `VESC_CAN_SendXxx`, не этот примитив.

### `HAL_StatusTypeDef VESC_CAN_SetTelemetryCallback(VESC_Handle_t *h, VESC_TelemetryCallback_t callback)`
Задаёт обработчик, вызываемый при получении **любого** распознанного статусного пакета (не только
кастомного датчика) — сигнатура `void(*)(VESC_Handle_t *h, VESC_CAN_PacketId_t status_id)`. `NULL`
отключает. **Вызывается из прерывания** — быстрый, неблокирующий код.

### `HAL_StatusTypeDef VESC_CAN_SetCustomSensorCallback(VESC_Handle_t *h, VESC_CustomSensorCallback_t callback)`
Задаёт обработчик, вызываемый при получении статуса кастомного датчика со значением
"pin-set"/"pin-reset" (не "none") — сигнатура `void(*)(VESC_Handle_t *h)`. `NULL` отключает.
**Вызывается из прерывания.**

---

## Имитация

Доступно всегда (если `VESC_CAN_SIM_ENABLE == 0` — заглушки, вызовы можно не убирать из кода).

### `void VESC_CAN_SetSimulated(VESC_Handle_t *h, uint8_t is_simulated)`
Помечает веску как имитируемую (1) или реальную (0, по умолчанию).

### `void VESC_CAN_SimulateTick(void)`
Один шаг имитации для всех помеченных весок сразу — модель "двигатель без нагрузки"
(разгон/торможение к последней команде скорости, малый ток, накопительные Ah/Wh, температура),
обновляет телеметрию так, как будто пришли все 7 статусов разом, и вызывает
`VESC_CAN_SetTelemetryCallback` 7 раз (по одному на статус) для консистентности с реальным приёмом.
Дельта времени считается автоматически — вызывайте периодически с любым периодом.

---

## Мост VESC Tool ↔ CAN (vesc_bridge.h)

Отдельный модуль (`vesc_bridge.h`/`vesc_bridge.c`) поверх `motor_vesc.h` — доступ к вескам из VESC
Tool на ПК так, будто в шину воткнут официальный **VESC Express**, транспорт-независимо
(Ethernet/UART/USB VCP). Полный протокол, честные ограничения и обоснование решений —
`BRIDGE_PROTOCOL.md`, примеры под каждый транспорт — `README.md`.

### `VESC_Bridge_t *VESC_Bridge_Init(const VESC_Bridge_Config_t *config)`
Создаёт мост (шина + свой CAN ID + колбэк отправки байт `tx_callback`). Шина должна быть уже
известна `motor_vesc.c` (хотя бы одна веска зарегистрирована через `VESC_CAN_Init()`). `NULL` при
ошибке.

### `void VESC_Bridge_FeedBytes(VESC_Bridge_t *br, const uint8_t *data, uint16_t len)`
Транспорт-независимая точка входа — скормить входящие байты откуда угодно (TCP-сокет, UART, USB
CDC). Можно по одному байту, можно кусками.

### `void VESC_Bridge_Tick(VESC_Bridge_t *br)`
Периодическое обслуживание таймаутов — вызывать регулярно (десятки мс).

### `void VESC_Bridge_OnCanFrame(VESC_Bridge_t *br, uint32_t ext_id, const uint8_t *data, uint8_t len)`
Приём ответных CAN-кадров от весок — вызывать из своей реализации `VESC_CAN_OnForeignFrame()` для
каждого "чужого" кадра.

### `uint8_t VESC_Bridge_IsTargetActive(VESC_Bridge_t *br, uint8_t vesc_id)`
1, если прямо сейчас идёт форвардинг именно этой веске (ждём ответа) — для временной приостановки
штатной отправки команд этой веске на время настройки через VESC Tool (мост это не делает сам,
решение — за вызывающим кодом).

### `void VESC_Bridge_OnLocalCommand(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)` — слабая функция
Вызывается для локальных команд VESC Tool, которые мост не умеет отвечать сам
(`COMM_FW_VERSION`/`COMM_PING_CAN` — умеет). Переопределите, чтобы добавить свои, отвечайте через
`VESC_Bridge_SendLocalReply()`.

### `void VESC_Bridge_SendLocalReply(VESC_Bridge_t *br, const uint8_t *payload, uint16_t len)`
Заворачивает payload во внешнее кадрирование и отправляет — для использования из
`VESC_Bridge_OnLocalCommand()`.

### `uint32_t VESC_Bridge_GetRxErrorCount(VESC_Bridge_t *br)` / `uint32_t VESC_Bridge_GetCanCrcErrorCount(VESC_Bridge_t *br)`
Диагностические счётчики — отвергнутые входящие пакеты (CRC/стоп-байт/ таймаут) и отвергнутые по CRC
ответы весок соответственно. В штатной работе должны оставаться на 0.
