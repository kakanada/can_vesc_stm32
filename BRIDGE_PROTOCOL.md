# Протокол моста VESC Tool ↔ CAN (vesc_bridge)

Техническая изнанка `vesc_bridge.h`/`vesc_bridge.c`: точные байтовые форматы протокола, чтобы не
пришлось второй раз лазить в исходники прошивки VESC при отладке или доработке.

Все форматы ниже сверены с исходным кодом прошивки VESC (репозиторий
[vedderb/bldc](https://github.com/vedderb/bldc)): `comm/comm_can.c`, `comm/commands.c`,
`comm/packet.c`, `datatypes.h`, а также `crc.c` (через
[vedderb/nrf51_vesc](https://github.com/vedderb/nrf51_vesc), тот же алгоритм) — см. §8.

## 1. Общая идея моста

VESC Tool умеет говорить с "хабом" (обычно это сама VESC, к которой ПК подключен напрямую по
USB/UART, либо VESC Express по WiFi/TCP) и через него адресовать команды **другим** VESC на той же
CAN-шине, которых ПК физически не видит. Хаб для этого:
1. Разбирает внешнее кадрирование протокола VESC Tool (пришедшее по USB/UART/TCP/вообще любому
   байтовому каналу).
2. Если payload начинается с `COMM_FORWARD_CAN` — достаёт целевой CAN ID и вложенную команду,
   форвардит её конкретной веске по CAN через штатный механизм многокадровой сборки
   (`CAN_PACKET_FILL_RX_BUFFER[_LONG]` + `CAN_PACKET_PROCESS_[SHORT_]BUFFER`).
3. Ответ, который веска-цель пришлёт по той же CAN-механике, пересобирает и отправляет обратно ПК
   тем же внешним кадрированием, **без** повторной обёртки в `COMM_FORWARD_CAN`.
4. Небольшой набор команд хаб обрабатывает **локально**, не форвардя на CAN (см. §5) — иначе VESC
   Tool не сможет даже "поздороваться" с хабом.

`vesc_bridge` реализует шаги 1-4 независимо от того, какой транспорт принёс байты (TCP-сокет, UART,
USB CDC) — не трогает сам транспорт, только скармливание/выдача сырых байт через два универсальных
колбэка (`VESC_Bridge_FeedBytes` / `tx_callback`). Форвардинг по CAN использует низкоуровневый
примитив `motor_vesc.h` (`VESC_CAN_SendRawFrame`) и точку расширения `VESC_CAN_OnForeignFrame` для
приёма ответных кадров.

## 2. Внешнее кадрирование (packet.c) — байт-поток ↔ пакет

```
короткий (payload ≤ 255 байт):
  [0x02][len:1 байт][payload: len байт][crc_hi][crc_lo][0x03]

средний (payload 256..65535 байт):
  [0x03][len_hi][len_lo][payload: len байт][crc_hi][crc_lo][0x03]
```

- CRC считается ТОЛЬКО по `payload` (не по старт-байту/длине/стоп-байту).
- Длинный вариант (3-байтная длина, старт-байт 0x04, до 16 МБ payload — под обновление прошивки) не
  реализован — см. §7.
- `vesc_bridge` умеет и принимать, и отправлять оба реализованных формата — короткий выбирается
  автоматически, если `len ≤ 255`, иначе средний.

### CRC16 (XMODEM)

```c
unsigned short crc16(unsigned char *buf, unsigned int len) {
    unsigned int i;
    unsigned short cksum = 0;                 /* init = 0x0000 */
    for (i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}
```

Стандартный **CRC-16/XMODEM** (poly `0x1021`, init `0x0000`, без рефлексии, без финального XOR). В
`vesc_bridge.c` реализован побитово (без 256-элементной таблицы) — математически идентичный
результат:

```c
crc ^= (byte << 8);
for (8 раз) { crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1); }
```

Проверено на общепринятом контрольном векторе: CRC-16/XMODEM("123456789") = `0x31C3`.

## 3. Многокадровая пересылка по CAN (comm_can.c)

Все CAN ID здесь — Extended, формат `(cmd_id << 8) | целевой_vesc_id`, как и у всего остального в
этой библиотеке.

### Коды команд (из `datatypes.h`, `CAN_PACKET_ID`)

| Имя | Код |
|---|---|
| `CAN_PACKET_FILL_RX_BUFFER` | 5 |
| `CAN_PACKET_FILL_RX_BUFFER_LONG` | 6 |
| `CAN_PACKET_PROCESS_RX_BUFFER` | 7 |
| `CAN_PACKET_PROCESS_SHORT_BUFFER` | 8 |

(Уже присутствуют в `motor_vesc.h` как часть `VESC_CAN_PacketId_t`.)

### Короткий путь — payload вложенной команды ≤ 6 байт

Один кадр `CAN_PACKET_PROCESS_SHORT_BUFFER`, payload (8 байт):
```
[0]  reply_to_id   - CAN ID отправителя (для нас - own_can_id моста)
[1]  send_flag     - см. §4
[2:] вложенная команда, как есть, до 6 байт
```

### Длинный путь — payload вложенной команды > 6 байт

1. `CAN_PACKET_FILL_RX_BUFFER` — по 7 байт данных за кадр, offset 1 байтом (0..255):
   ```
   [0] offset (0..255)
   [1:] до 7 байт данных
   ```
2. Для данных дальше 255-го байта — `CAN_PACKET_FILL_RX_BUFFER_LONG`, по 6 байт данных за кадр,
   offset 2 байтами (big-endian, 0..65535):
   ```
   [0] offset_hi
   [1] offset_lo
   [2:] до 6 байт данных
   ```
3. Завершающий кадр `CAN_PACKET_PROCESS_RX_BUFFER` (payload ровно 6 байт):
   ```
   [0] reply_to_id  - CAN ID отправителя (own_can_id моста)
   [1] send_flag    - см. §4
   [2] len_hi        - длина ВСЕГО собранного payload, big-endian
   [3] len_lo
   [4] crc_hi        - CRC16(payload) всего собранного буфера, big-endian
   [5] crc_lo
   ```
Получатель проверяет CRC над накопленным буфером на офсетах `0..len` и только при совпадении
исполняет команду.

Та же механика используется и в обратную сторону — ответ вески-цели пересобирается точно так же
(`VESC_Bridge_OnCanFrame()` в vesc_bridge.c): `FILL_RX_BUFFER[_LONG]` копируют кусок в буфер
пересборки по офсету, `PROCESS_RX_BUFFER`/`PROCESS_SHORT_BUFFER` завершают приём (с проверкой CRC
для длинного варианта) и передают готовый ответ клиенту.

### Собственный CAN ID моста (`own_can_id`)

Чтобы веска-цель знала, куда слать ответ (`reply_to_id` в кадрах выше), мост обязан иметь свой CAN
ID на шине. Это поле конфигурации (`VESC_Bridge_Config_t.own_can_id`) — назначается вручную и должно
быть уникальным на шине: не совпадать ни с одной реальной веской и ни с одним другим
мостом/устройством (библиотека это не проверяет, аналогично `position_memory_backup_index` в
`motor_vesc.h`).

## 4. Поле `send`/`commands_send` при форвардинге

При форвардинге `vesc_bridge` всегда передаёт `send_flag = 0` (константа `VESC_BRIDGE_SEND_FLAG` в
`vesc_bridge.c`) — это то же значение, которое использует официальный обработчик `COMM_FORWARD_CAN`
в прошивке VESC (`comm_can_send_buffer(data[0], data + 1, len - 1, 0)`), и единственное значение,
для которого подтверждено, что получатель корректно запоминает адрес возврата (`rx_buffer_last_id`)
для последующего ответа.

Если после тестирования какие-то конкретные команды не получают ответа через мост — это первое место
для проверки.

## 5. Локальные команды (обрабатываются мостом сам, не форвардятся)

Если `payload[0]` (COMM_PACKET_ID) не равен `COMM_FORWARD_CAN` — команда адресована самому мосту, а
не веске за ним.

### `COMM_FW_VERSION` (код 0)

VESC Tool шлёт это при подключении к любому устройству для хэндшейка/определения совместимости.
Реализован минимально достаточный ответ: major/minor версия и имя "железа" из `VESC_Bridge_Config_t`
(не зашиты намертво — можно подстроить без правки библиотеки), 12 нулевых байт вместо UUID,
остальные поля (флаги пары/QMLUI/NRF/кастомного конфига, CRC прошивки) — 0. Это не гарантированно
то, что ждёт конкретная версия VESC Tool — см. §7.

### `COMM_PING_CAN` (код 62)

Отвечает списком CAN ID весок, уже зарегистрированных в `motor_vesc.c` через `VESC_CAN_Init()` на
этой шине и считающихся "живыми" по `VESC_CAN_IsAlive()` — без живого пинга всей шины
(`CAN_PACKET_PING`/`PONG`). Форвардинг (`COMM_FORWARD_CAN`) при этом поддерживает адресацию на любой
CAN ID вне зависимости от регистрации — просто такая веска не попадёт в список автообнаружения VESC
Tool, придётся ввести её ID вручную.

### Остальные локальные команды

Не реализованы. Точка расширения `VESC_Bridge_OnLocalCommand()` (по аналогии с
`VESC_CAN_OnForeignFrame` в motor_vesc.c) вызывается для любого нераспознанного локального payload —
переопределяемая слабая функция, по умолчанию ничего не делает.

## 6. Сигнатура `VESC_CAN_OnForeignFrame()` (motor_vesc.h/.c, версия 1.3+)

`VESC_CAN_OnForeignFrame(VESC_CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t
len)`

Более раннее API принимало вторым параметром backend-специфичный заголовок
(`FDCAN_RxHeaderTypeDef*`/`CAN_RxHeaderTypeDef*`), из которого длину кадра приходилось бы доставать
вручную и по-разному на каждом бэкенде. Начиная с версии 1.3 — простые `ext_id`+`len`, посчитанные
самим `motor_vesc.c`. Если у вас есть код, переопределявший эту функцию под более раннюю версию
библиотеки — сигнатуру нужно обновить.

## 7. Ограничения

- **Обновление прошивки весок через мост не реализовано.** Команды
  `COMM_WRITE_NEW_APP_DATA`/`COMM_ERASE_NEW_APP` форвардятся так же, как любая другая (мост не
  различает их особо), но буфер сборки (`VESC_BRIDGE_MAX_PAYLOAD`, по умолчанию 1024 байт) рассчитан
  на конфигурационные пакеты (GET/SET MCCONF/APPCONF, GET_VALUES, terminal и т.п.), не на
  мегабайтные бинарные передачи — длинный (3-байтный) вариант внешнего кадрирования (>65535 байт)
  тоже не реализован.
- **Один активный форвардинг единовременно на мост.** Буфер пересборки общий (не пуловый по ID) —
  рассчитано на синхронную модель VESC Tool (следующий запрос отправляется после ответа/таймаута
  предыдущего, как и в самой прошивке VESC).
- **`COMM_FW_VERSION` и `COMM_PING_CAN` — минимальная реализация, не проверялись с реальным VESC
  Tool** (см. §5) — вероятное первое место для правок после тестирования.
- **Значение `send_flag = 0`** — см. §4, тоже кандидат на пересмотр, если форвардинг конкретных
  команд не получает ответа.
- **Конкурентный доступ к состоянию форвардинга.** `forward_src`/
  `forward_send_offset`/`forward_phase` пишутся и из контекста, откуда вызывается
  `VESC_Bridge_Tick()`/`VESC_Bridge_FeedBytes()` (может быть прерывание), и из
  `VESC_Bridge_OnCanFrame()` (CAN-прерывание) — защищено критическими секциями
  (`__disable_irq`/`__enable_irq`) в `vesc_bridge_forward_step()`/`vesc_bridge_start_forward()` и в
  таймауте `VESC_Bridge_Tick()`. Единственный незащищённый симметрично случай: сброс `forward_phase`
  в `IDLE` при получении ответа в `VESC_Bridge_OnCanFrame()` (одна инструкция, не рвётся) — при
  очень маловероятном совпадении по времени с началом нового форвардинга
  `VESC_Bridge_IsTargetActive()` может на мгновение вернуть неверный результат; сам обмен данными от
  этого не портится. Протокол VESC не даёт способа надёжно связать ответ с конкретным запросом (в
  ответных кадрах нет ID отправителя-вески, только адресата- моста), поэтому это ограничение
  принципиальное, не только этой реализации.
- **Таймауты** `VESC_BRIDGE_FORWARD_TIMEOUT_MS` (по умолчанию 500 мс) и `VESC_BRIDGE_RX_TIMEOUT_MS`
  (по умолчанию 300 мс) — оценочные значения, не измерялись на реальной задержке VESC Tool ↔ мост ↔
  CAN ↔ веска.

## 8. Примеры подключения по транспорту

Модуль сам не знает про TCP/UART/USB — только два колбэка: `VESC_Bridge_FeedBytes()` (скормить
входящие байты) и `tx_callback` (сюда мост сам отдаёт исходящие байты). Общая часть кода одинакова
для всех сценариев:

```c
#include "vesc_bridge.h"

static VESC_Bridge_t *s_bridge;

void BridgeInit(VESC_Bridge_TxCallback_t tx_callback)
{
    VESC_Bridge_Config_t cfg = {
        .hcan             = &hfdcan1, /* шина, где уже зарегистрированы вески через VESC_CAN_Init() */
        .own_can_id       = 250,      /* НЕ должен совпадать ни с одной веской/мостом на этой шине! */
        .tx_callback      = tx_callback,
        .fw_version_major = 6,
        .fw_version_minor = 5,
        .hw_name          = "STM32-BRIDGE",
    };
    s_bridge = VESC_Bridge_Init(&cfg);
}

void BridgeTick(void) { VESC_Bridge_Tick(s_bridge); } /* вызывать периодически (таймауты) */

void VESC_CAN_OnForeignFrame(VESC_CAN_HandleTypeDef *hcan, uint32_t ext_id,
                              const uint8_t *data, uint8_t len)
{
    (void)hcan; /* один мост - одна шина; при нескольких мостах сверяйте с VESC_Bridge_Config_t.hcan */
    VESC_Bridge_OnCanFrame(s_bridge, ext_id, data, len);
}
```

### ПК (VESC Tool) → Ethernet → STM32 → CAN → veska

STM32 сам поднимает TCP-сервер (lwIP) на отдельном порту (у настоящего VESC Express это 65102):

```c
#include "lwip/sockets.h"

static int s_client_fd = -1;

static void bridge_tx_to_socket(VESC_Bridge_t *br, const uint8_t *data, uint16_t len)
{
    (void)br;
    if (s_client_fd >= 0) { send(s_client_fd, data, len, 0); }
}

/* Отдельная задача lwIP (например поверх FreeRTOS) - принимает ОДНО клиентское
   подключение и гоняет байты в обе стороны */
void BridgeNetworkTask(void *arg)
{
    (void)arg;
    BridgeInit(bridge_tx_to_socket);

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    /* ... bind()/listen() на нужном порту - как обычно в вашем стеке lwIP ... */

    for (;;)
    {
        s_client_fd = accept(listen_fd, NULL, NULL);
        uint8_t buf[512];
        int n;
        while ((n = recv(s_client_fd, buf, sizeof(buf), 0)) > 0)
        {
            VESC_Bridge_FeedBytes(s_bridge, buf, (uint16_t)n);
        }
        close(s_client_fd);
        s_client_fd = -1;
    }
}
```

`BridgeTick()` вызывайте из отдельного периодического таймера/задачи (независимо от
`BridgeNetworkTask`, которая блокируется на `recv()`).

### ПК → Ethernet → одноплатный компьютер → UART → STM32 → CAN → veska

Одноплатник (Raspberry Pi и т.п.) прозрачно перекладывает байты из своего TCP-сокета в UART и
обратно (например через `socat` или пару строк на Python) — STM32 вообще не знает, что "за UART" в
итоге сеть, для него это просто UART:

```c
static uint8_t s_uart_rx_byte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)   /* UART, смотрящий на одноплатный компьютер */
    {
        VESC_Bridge_FeedBytes(s_bridge, &s_uart_rx_byte, 1U);
        HAL_UART_Receive_IT(huart, &s_uart_rx_byte, 1U); /* сразу готовим приём следующего байта */
    }
}

static void bridge_tx_to_uart(VESC_Bridge_t *br, const uint8_t *data, uint16_t len)
{
    (void)br;
    HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 100);
}

void MainInit(void)
{
    BridgeInit(bridge_tx_to_uart);
    HAL_UART_Receive_IT(&huart3, &s_uart_rx_byte, 1U);
}
```

Приём по одному байту с `HAL_UART_Receive_IT` — самый простой вариант; для высокой нагрузки лучше
DMA в кольцевой буфер с вычиткой в `BridgeTick()`, `VESC_Bridge_FeedBytes()` одинаково хорошо
принимает и по байту, и большими кусками.

### ПК → Ethernet → одноплатный компьютер → USB Virtual COM Port → STM32 → CAN → veska

Со стороны STM32 это ровно тот же код, что и сценарий с USB VCP напрямую (ниже) — одноплатник для
STM32 неотличим от ПК, подключенного по USB напрямую. Разница целиком на стороне одноплатника (он же
и открывает `/dev/ttyACM0`, и слушает сеть).

### ПК (VESC Tool) → USB Virtual COM Port → STM32 → CAN → veska

STM32 поднимает USB CDC (Virtual COM Port) через штатный USB Device Middleware из CubeMX:

```c
/* CDC_Receive_FS - колбэк, сгенерированный CubeMX в usbd_cdc_if.c */
int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    VESC_Bridge_FeedBytes(s_bridge, Buf, (uint16_t)*Len);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS); /* сразу готовим приём следующего пакета */
    return USBD_OK;
}

static void bridge_tx_to_usb(VESC_Bridge_t *br, const uint8_t *data, uint16_t len)
{
    (void)br;
    CDC_Transmit_FS((uint8_t *)data, len);
}

void MainInit(void)
{
    BridgeInit(bridge_tx_to_usb);
}
```

Дальше в VESC Tool выбираете COM-порт (сценарии по USB) либо TCP-адрес STM32 (сценарий по Ethernet)
и работаете с вескими за мостом как с обычным CAN-устройством VESC Tool.

## 9. Источники

- `comm/comm_can.c`, `comm/commands.c`, `comm/packet.c`, `datatypes.h` —
  <https://github.com/vedderb/bldc>
- `crc.c` — <https://github.com/vedderb/nrf51_vesc> (идентичный алгоритм, используется тем же
  `packet.c` в основной прошивке)
