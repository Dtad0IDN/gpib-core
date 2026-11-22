# Быстрый старт

[EN](QUICKSTART.md) | [RU](QUICKSTART_RU.md)

От нуля до обмена по GPIB за 15 минут.

## 1. Предварительные требования

- **Windows:** MinGW GCC (`gcc --version` работает)
- **Linux/macOS:** GCC или Clang, CMake 3.16+
- Без внешних зависимостей. Ничего устанавливать не нужно.

## 2. Сборка и проверка

```bash
git clone <repo> && cd gpib-core

# Windows
powershell -ExecutionPolicy Bypass -File build.ps1

# Linux
cmake -S . -B build && cmake --build build && cd build && ctest
```

Все тесты должны пройти (код выхода 0). Должно быть:
```
S1/S2: checks=177199 failures=0
DEVICE: checks=666 failures=0
```

## 3. Подключение к проекту

Скопируйте `include/gpib/` и `src/*.c` в вашу сборку. Всё —
не нужно собирать библиотеку, просто компилируйте исходники.

```cmake
# Пример CMakeLists.txt
add_library(gpib-core STATIC
    src/core.c src/submit.c src/results.c src/ledger.c
    src/executor.c src/trace.c src/proto.c src/lineexec.c
    src/mod4882.c src/epoch.c src/port_null.c
)
target_include_directories(gpib-core PUBLIC include)
```

## 4. Реализация порта (ЕДИНСТВЕННЫЙ аппаратный код)

Ядро общается с шиной через 5 колбэков. Минимальный
byte-pump порт (для тестирования; реальный порт использует `read_lines`/`drive`):

```c
#include "gpib/gpib_port.h"

static bool my_arm(void *user) { return true; }
static bool my_detached(void *user) { return true; }
static bool my_quiesced(void *user) { return true; }
static void my_inhibit(void *user) { /* отпустить все GPIO */ }
static uint64_t my_now_us(void *user) { return get_timer_us(); }
static size_t my_transfer(void *u, gpib_dir_t d, uint8_t *buf, size_t cap) {
    /* Ваш GPIO-хендшейк здесь. Вернуть число переданных байт. */
    return 0;
}
static bool my_poll_event(void *u, gpib_port_event_t *out) { return false; }

static const gpib_port_t my_port = {
    .name = "my-gpib",
    .arm = my_arm,
    .safe_detached = my_detached,
    .executor_quiesced = my_quiesced,
    .emergency_inhibit = my_inhibit,
    .now_us = my_now_us,
    .transfer = my_transfer,
    .poll_event = my_poll_event,
};
```

Для реальной шины также реализуйте линейные колбэки:

```c
static bool my_read_lines(void *u, gpib_line_snap_t *out) {
    /* Чтение всех 16 линий (8 DIO + 8 управления) ОДНИМ атомарным снапшотом. */
    out->dav  = !gpio_get(DAV_PIN);   /* активный низкий */
    out->nrfd = !gpio_get(NRFD_PIN);
    out->ndac = !gpio_get(NDAC_PIN);
    out->atn  = !gpio_get(ATN_PIN);
    out->eoi  = !gpio_get(EOI_PIN);
    out->srq  = !gpio_get(SRQ_PIN);
    out->ifc  = !gpio_get(IFC_PIN);
    out->ren  = !gpio_get(REN_PIN);
    out->dio  = gpio_read_bus(DIO_PORT);
    out->now_us = get_timer_us();
    return true;
}

static void my_drive(void *u, const gpib_line_drive_t *req) {
    gpio_set(DAV_PIN, !req->dav);    /* инверсия: asserted = LOW */
    gpio_set(NRFD_PIN, !req->nrfd);
    gpio_set(NDAC_PIN, !req->ndac);
    gpio_set(ATN_PIN, !req->atn);
    gpio_set(EOI_PIN, !req->eoi);
    gpio_set(REN_PIN, !req->ren);
    /* IFC/SRQ ядро обрабатывает само — не драйвить отсюда. */
}

static void my_drive_data(void *u, const uint8_t *dio) {
    if (dio) {
        gpio_bus_output(DIO_PORT);
        gpio_write_bus(DIO_PORT, *dio);
    } else {
        gpio_bus_input(DIO_PORT);    /* отпустить: open-collector */
    }
}
```

## 5. Контроллер: общение с прибором

```c
#include "gpib/gpib.h"

int main(void) {
    /* 1. Создание и запуск инстанса */
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init(&ctx, &(gpib_init_params_t){
        .system_controller = true,
        .own_address = { .primary = 0 },   /* наш GPIB-адрес */
        .port = &my_port,
    });
    gpib_start(&ctx);

    /* 2. Отправка *IDN? прибору по адресу 5 */
    gpib_handle_t t, b;
    gpib_request_t w = {
        .kind = GPIB_REQ_WRITE,
        .address = { .primary = 5 },
        .tx_data = "*IDN?\n",
        .tx_len = 6,
        .deadline_us = 0,   /* без таймаута */
    };
    gpib_submit(&ctx, &w, &t, &b);

    gpib_result_t res;
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);          /* из главного цикла или таймера */
    }
    /* res.reason == GPIB_TERM_COMPLETED, res.bytes_confirmed == 6 */
    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    /* 3. Чтение ответа */
    char buf[128] = {0};
    gpib_request_t r = {
        .kind = GPIB_REQ_READ,
        .address = { .primary = 5 },
        .rx_data = buf,
        .rx_cap = sizeof(buf) - 1,
    };
    gpib_submit(&ctx, &r, &t, &b);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);
    }

    if (res.reason == GPIB_TERM_COMPLETED) {
        /* buf содержит res.bytes_confirmed байт строки IDN */
        printf("IDN: %.*s\n", (int)res.bytes_confirmed, buf);
    }

    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    /* 4. Корректное выключение */
    gpib_request_shutdown(&ctx);
    gpib_process(&ctx);
    gpib_deinit(&ctx);
    return 0;
}
```

## 6. Прибор: ответ контроллеру

```c
int main(void) {
    GPIB_CONTEXT_STORAGE_DEFINE(ctx);
    gpib_init(&ctx, &(gpib_init_params_t){
        .system_controller = false,          /* мы прибор */
        .own_address = { .primary = 5 },
        .port = &my_port,
    });
    gpib_start(&ctx);

    /* Ждём, когда контроллер адресует нас и отправит данные */
    char rxbuf[256];
    gpib_handle_t t, b;
    gpib_device_sink(&ctx, rxbuf, sizeof(rxbuf), &t, &b);

    gpib_result_t res;
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);
        /* Проверка приборных команд (GET, DCL и т.д.) */
        uint8_t cmd;
        while (gpib_poll_device_event(&ctx, &cmd) == GPIB_OK) {
            /* Обработка trigger, clear, remote/local... */
        }
    }

    if (res.reason == GPIB_TERM_COMPLETED) {
        /* rxbuf[0..res.bytes_confirmed-1] — принятые данные */
    }
    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    /* Отправка ответа при адресации как talker */
    gpib_device_supply(&ctx, "HELLO\n", 6, &t, &b);
    while (gpib_result_peek(&ctx, t, &res) != GPIB_OK) {
        gpib_process(&ctx);
    }
    gpib_result_ack(&ctx, b);
    gpib_result_ack(&ctx, t);

    gpib_request_shutdown(&ctx);
    gpib_process(&ctx);
    gpib_deinit(&ctx);
    return 0;
}
```

## 7. Что дальше

- [Документация API](doc/html/index.html) (сгенерирована Doxygen)
- `tests/test_lineexec.c` — полные E2E-примеры
- `tests/oracle_peer.c` — независимая реализация порта
- `include/gpib/gpib_cfg.h` — ручки для размера/скорости
- [Контракт порта](include/gpib/gpib_port.h) — гарантии G1-G5
