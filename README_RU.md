# gpib-core

[English](README.md) | [Русский](README_RU.md)

Переносимое ядро протокола IEEE 488.1/488.2 (GPIB) для микроконтроллеров.

**[Quickstart (EN)](QUICKSTART.md) | [Quickstart (RU)](QUICKSTART_RU.md)**

- **Язык:** C11, без кучи, один статический контекст (2408/1216 байт full/min)
- **Роли:** Контроллер и Прибор из одной кодовой базы (`GPIB_CFG_ROLE`)
- **Стандарты:** IEEE 488.1-1987 (все интерфейсные функции) + IEEE 488.2-1992 (протоколы контроллера, Module E)
- **Верификация:** 177 199 проверок, 0 отказов; независимый wire-оракул + chaos-стресс + мутационные регрессии

## Быстрый старт

```
powershell -ExecutionPolicy Bypass -File build.ps1
```

Семь бинарников, все должны выйти с кодом 0:

| Бинарник | Проверок | Назначение |
|---|---|---|
| `test_s1` | 177 199 | Роль контроллера: 25 сюитов (lifecycle, ledger, FSM, E2E, оракул, chaos-стресс) |
| `test_device` | 666 | Роль прибора: E2E против симулированного контроллера |
| `matrix_c/d_full/min` | 4 x 35 | CI-матрица роль x профиль с отчётом RAM |
| `test_bigcfg` | 14 | Граничная конфигурация compile-time |

Один сюит: `build\test_s1.exe --suite oracle` (или `stress`, `lineexec` и т.д.)

Linux/CMake: `cmake -S . -B build && cmake --build build && ctest`

## Архитектура

```
Приложение
    │
    ▼
┌─────────────────────────────────────────┐
│  gpib.h — публичный API (submit/process/│
│  ack/peek/cancel/lifecycle)             │
├─────────────────────────────────────────┤
│  submit.c    │  executor.c  │  ledger.c │
│  (валидация, │  (жизненный  │  (txn/block│
│   стейджинг) │   цикл)      │   записи)  │
├──────────────┴──────────────┴───────────┤
│  lineexec.c — линейный секвенсор        │
│  (фазы, композер drive, детач)          │
├─────────────────────────────────────────┤
│  proto.c — интерфейсные FSM IEEE 488.1  │
│  (SH AH T/TE L/LE SR RL DC DT C PP)     │
├─────────────────────────────────────────┤
│  ПОРТ (gpib_port.h) — ЕДИНСТВЕННАЯ      │
│  точка контакта с железом:              │
│  read_lines/drive/drive_data/           │
│  poll_event/now_us + гарантии G1-G5     │
└─────────────────────────────────────────┘
```

Ядро не касается GPIO напрямую. Всё аппаратно-зависимое идёт через
интерфейс колбэков порта. Реальный адаптер (STM32, ESP32, bit-bang)
реализует пять колбэков; ядро — платформонезависимый C11.

## Структура файлов

| Путь | Назначение |
|---|---|
| `include/gpib/gpib.h` | Публичный API (Doxygen) |
| `include/gpib/gpib_port.h` | Контракт порта + гарантии G1-G5 |
| `include/gpib/gpib_types.h` | Коды статусов, enum'ы, хэндлы, результаты |
| `include/gpib/gpib_cfg.h` | Compile-time ручки (роль, размеры, тайминги) |
| `include/gpib/gpib_proto.h` | Типы FSM + encode-хелперы |
| `include/gpib/gpib_storage.h` | Layout контекста + fingerprint |
| `include/gpib/gpib_4882.h` | Module E: API helper'а |
| `src/` | 11 файлов реализации (см. @file) |
| `tests/` | 25 сюитов, 45+ файлов |
| `docs/norms/` | Проверенные нормативные экстракты |
| `doc/html/` | Сгенерированная Doxygen-документация |

## Обзор API

```c
#include "gpib/gpib.h"

GPIB_CONTEXT_STORAGE_DEFINE(ctx);     /* статическое размещение */
gpib_init(&ctx, &(gpib_init_params_t){
    .system_controller = true,
    .own_address = { .primary = 5 },
});
gpib_start(&ctx);

/* Запись */
gpib_handle_t t, b;
gpib_request_t r = { .kind = GPIB_REQ_WRITE,
                     .address = { .primary = 6 },
                     .tx_data = "*IDN?\n", .tx_len = 6 };
gpib_submit(&ctx, &r, &t, &b);

/* Прокачка */
gpib_result_t res;
while (gpib_result_peek(&ctx, t, &res) != GPIB_OK)
    gpib_process(&ctx);

/* Результат */
if (res.reason == GPIB_TERM_COMPLETED)
    /* res.bytes_confirmed == 6 */;

/* ACK (владение возвращается) */
gpib_result_ack(&ctx, b);
gpib_result_ack(&ctx, t);

gpib_request_shutdown(&ctx);
gpib_process(&ctx);
gpib_deinit(&ctx);
```

## Слои верификации

1. **FSM-векторы** — направленные тесты из нормативных экстрактов (`docs/norms/`)
2. **S2-оракул** — исчерпывающая перекрёстная проверка таблиц переходов И выходов SH/AH
3. **E2E мок-приборы** — интеграционные тесты с FSM-shared партнёрами
4. **Core-to-core** — два реальных инстанса обмениваются TCT на одной шине
5. **Независимый wire-оракул** (`tests/oracle_peer.c`) — ноль общего кода протокола; нашёл 3 реальных бага, невидимых для FSM-shared тестов
6. **Chaos-стресс** — враждебная шина (случайные IFC/ATN-угон/SRQ) + сидированный workload + двусторонняя сверка байтов

## Конфигурация для слабых МК

```c
/* В gpib_cfg.h или через -D: */
#define GPIB_CFG_PROCESS_BUDGET 1     /* один обмен на process() */
#define GPIB_CFG_TRACE_EVENTS 0      /* трасса выключена: экономия кода и RAM */
#define GPIB_CFG_MAX_OPERATIONS 2    /* 2 слота транзакций для простого прибора */
#define GPIB_CFG_MAX_BLOCKS 2       /* 1 блок на операцию */
```

Даёт ~800 байт контекста, ~6-8 КБ flash (device-role min).

## Документация

```
tools\doxygen\doxygen.exe Doxyfile
```

Генерирует `doc/html/index.html` (173 страницы) по всем публичным заголовкам,
API-функциям и внутренней реализации.

## Статус

Host-уровень реализован полностью и закалён аудитами. Обе роли проходят
все тесты. Оставшаяся работа — аппаратная:

- Level-2 порт-адаптеры (STM32H743, ESP32, bit-bang)
- Валидация на стенде (реальные тайминги трансивера T1-T10)
- Независимое тестирование мультиконтроллерного takeover

## Нормативная база

`docs/norms/` содержит проверенные инженерные экстракты из IEEE 488.1-1987
и IEEE 488.2-1992, использованные как эталон для всех FSM.
См. таблицу в каталоге norms для деталей покрытия.

## Лицензия

TBD
