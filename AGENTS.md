# AGENTS.md — VOSTOK Configuration

Конфигурация Klipper для 3D-принтера VOSTOK (IDEX, две печатающие головы).
Репозиторий содержит базовую конфигурацию, набор модулей электроники и
слайсер-постпроцессоры для быстрой смены инструмента.

## Стек и зависимости

- **Klipper**: обязательно ветка [`generic-cartesian`](https://github.com/dmbutyugin/klipper/tree/generic-cartesian) от dmbutyugin. Mainline не подходит — нужен `generic_cartesian` kinematics и `dual_carriage` с `SET_DUAL_CARRIAGE MODE=DIRECT` (см. `printer_base.cfg:14`, `printer_base.cfg:47`).
- **Python 3**: для `postprocessing/*.py`. Только stdlib (argparse, re, math, heapq, datetime, optparse, sys, pathlib). Никаких pip-зависимостей.
- **Слайсер**: post-processing scripts цепляются в PrusaSlicer / SuperSlicer / OrcaSlicer.

## Структура репозитория

```
printer.cfg                  # Пользовательский файл. Редактируется под каждый принтер.
printer_base.cfg             # !!! НЕ РЕДАКТИРОВАТЬ. База: carriages, steppers, extruders, homing, IDEX-макросы.
electronics_*.cfg            # Модули распиновки под конкретную плату + коммутационные платы.
postprocessing/
  fast_tool_swaps.py         # Слайсер-поцессор; вставляет wipe-движения при T0/T1.
  fix_toolchange.py          # Чистит G-code: убирает G1 E* при T-переключениях, склеивает дубли, добавляет deretract.
pics/                        # Картинки для README.
README.md                    # Пользовательская документация (на русском).
README.en.md                 # её английский перевод.
user_configs/                # Пользовательские конфигурации электроники.
  README.md                  # Описание папки (на русском) + README.en.md.
```

## Архитектура конфигурации

Конфиг разделён на три слоя по принципу "приоритет последнего include" (Klipper применяет последнее значение одноимённого параметра):

1. **`printer_base.cfg`** — параметры и макросы, общие для всех принтеров. Содержит:
   - секции кареток `[carriage xc]`, `[dual_carriage wc]`, `[carriage ylc]`, `[extra_carriage yrc]`, `[carriage zc]`;
   - stepper-секции `[stepper motor_x]`, `[stepper motor_w]`, `[stepper motor_yl]`, `[stepper motor_yr]`, `[stepper motor_z]`;
   - `[extruder]` и `[extruder1]`;
   - `[homing_override]` с G28-логикой, использующей `_IDEX_SETTINGS` и `_HOME_Z` (`printer_base.cfg:162`);
   - весь блок IDEX-макросов: `IDEX_RESET`, `IDEX_MODE_FULL_CONTROL`, `IDEX_MODE_COPY`, `IDEX_MODE_MIRROR`, `T0`, `T1`, `IDEX_SHAPER_CALIBRATE`, `SET_PRESS_ADVANCE_EQUAL`, `START_PRINT_IDEX_MODE_COPY`, `START_PRINT_IDEX_MODE_MIRROR`.
2. **`electronics_*.cfg`** — только распиновка (`step_pin`, `dir_pin`, `enable_pin`, `endstop_pin`, `heater_pin`, `sensor_pin`, `fan_pin`) и настройки драйверов (`run_current`, `tmcXXXX`).
3. **`printer.cfg`** — калибруемые пользователем параметры: `position_min/max/endstop`, `max_velocity`, `rotation_distance`, `pid_*`, `z_offset`, `bed_mesh`, `screws_tilt_adjust`, `_IDEX_SETTINGS` (расстояние между головами, оффсеты, safety_offset, параметры toolchange), `[include ...]` нужного `electronics_*.cfg`.

**`printer.cfg` подключается строго в порядке** (см. `printer.cfg:55-64`):

```
[include printer_base.cfg]
[include electronics_*.cfg]    # ← единственный include, который выбирает пользователь
... остальное — параметры и переопределения
```

Не добавлять `[include]` между `printer_base.cfg` и `electronics_*.cfg` — `printer_base.cfg` должен идти первым, иначе IDEX-макросы перебьют параметры из последующих include.

## Правила именования и каретки

- Оси: `X` управляется кареткой `xc`, `W` (правая IDEX-каретка) — `wc`, `Y` — двумя каретками `ylc` (ведущая) + `yrc` (ведомая через `[extra_carriage]`), `Z` — `zc`.
- Extruders: `extruder` (левый, на T0CB) и `extruder1` (правый, на T1CB).
- Моторы: `motor_x`, `motor_w`, `motor_yl`, `motor_yr`, `motor_z` — эти имена должны сохраняться везде, на них ссылаются секции `[carriage ...]` и `homing_override`.
- Шаблоны имён MCU: `T0CB` (левая коммутационная плата) и `T1CB` (правая). CAN-uuid задаётся в `printer.cfg` (`[mcu T0CB]`, `[mcu T1CB]`); без этих секций пины вида `T0CB:PB9` не разрешатся.

## Именование `electronics_*.cfg`

Шаблон: `electronics_<материнская>_<ревизия>_<MCU>_<двигатели-материнской>x<модель-драйвера>_<коммутационные>x<модель>.cfg`.

Пример: `electronics_octopus_pro_v1.1_h723_5x2240_2xH36v1.3.cfg` = BTT Octopus Pro v1.1 (STM32H723) + 5 драйверов TMC2240 + 2 коммутатора Fysetc H36 v1.3.

В шапке такого файла обязательно:
- перечень плат и драйверов;
- таблица соответствия "оборудование → разъём" (см. `electronics_octopus_pro_v1.1_h723_5x2240_2xH36v1.3.cfg:17-42`);
- блоки `[stepper ...]`, `[extruder]`, `[extruder1]`, `[heater_bed]`, `[fan_*]`, `[probe]`, `[endstop *_pin]` с пинами.

## Команды печати (IDEX)

Макросы для пользователя (см. `printer_base.cfg:301-820`):

| Макрос | Назначение |
|---|---|
| `IDEX_RESET` | Сброс в классический режим, активная голова = T0. |
| `IDEX_MODE_FULL_CONTROL` | Полное раздельное управление головами. |
| `IDEX_MODE_COPY [PRINTHEAD_DISTANCE=...]` | Дублирующий режим. |
| `IDEX_MODE_MIRROR` | Зеркальный режим. |
| `T0`, `T1` | Смена инструмента с парковкой и retract. |
| `IDEX_SHAPER_CALIBRATE` | Калибровка input shaper для обеих голов. |
| `SET_PRESS_ADVANCE_EQUAL [ADVANCE=...] [SMOOTH_TIME=...]` | Выровнять PA между головами. |
| `START_PRINT_IDEX_MODE_COPY` / `START_PRINT_IDEX_MODE_MIRROR` | Стартовые макросы для слайсера. |

Ключевые переменные в `_IDEX_SETTINGS` (`printer.cfg:73-82`):
- `default_printhead_distance` — расстояние между головами в режиме копирования;
- `right_printhead_x/y_offset` — оффсеты правой головы (только классический режим);
- `safety_offset` — отступ от крайних позиций при парковках;
- `toolchange_retract_length/speed/travel_speed` — параметры T0/T1.

## Post-processing scripts

`postprocessing/` запускаются из слайсера. В обоих файлах:

- `fast_tool_swaps.py` — основной. Парсит G-code, моделирует состояние принтера (`GCodeState`, `printer_base.cfg:21-50`), вставляет `SYNC_EXTRUDER_MOTION` и wipe-движения перед T-переключениями. Помечает выход `; Processed by fast tool swaps script`. Запускается в OrcaSlicer / PrusaSlicer как post-processing script.
- `fix_toolchange.py` — вспомогательный. Удаляет `G1 E*` в строках вида `T... X... Y...`, удаляет подряд идущие дубли, добавляет deretract-команду (`--long_deretract_length`).

Замечание: эти два скрипта **никак не связаны с конфигом Klipper** и не импортируются им. Они обрабатывают G-code до печати.

## Стиль и конвенции

- **Кодировка и язык**: комментарии — на русском, UTF-8.
- **Лицензия**: CC-BY 4.0, шапка указана в каждом cfg.
- **Macro-именование**: пользовательские макросы — `UPPER_SNAKE_CASE` (`IDEX_RESET`, `T0`, `START_PRINT_IDEX_MODE_COPY`). Служебные (с подчёркиванием) — `_IDEX_SETTINGS`, `_IDEX_VARIABLES`, `_HOME_Z`.
- **Параметры Klipper**: `snake_case`.
- **Порядок секций в `printer_base.cfg`**: printer → carriages → steppers → extruders → bed/probe → homing → fans → macros. При добавлении секции держитесь этой структуры.
- **Header в `printer_base.cfg`** начинается с `# !!! НЕ ВНОСИТЕ ИЗМЕНЕНИЯ В ЭТОТ ФАЙЛ !!!`. Любые пользовательские правки делаются в `printer.cfg` путём копирования секции и переопределения значения.

## Типичные задачи

### Добавить конфиг для новой платы
1. Создать `electronics_<плата>.cfg` в корне.
2. В шапке — таблицу соответствия разъёмов.
3. Скопировать структуру существующего `electronics_*.cfg`.
4. Создать ветку `add-<платы>`, оформить PR (см. README.md:19-41).

### Изменить параметр из базы
Скопировать нужную секцию/параметр в `printer.cfg` в раздел "Пользовательские настройки" (`printer.cfg:192-197`) и поменять значение. Не редактировать `printer_base.cfg`.

### Калибровка
- `rotation_distance` экструдера — `printer.cfg:133,143`.
- `position_min/endstop/max` кареток — `printer.cfg:113-126`.
- `safe_distance` второй каретки — `printer.cfg:121` (занизить на ~0.1мм для обхода бага расчёта safe-distance).
- PID — закомментированы в `printer.cfg:136-149,154-157`; раскомментировать и вписать после `PID_CALIBRATE`.

### Изменение параметров T0/T1
Через `_IDEX_SETTINGS` в `printer.cfg:73-82`. Не трогать сами макросы `T0`/`T1` в `printer_base.cfg`.

## Язык ответов

Пользовательская документация в репозитории — на русском. Комментарии в `*.cfg`, обсуждения PR и issues — также на русском. Отвечайте на русском, если не указано иное.
