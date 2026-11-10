# go1-sim: инструкции для Claude

Симулятор Go1 в Webots, ROS 2 Jazzy, работает в Docker. Подробности для пользователя в `README.md`.

## Безопасность

- Настоящего робота это репозиторий не трогает. Не добавлять сюда ничего, что по умолчанию шлёт команды на `ROS_DOMAIN_ID=0` или в `SSL_WIFI`.
- Программы для симулятора запускать через `./docker/run.sh`, не `ros2 run` напрямую на ноутбуке.
- Не коммитить и не публиковать `policies/` (извлечено из прошивки робота), материалы TrueTechArena (`worlds/level1.wbt`, `controllers/referee/`, `protos/Wall.proto`, `protos/Checkpoint.proto`, `protos/textures/`, `config/level1.json`).

## Правила изменения кода

- `controllers/go1_sim/sport.py` замещает `Legged_sport`. Новый режим добавлять только если он проверен на настоящем роботе, иначе отклонять с предупреждением (как сейчас).
- `controllers/go1_sim/locomotion.py` и политика ходьбы из Go2: менять осторожно. `SPEED_OFFSET` подбирался по замеру.
- `go1_control/` должен работать на роботе без изменений. Любую правку проверять и в симуляторе, и тестами (`./docker/run.sh exec bash -c 'cd /ws/go1_sim/go1_control && python3 -m pytest test'`).
- `Go1.proto` генерируется: не править руками, менять `tools/make_proto.py` и пересоздавать из URDF.
- Файлы `runtime.ini` и `result.json` создаются при запуске, в git не попадают.

## Docker

- Образ: `./docker/run.sh build`. Пересобирать после правок `Dockerfile` или `docker/entrypoint.sh`.
- Сборка `ros2_unitree_legged_msgs` и `go1_control` кэшируется в volume `go1-sim-build`. После правок сообщений или `go1_control` удалить: `docker volume rm go1-sim-build`.
- `unitree_ws` монтируется только для чтения.

## Git

- Это отдельный репозиторий внутри `ROS2026`. Коммиты только по прямой просьбе пользователя.
- Не коммитить в домашний репозиторий (`/home/fedoz`).
- `go1_control` здесь - копия пакета из `unitree_ws/src/go1_control`. Правки делаем здесь; копию в `unitree_ws` пока не трогаем.
