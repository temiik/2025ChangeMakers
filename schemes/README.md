# 📦 Hardware documentation and setup guide

The robot consists of multiple motors, sensors and two processing units — Raspberry Pi and LEGO EV3 Brick. The chassis is built on a modified LEGO EV3 Ackermann steering platform. All power comes from EV3 rechargeable battery and a dedicated power bank for Raspberry Pi. Image processing is handled by the Pi via a connected USB webcam.

---

## 📑 schemes/ — Схемы подключения

В этом разделе представлены схемы подключения компонентов автономной машинки на базе LEGO EV3 и Raspberry Pi.

---

## 📦 Список схем:

- `ev3_brick_wiring.png` — Подключение моторов и сенсоров к портам EV3 Brick.
- `ev3_digital_sensors.png` — Схема подключения Ultrasonic сенсоров, Touch Sensor и Color Sensor.
- `ev3_to_raspberry_connection.png` — Соединение Raspberry Pi и EV3 по USB.
- `photo_scheme.png` — Фото расположения камеры и сенсоров.

---

## 📡 Подключение оборудования:

| Device                        | Type             | Port / Connection     | Function                          |
|:-----------------------------|:----------------|:---------------------|:----------------------------------|
| **Medium Motor (drive)**      | Motor            | EV3 OUTPUT B          | Привод машинки                    |
| **Medium Motor (steering)**   | Motor            | EV3 OUTPUT C          | Поворот колёс                     |
| **Ultrasonic Sensor (left)**  | Digital Sensor   | EV3 INPUT 1           | Расстояние до препятствия слева  |
| **Ultrasonic Sensor (right)** | Digital Sensor   | EV3 INPUT 2           | Расстояние до препятствия справа |
| **Touch Sensor**              | Digital Sensor   | EV3 INPUT 3           | Старт и сброс алгоритма объезда   |
| **Color Sensor**              | Color Sensor     | EV3 INPUT 4           | Подсчёт кругов по меткам          |
| **Raspberry Pi 4/5**          | Single Board PC  | USB-A to Mini-USB EV3 | Камера и обработка изображения   |
| **Webcam**                    | USB Camera       | Raspberry Pi          | Видеопоток трассы                 |
| **PowerBank**                 | Battery          | Raspberry Pi 5V       | Питание Raspberry Pi              |
| **EV3 battery**               | Battery          | EV3 Brick             | Питание моторов и сенсоров       |

---

---

## 📄 Подключение по USB:

Raspberry Pi подключается к LEGO EV3 через USB кабель:
- **EV3**: Mini-USB порт (боковая панель)
- **Raspberry Pi**: USB-A порт

Протокол: TCP сокет, порт `12345`

---

## 📌 Правила подключения:

- Все моторы и сенсоры подключать только при выключенном EV3.
- Кабели не должны перекрывать поле зрения камеры.
- Проверить плотность всех коннекторов перед запуском.

---

## 📄 Описание работы:

- Raspberry Pi обрабатывает изображение.
- Определяет положение и цвет столбца.
- Отправляет EV3 команду через TCP.
- EV3 выполняет поворот или продолжает движение.
- Ультразвуковые сенсоры корректируют центрирование после объезда.
