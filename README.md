# 🧩 CRTeam Sistema de Telemetría

## Cableado

### 🔹 Raspberry Pi + MCP2515 + Nextion LCD

#### ✅ Summary
Este circuito conecta una **Raspberry Pi 3B+** con un controlador **CAN MCP2515** y una pantalla táctil **Nextion LCD 2.4"**. La Raspberry Pi gestiona la comunicación CAN y la interfaz de usuario a través de la pantalla.

#### 🧰 Componentes
- Raspberry Pi 3B+
- MCP2515 (Controlador CAN)
- Pantalla táctil Nextion 2.4"

#### 🔌 Conexiones

![Diagrama Raspberry + MCP2515 + Nextion](Conexiones-Raspberry.png)

| Raspberry Pi GPIO        | Conectado a                   | Componente             |
|--------------------------|-------------------------------|------------------------|
| 3.3V                     | VCC                           | MCP2515                |
| 5V                       | VCC                           | Nextion LCD            |
| GND                      | GND                           | MCP2515 & Nextion LCD  |
| GPIO 11 (SCLK - SPI0)    | SCK                           | MCP2515                |
| GPIO 10 (MOSI - SPI0)    | SI                            | MCP2515                |
| GPIO 09 (MISO - SPI0)    | SO                            | MCP2515                |
| GPIO 8  (CE0 - SPI0 CS)  | CS                            | MCP2515                |
| GPIO 25                  | INT                           | MCP2515                |
| GPIO 17 (CE1 - SPI1)     | TX                            | Nextion LCD            |
| GPIO 27                  | RX                            | Nextion LCD            |

---

### 🔹 Arduino Nano + Teensy 4.1 + MPU6050 Sensors

#### ✅ Summary
Este circuito usa un **Arduino Nano** y un **Teensy 4.1+** para leer datos de tres sensores **MPU6050** (acelerómetro + giroscopio). El Nano gestiona un sensor y el Teensy los otros dos, compartiendo líneas I2C para cada par.

#### 🧰 Componentes

- Arduino Nano
- MPU6050 (acelerómetro + giroscopio)
- Teensy 3.5

### 🔌 Conexiones

![Diagrama Teensy + Nano + MPU6050](Conexiones-Acelerometros.png)

#### Arduino Nano → MPU6050 (Sensor 1)
| Arduino Nano | MPU6050 |
|--------------|---------|
| 5V           | VCC     |
| GND          | GND     |
| A5           | SCL     |
| A4           | SDA     |

#### Teensy 4.1+ → MPU6050 (Sensor 2 & 3)
| Teensy Pin   | MPU6050 (1) | MMPU6050 (2) |
|--------------|-------------|--------------|
| Pin 64 (5V)  | VCC & INT   | VCC          |
| Pin 12 (GND) | GND         | GND & INT    |
| Pin 57       | SCL         | SCL          |
| Pin 56       | SDA         | SDA          |

---

## ⚙️ Códigos del los componentes

### 📦 Librerías Arduino (Sender & Receiver)
- `ArduinoJson`
- `mrf24_lib`

### 📦 Librerías Teensy (Acelerómetro)
- `ACAN`
- `Madgwick`
- `MPU6050`
