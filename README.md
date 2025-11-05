# ESP32 – Water Quality Monitoring System (TDS + Temperature) with LoRa Communication

This repository contains an embedded system developed using **ESP-IDF** for the ESP32 platform.  
The system measures:

- Water temperature using a **DS18B20 digital sensor**
- Total Dissolved Solids (TDS) using an **analog TDS probe** (ADC input)

The data is transmitted using a **LoRa P2P (peer-to-peer)** link, with a custom reliability mechanism that includes:

- Acknowledgment (ACK) from receiver
- Automatic retransmission
- Local buffering of unsent packets using SPIFFS (persistent flash storage)

The ESP32 remains mostly in **deep sleep mode**, waking only periodically to capture data and transmit, optimizing energy usage for battery-powered deployments.

---

## System Overview

The program follows this execution sequence:

1. Wake from deep sleep
2. Power and read the DS18B20 temperature sensor
3. Power the TDS probe, perform multiple ADC readings, compute the average and convert to ppm, compensating according to temperature
4. Initialize LoRa radio
5. Attempt to re-transmit previously failed transmissions (stored in SPIFFS)
6. Generate a packet containing:
   - Packet ID (persistent across reboots using NVS)
   - Temperature (°C)
   - TDS (ppm)
7. Transmit packet and wait for ACK from receiver
8. If transmission fails after multiple attempts, store the packet in SPIFFS
9. Enter deep sleep until next cycle

---

## Hardware Connections

| Component                | ESP32 GPIO | Description |
|--------------------------|------------|-------------|
| TDS probe – Power enable | GPIO 32    | Controls sensor power (reduces consumption) |
| TDS probe – Analog output | GPIO 36 (ADC1_CHANNEL_0) | ADC input |
| DS18B20 – Power enable   | GPIO 25    | Controls sensor power (prevents parasite consumption during sleep) |
| DS18B20 – Data (DQ)      | GPIO 14    | Requires a 4.7 kΩ pull-up resistor to 3.3 V |
| LoRa (SX1276 / SX1278)   | Standard SPI pins | SPI configuration via ESP-IDF |

---

## Payload Format (LoRa Transmission)

Packets transmitted follow this structure:

ID:<packet_id>;TEMP:<temperature_celsius>;TDS:<tds_ppm>

Example:

ID:42;TEMP:21.87;TDS:512.44

The receiver sends back an ACK to confirm packet delivery:

ACK:42


---

## Key Software Features

| Functionality | Description |
|---------------|-------------|
| ADC calibration | Uses ESP-IDF calibration (curve fitting or line fitting) if supported by hardware |
| TDS measurement | Reads multiple samples and averages the ADC value |
| Temperature compensation | TDS conversion formula incorporates temperature adjustment |
| LoRa reliability | Implements retry + ACK system; packet considered valid only after acknowledgment |
| Flash persistence | Unsent packets are appended to `spiffs/pendentes.csv` and retransmitted on the next cycle |
| Deep sleep management | Significantly reduces energy consumption between measurement intervals |

---

## Configuration Parameters (located in code)

| Constant | Description |
|----------|-------------|
| `TEMPO_SONO_S` | Deep sleep duration between measurements |
| `NUM_AMOSTRAS_TDS` | Number of ADC samples for averaging |
| `TEMPO_ESTABILIZACAO_SENSOR_TDS` | TDS warm-up time after powering the probe |
| `TEMPERATURA_PADRAO` | Fallback value if DS18B20 read fails |

---

## Requirements

- ESP32 development board
- ESP-IDF v5.x or newer
- LoRa module (SX1276/SX1278 family)
- DS18B20 digital temperature sensor
- TDS analog probe

---

## Build and Flash Instructions

```sh
idf.py build
idf.py flash
idf.py monitor
```

Ensure that SPIFFS is configured in the partition table and that the LoRa library is correctly included in the project.

## Possible Extensions

- Forward processed data to a cloud platform via LoRa Gateway (MQTT or HTTP)
- Use CRC validation inside payload for additional data integrity
- Integrate calibration curve for TDS probe based on reference solutions

## License

This project is open for academic and research use.

## Citation (if used in academic documents)

If referencing this implementation, cite it as:

```
Fajardo, Nicolas. ESP32-Based Low-Power Water Quality Monitoring System Using LoRa and Flash Persistence, 2025.
```
