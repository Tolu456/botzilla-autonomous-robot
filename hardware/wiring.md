# Wiring — Phase C (final firmware)

Pin numbers match `firmware/phase-c-final/final_code_v2/final_code_v2.ino` (Arduino Mega 2560).

## Power
| Connection | To |
|---|---|
| TB6612 VCC (both) | Mega 5 V logic rail |
| TB6612 GND (both) | Common ground |
| TB6612 STBY (both) | D5 |
| TB6612 VM (both) | Battery + (fused) |
| HC-SR04 VCC | Mega 5 V (never raw LiPo) |

## Motor drivers
| Motor | Driver channel | IN1 | IN2 | PWM | Inverted |
|---|---|---|---|---|---|
| Left front  | TB1 A | D22 | D23 | D6 | yes |
| Left rear   | TB1 B | D24 | D25 | D7 | yes |
| Right front | TB2 A | D26 | D27 | D8 | no |
| Right rear  | TB2 B | D28 | D29 | D9 | no |

N20 motor terminals M1/M2 go to the driver outputs (AO1/AO2, BO1/BO2). Encoder channels C1/C2 were reserved on D30–D37.

## Sensors
| Sensor | TRIG | ECHO |
|---|---|---|
| Front HC-SR04 | D2 | D11 |
| Left HC-SR04  | D3 | D12 |
| Right HC-SR04 | D4 | D13 |

| MPU9250 | Mega |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SDA | D20 |
| SCL | D21 |
