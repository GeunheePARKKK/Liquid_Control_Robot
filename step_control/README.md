
## 참고

- [Pololu DRV8825 Stepper Motor Driver Carrier](https://www.pololu.com/product/2133)
- [TI DRV8825 Datasheet](https://www.ti.com/lit/ds/symlink/drv8825.pdf)

---

## 제어 ESP32-S3 에 DRV8825 두 개 — 배선표

| DRV8825 핀 | 드라이버 #1 | 드라이버 #2 |
|---|---|---|
| STEP | GPIO 15 | GPIO 14 |
| DIR | GPIO 16 | GPIO 21 |
| ENABLE | GND | GND |
| RESET | 3V3 | 3V3 |
| SLEEP | 3V3 | 3V3 |
| M0 / M1 / M2 | 미연결 | 미연결 |
| FAULT | 미연결 | 미연결 |
| GND (맨 아래) | ESP32 GND | ESP32 GND |
| VMOT | 18V (+) | 18V (+) |
| GND (VMOT 아래) | 18V (−) | 18V (−) |
| A1, A2 | 코일 1 | 코일 1 |
| B1, B2 | 코일 2 | 코일 2 |

- VMOT–GND 사이 전해 100µF/35V, 드라이버마다 하나.
- 18V(−) 와 ESP32 GND 공통.
- 4·5 는 CAN 이 쓰므로 안 씀.
