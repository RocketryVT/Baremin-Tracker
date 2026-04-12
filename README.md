# Bareman-Tracker

## Current Setup

- The Onboard GPS and LR1121 are currently not working so we are using breakouts.

### Pinout



## Original Pinout

RP2350 custom PCB with a LR1121 with this pinout:

| Pin | Function |
| --- | -------- |
| 6   | SPI0 SCK |
| 7   | SPI0 MOSI|
| 4   | SPI0 MISO|
| 5   | SPI0 NSS |
| 0   | LR1121 BUSY |
| 1   | LR1121 NRESET |
| 2   | LR1121 DIO1 (interrupt) |

With GPS on UART here:

| Pin | Function |
| --- | -------- |
| 16 | UART0 TX |
| 17 | UART0 RX |

LEDs on:

| Pin | Function |
| --- | -------- |
| 12  | STATUS   |
| 13  | TX       |
| 14  | RX       |

MS5607 on I2C here:

I2C0 SCL
I2C0 SDA

ISM-40609-D on I2C here:

I2C0 SCL = PIN 33 = GPIO 21
I2C0 SDA = PIN 32 = GPIO 20
Interrupt Pin = PIN 36 = GPIO 24
