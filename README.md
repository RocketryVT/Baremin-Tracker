# Bareman-Tracker

RP2350 + RFM95W (SX1276) starter project using FreeRTOS and the in-repo LoRa stub driver.

Quick start (from repo root):

```
cmake -S projects/bareman_tracker -B build/bareman_tracker -G Ninja
cmake --build build/bareman_tracker
```

Notes:
- Update the pin map in `projects/bareman_tracker/src/main.cpp` to match your board wiring.
- Override the board if needed: `-DPICO_BOARD=<your_board>`.
