# Portable antagonistic tendon servo controller (C99)

This small library maps 10 normalized finger-joint commands to 20 STS3215 position targets: one flexor servo and one extensor servo per axis. The first implementation assumes the 10-axis layout used in this project (thumb 4, index 3, middle 3); adapt the axis count if the mechanism differs.

## Hardware boundary

The library deliberately does not implement a UART/half-duplex Feetech protocol driver because the MCU, serial adapter, baud rate, and bus implementation have not been specified. Provide `AT_SyncWritePositions` using your board's Feetech driver. It must send one synchronized/batched goal-position command for the supplied IDs. The callback returns `true` only when the bus accepts the command.

## Bring-up requirements

1. Confirm each servo is in position mode and verify its position-unit range and ID using the servo documentation/configuration tool.
2. Mechanically secure the hand and test one antagonistic pair at low speed/current limits before connecting all 20 servos.
3. Calibrate each servo's `open_position` and `closed_position` independently. Set them so one tendon takes up as the opposing tendon pays out, while maintaining only the intended light preload. Do not copy endpoint numbers between mirrored servos without checking their directions.
4. Read all 20 present positions and seed `current_position` with those measured values before calling `at_init`; the library intentionally does not assume a safe startup pose.
5. Set `max_step_per_update` based on control-loop rate and a conservative motion speed. `q` is flexion in [0, 1]; invalid or non-finite commands are rejected.
6. Add a hardware watchdog/emergency stop and monitor servo temperature, voltage, and load. This position-only library does not measure tendon tension, detect collisions, or guarantee a safe state after communication loss.

## Build

```sh
cc -std=c99 -Wall -Wextra -Werror -pedantic -c antagonistic_tendon.c
```

Link with the math library where required (for example, `-lm` on some host toolchains).
