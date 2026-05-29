# rpi5_ldrobot_ld14p

Read data from **and** control the LDROBOT LD14P 2D LiDAR using a Raspberry Pi 5 (or Pi 4) and Python — no microcontroller in the middle.

Two small, dependency-light scripts:

- **`ld14p_pi.py`** — read live scan data (angle, distance, intensity) and print it.
- **`ld14p_motor.py`** — start, stop, and set the scan speed of the motor over UART.

## Tutorials

- **Part 1 — Connect & read:** https://makerspet.com/blog/tutorial-connect-ldrobot-ld14p-lidar-to-raspberry-pi-python/
- **Part 2 — Control the motor:** https://makerspet.com/blog/tutorial-control-ldrobot-ld14p-lidar-motor-raspberry-pi-python/

## Hardware

- LDROBOT LD14P LiDAR (ships with the JST GH 4-pin breakout cable)
- Raspberry Pi 4 or Pi 5 running Raspberry Pi OS (Bookworm or newer)
- A Pi power supply with ~300 mA of headroom for the LiDAR's motor + laser
- A few jumper wires

## Wiring (GPIO UART)

Reading data needs three wires; sending motor commands needs a fourth (Pi TX → LiDAR RX).

| LD14P cable | Pi GPIO | Board pin | Needed for |
|---|---|---|---|
| 5V  | 5 V          | pin 2 or 4 | power |
| GND | GND          | pin 6      | power |
| TX  | GPIO15 / RXD | pin 10     | reading data |
| RX  | GPIO14 / TXD | pin 8      | sending motor commands |

3.3 V logic on both sides — no level shifter needed.

## One-time Pi setup

On the Pi 5, `/dev/serial0` often points at the debug UART by default. Enable the
GPIO UART and free it from the serial console (full step-by-step in Part 1). In short:

- add `enable_uart=1` and `dtparam=uart0=on` to `/boot/firmware/config.txt`
- remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`
- `sudo systemctl disable --now serial-getty@ttyAMA0.service`
- `sudo usermod -aG dialout $USER`, then reboot

Install pyserial:

```
sudo apt update && sudo apt install python3-serial
# or: pip install pyserial
```

## Usage

```
# Summarized output (one line per 12-point packet)
python3 ld14p_pi.py

# Raw output (one line per measurement)
python3 ld14p_pi.py --raw

# Or with a USB-to-serial adapter
python3 ld14p_pi.py /dev/ttyUSB0 --raw

# Motor control (needs the pin-8 wire above)
python3 ld14p_motor.py status     # report current scan rate, or "stopped"
python3 ld14p_motor.py stop        # stop the motor (data stream halts)
python3 ld14p_motor.py start       # start / resume spinning
python3 ld14p_motor.py speed 6     # set scan rate to 6 Hz (valid 2-8)
python3 ld14p_motor.py stop --port /dev/ttyAMA0 --baud 230400
```

## Motor command reference

8-byte frames on the same UART: `0x54 | cmd | 0x04 | 4 payload | CRC-8` (poly 0x4D).

| Command  | Bytes |
|----------|-------|
| Start    | `54 A0 04 00 00 00 00 5E` |
| Stop     | `54 A1 04 00 00 00 00 4A` |
| Set 6 Hz | `54 A2 04 70 08 00 00 A1` |

Set-speed payload is deg/sec little-endian (Hz × 360), valid 2–8 Hz. Command bytes
and CRC are derived from the [kaiaai/LDS](https://github.com/kaiaai/LDS) library.

## Expected output

```
LD14P: opening /dev/serial0 @ 230400 baud  (Ctrl-C to stop)
 pkt_start   pkt_end    rpm   min_mm   max_mm  n_valid
    186.20    192.16  358.0      198      204        9
    192.70    198.61  358.3      199      201       12
    199.15    205.05  358.3      200      211       12
    205.59    211.50  358.5      211      221       12
    218.49    224.42  358.5      237      255       12
^C
Stopped.
```

## License

Apache License 2.0 — see [LICENSE](LICENSE).
