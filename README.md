# rpi5_ldrobot_ld14p
Capture LDROBOT LD14P LiDAR sensor data using Raspberry Pi 5 and Python

Step-by-step instructions in this article https://makerspet.com/blog/how-to-connect-l298n-motor-driver-to-esp32/

```
# Summarized output (one line per 12-point packet)
python3 ld14p_pi.py

# Raw output (one line per measurement)
python3 ld14p_pi.py --raw

# Or with a USB-to-serial adapter
python3 ld14p_pi.py /dev/ttyUSB0 --raw

# Stop the motor (data stream halts)
python3 ld14p_motor.py stop
python3 ld14p_motor.py stop --port /dev/ttyAMA0 --baud 230400

# Start / resume spinning
python3 ld14p_motor.py start

# Set scan rate to 6 Hz (valid 2-8)
python3 ld14p_motor.py speed 6

# Report current scan rate, or "stopped"
python3 ld14p_motor.py status
```

Expected output

```
LD14P: opening /dev/serial0 @ 230400 baud  (Ctrl-C to stop)
 pkt_start   pkt_end    rpm   min_mm   max_mm  n_valid
    186.20    192.16  358.0      198      204        9
    192.70    198.61  358.3      199      201       12
    199.15    205.05  358.3      200      211       12
    205.59    211.50  358.5      211      221       12
    212.04    217.96  358.5      222      236       12
    218.49    224.42  358.5      237      255       12
    224.95    230.87  358.0      257      270       12
    231.41    237.30  358.0      237      258       12
    237.83    243.76  358.0      220      235       12
    244.30    250.21  358.0      211      219       12
^C
Stopped.
```
