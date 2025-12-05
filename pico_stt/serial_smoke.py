# serial_smoke.py
import serial, struct, time
PORT = "/dev/tty.usbmodem2101"   # <- put your exact device here
MAGIC = b"FRME"

with serial.Serial(PORT, 115200, timeout=1) as s:
    print("Opened", s.port)
    seen, t0 = 0, time.time()
    while True:
        b = s.read(1)
        if not b: 
            continue
        if b != MAGIC[:1]:
            continue
        if s.read(3) != MAGIC[1:]:
            continue
        hdr = s.read(4)
        if len(hdr) < 4:
            continue
        length, seq = struct.unpack("<HH", hdr)
        payload = s.read(length)
        if len(payload) < length:
            continue
        seen += 1
        if seen % 50 == 0:
            dt = time.time() - t0
            print(f"{seen} frames in {dt:.1f}s (~{seen/dt:.1f} fps)")