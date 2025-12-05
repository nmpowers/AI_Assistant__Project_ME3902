# serial_meter.py
import serial, struct, numpy as np, time
PORT = "/dev/cu.usbmodem2101"   # <-- put YOUR exact device here
BAUD = 115200
MAGIC = b"FRME"
RATE = 16000
FRAMESZ = 1024

def read_frame(s):
    # robust resync on FRME
    while True:
        b = s.read(1)
        if not b: continue
        if b == MAGIC[:1] and s.read(3) == MAGIC[1:]:
            hdr = s.read(4)
            if len(hdr) < 4: continue
            ln, seq = struct.unpack("<HH", hdr)
            pay = s.read(ln)
            if len(pay) == ln:
                return pay, seq

def meter(x):
    rms = float(np.sqrt(np.mean(x*x)))
    pk  = float(np.max(np.abs(x)))
    # crude dBFS
    db  = -120.0 if rms <= 1e-6 else 20*np.log10(rms)
    bar = "#" * int(min(50, max(0, db + 60)//2))  # -60..0 dBFS ≈ 0..30 #
    return rms, pk, db, bar

with serial.Serial(PORT, BAUD, timeout=1) as s:
    print("Opened", s.port, "— speak near the mic")
    while True:
        pay, _ = read_frame(s)
        x = np.frombuffer(pay, dtype="<i2").astype(np.float32)/32768.0
        # remove DC (helps if there’s any offset)
        x = x - np.mean(x)
        rms, pk, db, bar = meter(x)
        print(f"lvl={rms:.4f}  pk={pk:.4f}  {db:6.1f} dBFS  [{bar:<30}]", end="\r")