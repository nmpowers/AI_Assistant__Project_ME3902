# main.py (MicroPython) — I2S mic → USB CDC framed PCM
import machine, utime, struct

# I2S pins
SCK = machine.Pin(14)   # BCLK
WS  = machine.Pin(15)   # LRCLK/WS
SD  = machine.Pin(16)   # DOUT

RATE    = 16000
I2S_ID  = 0
BYTES   = 4096              # I2S DMA buffer (multiple of 4)
FRAMESZ = 1024              # samples per frame sent to host (16-bit)
MAGIC   = b'FRME'
seq     = 0

# Onboard LED toggles every frame so you can SEE it’s streaming
try:
    led = machine.Pin(25, machine.Pin.OUT)
except:
    led = None

# Init I2S (most MEMS mics output 24/32-bit left-justified)
i2s = machine.I2S(
    I2S_ID,
    sck=SCK, ws=WS, sd=SD,
    mode=machine.I2S.RX,
    bits=32,
    format=machine.I2S.MONO,
    rate=RATE,
    ibuf=BYTES
)

raw   = bytearray(FRAMESZ * 4)     # 32-bit samples
pcm16 = bytearray(FRAMESZ * 2)     # 16-bit payload

def downshift_32_to_16(src32, dst16):
    # Convert 32-bit little-endian signed samples to 16-bit LE by >> 16
    mv32 = memoryview(src32)
    mv16 = memoryview(dst16)
    for i in range(FRAMESZ):
        s0 = int.from_bytes(mv32[i*4:i*4+4], "little", True)
        s1 = s0 >> 16
        mv16[i*2:i*2+2] = int.to_bytes(s1, 2, "little", True)

# Write to same CDC as REPL — RUN HEADLESS (don’t keep Thonny open)
import sys
w = sys.stdout.buffer

while True:
    n = i2s.readinto(raw)
    if not n:
        utime.sleep_ms(1)
        continue
    downshift_32_to_16(raw, pcm16)
    header = MAGIC + struct.pack("<HH", len(pcm16), seq & 0xFFFF)
    w.write(header)
    w.write(pcm16)
    try:
        w.flush()
    except:
        pass
    seq += 1
    if led:
        led.toggle()

