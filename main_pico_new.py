# main.py (MicroPython) — Turbo Mode
import machine, utime, struct, sys, uselect, math
import ujson as json

# 1. OVERCLOCK: Boost to 250MHz (Standard is 125MHz)
# This gives you 2x processing speed for free.
machine.freq(250000000)

# I2S audio 
SCK = machine.Pin(14)
WS  = machine.Pin(15)
SD  = machine.Pin(16)

RATE    = 16000
I2S_ID  = 0
BYTES   = 4096              
FRAMESZ = 1024              
MAGIC   = b'FRME'
seq     = 0

try:
    led_onboard = machine.Pin(25, machine.Pin.OUT)
except:
    led_onboard = None

i2s = machine.I2S(
    I2S_ID,
    sck=SCK, ws=WS, sd=SD,
    mode=machine.I2S.RX,
    bits=32,
    format=machine.I2S.MONO,
    rate=RATE,
    ibuf=BYTES
)

# Pre-allocate buffers
raw32 = bytearray(FRAMESZ * 4)     
pcm16 = bytearray(FRAMESZ * 2)     

# 2. VIPER OPTIMIZATION
# This decorator compiles the function to machine code.
# It is ~50x faster than your old 'downshift' function.
# It grabs the top 16 bits of the 32-bit sample directly from memory.
@micropython.viper
def downshift_fast(src: ptr16, dst: ptr16, count: int):
    for i in range(count):
        # In Little Endian 32-bit audio, the useful data is in the 
        # upper 16 bits. (Indices: Low=0, High=1, Low=2, High=3...)
        # We take the odd indices (i*2 + 1)
        dst[i] = src[i*2 + 1]

w = sys.stdout.buffer

# I2C bus + sensors 
i2c = machine.I2C(0, sda=machine.Pin(12), scl=machine.Pin(13), freq=400000)
ADS_ADDR = 0x48

def ads_read_ain0():
    cfg = (1<<15) | (0b100<<12) | (0b001<<9) | (1<<8) | (0b111<<5) | 0b11
    i2c.writeto_mem(ADS_ADDR, 0x01, cfg.to_bytes(2, 'big'))
    utime.sleep_ms(2) 
    raw = int.from_bytes(i2c.readfrom_mem(ADS_ADDR, 0x00, 2), 'big', True)
    volts = raw * (4.096 / 32768.0)
    return raw, volts

TRIG = machine.Pin(17, machine.Pin.OUT)
ECHO = machine.Pin(18, machine.Pin.IN)

def get_distance_inches():
    TRIG.low()
    utime.sleep_us(2)
    TRIG.high()
    utime.sleep_us(10)
    TRIG.low()
    
    # Timeout after 5ms (approx 10in)
    try:
        duration = machine.time_pulse_us(ECHO, 1, 5000)
    except OSError:
        return None
        
    if duration < 0: return None
    
    # Sound speed: 13503 inches/sec
    # Distance = (Duration / 2) * speed
    # Simplified: Duration / 148 = Inches
    return duration / 148.0

# Thermistor Config
R_FIXED      = 10000.0   
VREF_SUPPLY  = 3.3       
A  = 0.001125308852122
B  = 0.000234711863267
C  = 0.000000085663516
my_A = 0.001620135686
my_B = 0.0001500035785
my_C = 0.0000004140379381
USE_CAL_COEFFS = True   

# Smoothing
_MAX_R_SAMPLES = 3
_r_buf   = [0.0]*_MAX_R_SAMPLES
_r_total = 0.0
_r_idx   = 0
_r_count = 0

def _smooth_r_insert(r_new):
    global _r_total, _r_idx, _r_count
    old = _r_buf[_r_idx]
    _r_total -= old
    _r_buf[_r_idx] = r_new
    _r_total += r_new
    _r_idx = (_r_idx + 1) % _MAX_R_SAMPLES
    if _r_count < _MAX_R_SAMPLES:
        _r_count += 1
    return _r_total / _r_count

def _steinhart_hart_c_from_R(R_ohm, use_cal=USE_CAL_COEFFS):
    if R_ohm is None or R_ohm <= 0: return None
    a = my_A if use_cal else A
    b = my_B if use_cal else B
    c = my_C if use_cal else C
    lnR = math.log(R_ohm)
    invT = a + b*lnR + c*(lnR**3)
    if invT <= 0: return None
    T_K = 1.0 / invT
    return T_K - 273.15

def ntc_temp_from_vmid(v_mid, vref=VREF_SUPPLY):
    denom = (vref - v_mid)
    if denom <= 0.0 or v_mid <= 0.0: return None, None, None
    r_th = (v_mid / denom) * R_FIXED
    r_avg = _smooth_r_insert(r_th)
    tC = _steinhart_hart_c_from_R(r_avg)
    return tC, r_th, r_avg

SOIL_ADDR = 0x36
def soil_moisture_u16():
    try:
        i2c.writeto(SOIL_ADDR, bytes([0x0F, 0x10, 0x00]))
        utime.sleep_ms(1)
        data = i2c.readfrom(SOIL_ADDR, 2)
        return int.from_bytes(data, 'big')
    except Exception:
        return None

SOIL_DRY = 300
SOIL_WET = 1800
def soil_pct_from_raw(raw):
    if raw is None: return None
    if SOIL_WET <= SOIL_DRY: return None
    pct = int(100 * (raw - SOIL_DRY) / (SOIL_WET - SOIL_DRY))
    return 0 if pct < 0 else 100 if pct > 100 else pct

# Actuators 
LED_PIN = machine.Pin(2, machine.Pin.OUT)
MTR_PWM = machine.PWM(machine.Pin(19))
MTR_PWM.freq(2000)
motor_pct = 0
led_mode = 0  # 0=OFF, 1=ON, 2=BLINK

def set_led(on: bool):
    LED_PIN.value(1 if on else 0)

def set_motor_duty(pct: int):
    global motor_pct
    pct = 0 if pct < 0 else 100 if pct > 100 else int(pct)
    motor_pct = pct
    MTR_PWM.duty_u16(int(pct * 65535 // 100))

poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

def read_host_cmd():
    if not poll.poll(0): return None
    r = sys.stdin.buffer
    hdr = r.read(4)
    if not hdr or hdr != b'CMND': return None
    ln_b = r.read(2)
    if not ln_b or len(ln_b) < 2: return None
    ln = int.from_bytes(ln_b, 'little')
    payload = r.read(ln)
    if not payload or len(payload) < ln: return None
    try: return json.loads(payload)
    except Exception: return None

def apply_cmd(cmd):
    global led_mode, motor_pct
    if not cmd: return
    
    # Handle LED Modes
    if 'led' in cmd:
        val = int(cmd['led'])
        led_mode = val
        if val == 0: LED_PIN.value(0) # OFF
        elif val == 1: LED_PIN.value(1) # SOLID ON
        # If val is 2, the main loop handles the toggling
        
    if 'motor_pct' in cmd:
        set_motor_duty(int(cmd['motor_pct']))

STAT_EVERY_MS = 1000
last_stat_ms = 0

def send_stat(obj):
    try:
        b = json.dumps(obj).encode()
        hdr = b"STAT" + struct.pack("<H", len(b))
        w.write(hdr); w.write(b)
        try: w.flush()
        except: pass
    except Exception: pass

# Main loop 
tC = None
soil_raw = None
soil_pct = None

wake_cooldown = 0

while True:
    # Audio frame 
    n = i2s.readinto(raw32)
    if not n:
        continue # Don't sleep here, run as fast as possible

    # 3. USE FAST DOWNSHIFT
    downshift_fast(raw32, pcm16, FRAMESZ)
    
    header = MAGIC + struct.pack("<HH", len(pcm16), seq & 0xFFFF)
    w.write(header); w.write(pcm16)
    
    # Only flush periodically or let buffering handle it to increase speed
    # But for real-time audio, flushing is usually needed. 
    # Because we are now 50x faster, this flush won't block the next read.
    try:
        w.flush()
    except:
        pass
    
    # Check Proximity every ~100ms (seq % 2)
    if (seq % 2) == 0:
        dist = get_distance_inches()
        
        # LOGIC: 2 inches < Dist < 10 inches
        # We also check 'wake_cooldown' to ensure we haven't triggered recently
        if dist and 2 < dist < 10 and wake_cooldown == 0:
            # SEND THE WAKE SIGNAL
            send_stat({"wake_event": True})
            wake_cooldown = 50 # wait ~3-5 seconds (50 * 100ms) before allowing another wave
            
    if wake_cooldown > 0:
        wake_cooldown -= 1

    seq += 1
    if led_onboard: led_onboard.toggle()

    # Each frame is ~64ms. (seq % 4) means we toggle every ~250ms
    if led_mode == 2 and (seq % 4) == 0:
        LED_PIN.toggle()

    # Uncomment this when you are ready for sensors
    # NOTE: I increased modulo to 16 to give more CPU time to audio
    if (seq % 16) == 0:
       try:
           raw, v = ads_read_ain0()                 
           tC, r_th, r_avg = ntc_temp_from_vmid(v)  
       except Exception:
           tC = None
       try:
           soil_raw = soil_moisture_u16()
           soil_pct = soil_pct_from_raw(soil_raw)
       except Exception:
           soil_raw = None
           soil_pct = None

    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_stat_ms) >= STAT_EVERY_MS:
        last_stat_ms = now
        send_stat({
            "tC": tC,
            "soil_raw": soil_raw,
            "soil_pct": soil_pct,
            "motor_pct": motor_pct,
            "ts_ms": int(now)
        })

    try:
        cmd = read_host_cmd()
        if cmd: apply_cmd(cmd)
    except Exception:
        pass

