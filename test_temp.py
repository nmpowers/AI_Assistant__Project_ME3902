import machine, utime, struct, sys, uselect, math
import ujson as json

# Note: No ID (0 or 1) is needed for SoftI2C
i2c = machine.SoftI2C(sda=machine.Pin(12), scl=machine.Pin(13), freq=400000)
ADS_ADDR = 0x48

def ads_read_ain0():
    # Config: Single-ended AIN0, +/- 4.096V range
    cfg = (1<<15) | (0b100<<12) | (0b001<<9) | (1<<8) | (0b111<<5) | 0b11
    i2c.writeto_mem(ADS_ADDR, 0x01, cfg.to_bytes(2, 'big'))
    utime.sleep_ms(2) 
    raw = int.from_bytes(i2c.readfrom_mem(ADS_ADDR, 0x00, 2), 'big', signed=True)
    volts = raw * (4.096 / 32768.0)
    return raw, volts

# Thermistor Config
R_FIXED      = 10000.0   
VREF_SUPPLY  = 3.3       
# Steinhart-Hart Coefficients
my_A = 0.001620135686
my_B = 0.0001500035785
my_C = 0.0000004140379381

# Smoothing
_MAX_R_SAMPLES = 5  # Increased slightly for smoother testing
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

def _steinhart_hart_c_from_R(R_ohm):
    if R_ohm is None or R_ohm <= 0: return None
    lnR = math.log(R_ohm)
    invT = my_A + my_B*lnR + my_C*(lnR**3)
    if invT <= 0: return None
    T_K = 1.0 / invT
    return T_K - 273.15

def ntc_temp_from_vmid(v_mid, vref=VREF_SUPPLY):
    # Protect against divide by zero if v_mid equals vref
    denom = (vref - v_mid)
    if denom <= 0.001 or v_mid <= 0.0: return None, None, None
    
    # Voltage Divider Math for: 3.3V -> 10k -> Pin -> NTC -> GND
    r_th = (v_mid / denom) * R_FIXED
    r_avg = _smooth_r_insert(r_th)
    tC = _steinhart_hart_c_from_R(r_avg)
    return tC, r_th, r_avg

print("=== Thermistor Test Started ===")
print("Hold the sensor to see Temp rise and Ohms fall.")

while True:
    try:
        raw, v = ads_read_ain0()                 
        tC, r_th, r_avg = ntc_temp_from_vmid(v)
        
        # We print a formatted string for readability, 
        # but keep the JSON format if you are parsing this elsewhere.
        if tC is not None:
            print(f"Temp: {tC:.2f} °C | Volts: {v:.3f} V | Ohms: {int(r_avg)}")
        else:
            print(f"Error: Volts={v:.3f} (Check wiring)")
            
    except Exception as e:
        print("I2C Error:", e)

    utime.sleep(0.5) # Update every half second