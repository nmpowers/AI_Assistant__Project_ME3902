import machine, utime, struct, sys, uselect, math
import ujson as json

TRIG = machine.Pin(17, machine.Pin.OUT)
ECHO = machine.Pin(18, machine.Pin.IN)

def get_distance_inches():
    TRIG.low()
    utime.sleep_us(2)
    TRIG.high()
    utime.sleep_us(10)
    TRIG.low()
    
    # Timeout after 30ms (approx 15 feet)
    try:
        duration = machine.time_pulse_us(ECHO, 1, 30000)
    except OSError:
        return None
        
    if duration < 0: return None
    
    # Sound speed: 13503 inches/sec
    # Distance = (Duration / 2) * speed
    # Simplified: Duration / 148 = Inches
    return duration / 148.0

while True: 
    dist = get_distance_inches()
    if dist is None:
        print("Distance: Out of Range")
    else:
        print("Distance: {:.2f} inches".format(dist))
    utime.sleep(1)