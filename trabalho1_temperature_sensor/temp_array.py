from machine import ADC, Timer
import time
import math
import micropython

def stats_bytes(data):
    s = 0
    c = 0
    ssq = 0
    for i in range(0, len(data), 2):
        val = (data[i] << 8 | data[i+1])
        temp = 27.0 - ( ( ( 3.3 * (val / 65535.0) ) - 0.706 ) / 0.001721 )
        s = s + temp
        c = c + 1
    avg = s/c
    for i in range(0, len(data), 2):
        val = (data[i] << 8 | data[i+1])
        temp = 27.0 - ( ( ( 3.3 * (val / 65535.0) ) - 0.706 ) / 0.001721 )
        ssq = ssq + (temp - avg)**2
        
    variance = ssq/c
    stdev = math.sqrt(variance)
    return {"avg" : avg, "var" : variance, "std" : stdev}

def timer_callback(timer):
    global temps, num_measurements, adc, count
    val = adc.read_u16()
    temps.extend(val.to_bytes(2, 'big'))
    count = count + 1
    if num_measurements == count:
        timer.deinit()

adc = ADC(ADC.CORE_TEMP)
num_measurements = 10000
count = 0
temps = bytearray()

tmr1 = Timer(period=1, mode=Timer.PERIODIC, callback=timer_callback)

micropython.mem_info()

while num_measurements > count:
    pass

micropython.mem_info()

result = stats_bytes(temps)
print("avg: %.4f\tstd: %.4f\t var: %.4f" % (result["avg"], result["std"], result["var"]))





                



