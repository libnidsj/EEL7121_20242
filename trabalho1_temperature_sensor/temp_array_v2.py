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
    global temps, num_measurements, adc, count, num_measurements, count_stage
    val = adc.read_u16()
    temps.extend(val.to_bytes(2, 'big'))
    count = count + 1
    if num_measurements == count:
        timer.deinit()
        count = 0
        result = stats_bytes(temps)
        
        # print("avg: %.4f\tstd: %.4f\tvar: %.4f\tn_meas: %d" % (result["avg"], result["std"], result["var"], num_measurements)) # Plot with num_measurements var
        #print("avg: %.4f\tstd: %.4f\tvar: %.4f" % (result["avg"], result["std"], result["var"])) # Plot all data
        print("avg: %.4f\tvar: %.4f" % (result["avg"], result["var"])) # Plot only variance and avg
        
        # Do all 3 measurements sequentially
        if count_stage == 0:
            num_measurements = 100
            count_stage = count_stage + 1
            tmr1 = Timer(period=1, mode=Timer.PERIODIC, callback=timer_callback)
        elif count_stage == 1:
            num_measurements = 1000
            count_stage = count_stage + 1
            tmr1 = Timer(period=1, mode=Timer.PERIODIC, callback=timer_callback)
        else:
            num_measurements = 10000
            count_stage = 0
        temps = bytearray()
        
def timer_run(timer):
    if count == 0:
        tmr1 = Timer(period=1, mode=Timer.PERIODIC, callback=timer_callback)
    

adc = ADC(ADC.CORE_TEMP)
num_measurements = 100
count = 0
count_stage = 0
temps = bytearray()

tmr1 = Timer(period=1, mode=Timer.PERIODIC, callback=timer_callback)
tmr2 = Timer(period=60000,  mode=Timer.PERIODIC, callback=timer_run)

while 1:
    pass







                




