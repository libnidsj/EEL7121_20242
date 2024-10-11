from machine import Pin, time_pulse_us
import time

# Configuração dos pinos
pin_charge = Pin(15, Pin.OUT)
pin_measure = Pin(14, Pin.IN)

def measure_resistance():
    # Carregar o capacitor
    pin_charge.value(1)
    time.sleep_us(10)
    pin_charge.value(0)
    
    # Medir o tempo de descarga
    discharge_time = time_pulse_us(pin_measure, 0)
    
    return discharge_time

while True:
    discharge_time = measure_resistance()
    print(f"Tempo de descarga: {discharge_time} us")
    time.sleep(1)
