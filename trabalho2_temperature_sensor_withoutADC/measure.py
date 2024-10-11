from machine import Pin, time_pulse_us
import time
import math

# Configuração dos pinos
pin_charge = Pin(15, Pin.OUT)
pin_measure = Pin(14, Pin.IN)

# Constantes e parâmetros
R1 = 2191  # Resistência em ohms
R2 = 10023  # Resistência em ohms
C = 2.18e-6  # Capacitância em Farads
VIL = 1.2730  # Tensão lógica baixa em volts
VIH = 1.2812  # Tensão lógica alta em volts
VDD = 3.3  # Tensão de alimentação em volts
N0 = 548  # Valor inicial do contador
tloop = 2.6017e-6  # Tempo de loop em segundos

def measure_adc():
    # Carregar o capacitor
    pin_charge.value(1)
    time.sleep_us(10)
    pin_charge.value(0)
    
    # Medir o tempo de descarga
    discharge_time = time_pulse_us(pin_measure, 0) * 1e-6  # Tempo em segundos

    # Calcular A_in dependendo do valor do tempo medido
    if discharge_time < 0:  # Se o tempo for negativo, não conseguiu carregar até VIH
        Ain = calculate_ain_below_vih(discharge_time)
    else:  # Se o tempo for positivo, carrega até VIL
        Ain = calculate_ain_above_vih(discharge_time)

    return Ain

def calculate_ain_above_vih(td):
    # Calcula A_in quando td é medido (A_in >= VIL)
    exponent = -td / (R1 * C)
    Ain = VIL * (R1 + R2) / (R1 + R2 * math.exp(exponent))
    return Ain

def calculate_ain_below_vih(tc):
    # Calcula A_in quando tc é medido (A_in < VIH)
    exponent = -tc / (R1 * C)
    Ain = (VIH * (R1 + R2) - R2 * VDD * (1 - math.exp(exponent))) / (R1 + R2 * math.exp(exponent))
    return Ain

while True:
    voltage = measure_adc()
    print(f"Tensão medida: {voltage:.4f} V")
    time.sleep(1)
