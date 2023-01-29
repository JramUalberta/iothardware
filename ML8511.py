import machine
import utime

#connections:
    #Sensor-to-MCU
    #3.3V-to-3.3V
    #Vin-to-ADC_VREF
    #GND-to-AGND
    #EN-to-3.3V
    #OUT-to-ADC1

uv_sensor_datain = machine.ADC(1)  #get analog data from the ML8511 UV Sensor

conversion_factor = 3.3/65536      #to convert analog data to voltage

#interpolation formula to get UV index

def interpolation(uv_volt_reading, volt_in_min, volt_in_max, uv_intensity_min, uv_intensity_max):
    return (uv_volt_reading - volt_in_min) * (uv_intensity_max - uv_intensity_min) / (volt_in_max - volt_in_min) + uv_intensity_min

#loop to output voltage and UV index values

while True:
    uv_volt_reading = uv_sensor_datain.read_u16() * conversion_factor
    uv_index = interpolation(uv_volt_reading,0.99,2.8,0.0,15.0)
    print('Voltage Output:', uv_volt_reading,'V',',','UV Index: ', uv_index)
    utime.sleep(0.25)
