# I2C Interface, genuine Micropython
import uos
import machine
from time import sleep
import utime
from machine import Pin, I2C
from bme680 import *

recv_buf="" # receive buffer global variable

#print()
#print("Machine: \t" + uos.uname()[4])
#print("MicroPython: \t" + uos.uname()[3])


i2c=I2C(1,sda=Pin(2), scl=Pin(3), freq=400000)    #initializing the I2C method 
bme = BME680_I2C(i2c=i2c)
uart0 = machine.UART(0, baudrate=115200)
print(uart0)


delay = 0.1
elapsed = delay

while True:
    temperature = str(round(bme.temperature, 2)) + ' C'
    humidity = str(round(bme.humidity, 2)) + ' %'
    pressure = str(round(bme.pressure, 2)) + ' hPa'
    gas = str(round(bme.gas/1000, 2)) + ' kOhms'
    print('Temperature:', temperature)
    print('Humidity:', humidity)
    print('Pressure:', pressure)
    print('Gas:', gas)
    #print('Altitude: {} meters'.format(bme.altitude))
    print('Duration Logged: {:.2f} s'. format(elapsed))
    elapsed += delay
    print('\n')
    sleep(delay)