import serial
import csv

file = raw_input('Save File As: ')
saveFile = open(file, 'w')

serialport = raw_input('Enter Port: ')
port1 = serialport

print "Connecting to....", port1

arduino = serial.Serial(port1, 115200)

print "Arduino detected"

while True: 
    time.sleep(.01)
    data = arduino.readline()
    saveFile.write(data)
    print data
