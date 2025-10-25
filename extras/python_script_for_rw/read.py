import serial, struct, sys, time


ser = serial.Serial("COM3", 115200, timeout=144000) #'/dev/ttyACM0'
for i in range(50):
    if ser.in_waiting > 0: 
        line = ser.readline()
        print(line.decode('utf-8')) 
    time.sleep(0.01)

ser.write(b'OK')
f = open('log.txt','w')
while True:
    if ser.in_waiting > 0: 
        line = ser.readline()
        #print(line.decode('utf-8'))
        f.write(line.decode('utf-8'))
    time.sleep(0.01)
f.close()    
ser.close()
