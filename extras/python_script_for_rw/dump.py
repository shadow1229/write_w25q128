import serial, struct, sys, time
with open('data.bin', 'rb') as f:
    data_full = f.read()

four_mb = 2**22
idx = 0 

n_parts = 1 + (len(data_full)-1)//four_mb #= 6
print(n_parts)
if (idx >= n_parts):
    print("invalid part")
    raise ValueError
if (idx < n_parts-1): 
    data = data_full[ idx*four_mb : (idx+1)*four_mb]
else:
    data = data_full[ idx*four_mb :]
    
addr = idx*four_mb

ser = serial.Serial("COM3", 115200) #'/dev/ttyACM0'
for i in range(50):
    if ser.in_waiting > 0: 
        line = ser.readline()
        print(line.decode('utf-8')) 
    time.sleep(0.01)

ser.write(struct.pack('<I', addr))
ser.write(struct.pack('<I', len(data)))
ser.write(data)

while True:
    if ser.in_waiting > 0: 
        line = ser.readline()
        print(line.decode('utf-8'))
    time.sleep(0.01)