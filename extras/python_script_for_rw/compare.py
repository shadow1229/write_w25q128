import serial, struct, sys, time
with open('data.bin', 'rb') as f:
    data_full = f.read()

two_mb = 2**21 #22 -> 21 
idx = 0 #idx 0x0200000 -> 0x0000000
#split with 1MB segment
n_parts = 1 + (len(data_full)-1)//two_mb #= 6
data = data_full[ idx*two_mb : (idx+1)*two_mb]

log = []
with open('log.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
        if line.startswith('#'):
            continue
        words = line.strip().split()
        numbers = [ int(word) for word in words ]
        log.extend(numbers)
    
for i in range(two_mb):
    if data[i] != log[i]:
        print(i)
        break
        
print(log[16*(i//16):16*(i//16)+16])
print(data[16*(i//16):16*(i//16)+16])