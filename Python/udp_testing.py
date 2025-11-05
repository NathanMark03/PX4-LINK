import socket
import ast
from random import random

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to a specific IP and port
UDP_IP = "127.0.0.1" 
UDP_PORT = 65431
UDP_SEND = 65432  
sock.bind((UDP_IP, UDP_PORT))

int_16_max = 65535
n = 0

def _byte_decode(x):
    # j = 1
    data = []
    for i in range(0, len(x), 2):
        # if(~j & 1): # is even
        big = x[i] << 8 # first 8 bits
        little = x[i-1] # last 8 bits
        num = big + little
        if(big >> 15): # negative
            num = -1 * ((num ^ int_16_max) + 1) # twos compliment
        data.append(num/127) # all data *127
        # j += 1
    return data

def _pack_bytes(y):
    b1 = "b\'"
    be = "\'"
    h1 = "\\x"
    packed_bytes = b1
    Q = 10000
    singedb = 1 << 15 # 0b1000000000000000
    for num in y: # -1 <= num <= 1
        num *= Q # float q value
        if(num<0):  # find singed eqivalent
            num = ((-num) ^ int_16_max) + 1
        hn = hex(int(num))[2:] # get rid of 0x
        while len(hn) < 4: # pad with zeros
            hn = "0"+hn
        big = hn[0:2]  # first two bytes
        little = hn[2:] # last two bytes
        big = ("0" + big) if len(big) < 2 else big
        little = ("0" + little) if len(little) < 2 else little
        packed_bytes += ((h1 + little) + (h1 + big)) # little endien
    byte_object = ast.literal_eval(packed_bytes + be)
    return byte_object

def _num_gen():
    return random()


while n < 1000:
    # Receive data
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes

    translated_data = _byte_decode(data)

    # packed = _pack_bytes([_num_gen(), _num_gen(), _num_gen(), _num_gen(), 0, 0, 0, 0]) # sample data
    packed = _pack_bytes([0.0010000000474974513, 0.0010000000474974513, 0.0010000000474974513, 0.0010000000474974513, 0, 0, 0, 0]) # sample data

    unpacked = _byte_decode(packed)

    i = 0
    for item in unpacked:
        unpacked[i] = item * 127
        i+=1

    print("Raw data:", data)
    print("Message Array:", translated_data)
    print("Packed data:", packed)
    print("Unpacked data:", unpacked)    

    sock.sendto(packed, (UDP_IP, UDP_SEND))

    n += 1