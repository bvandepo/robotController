# B.Vandeportaele 2024

import time
import serial,sys
import struct

from statistics import mean,stdev 


#tentative d'ouverture du port, 2 noms possibles
port='/dev/ttyACM0'
#baudrate=9600
baudrate=1000000
try:
    serial = serial.serial_for_url(port, baudrate,timeout=0,  parity=serial.PARITY_NONE,    stopbits=serial.STOPBITS_ONE,   bytesize=serial.EIGHTBITS) 
except:
    try: 
        port='/dev/ttyUSB0'
        serial = serial.serial_for_url(port, baudrate,timeout=0,  parity=serial.PARITY_NONE,    stopbits=serial.STOPBITS_ONE,   bytesize=serial.EIGHTBITS) 
    except:
        print("erreur ouverture port")
        exit(0)       


#vider les buffers de reception avant de commencer!!!
try:
    nb=serial.inWaiting()
    print("nb bytes flushed: "+str(nb))
    incomming=serial.read(nb)  #data = self.serial.read(1).decode("utf8")
    print("vidage: "+str(incomming))
except ValueError:
    incomming=''

#delay pour laisser le temps à l'arduino de redémarrer après l'ouverture du port série!!!!!!! (piège)
time.sleep(3)

#revider les buffers de reception avant de commencer!!!
try:
    nb=serial.inWaiting()
    print("nb bytes flushed: "+str(nb))
    incomming=serial.read(nb)  #data = self.serial.read(1).decode("utf8")
    print("vidage: "+str(incomming))
except ValueError:
    incomming=''

debug_display=False

latency_list=[]
nbcarframe=28
for cptcar in range(1000):

    time_begin=time.time()
    b = struct.pack('>B', cptcar%256) # b de classe bytes

    ba=bytearray()
    for n in range(nbcarframe):
        ba.extend(b)
    if debug_display: print(ba)    
    serial.write(ba)

    nbcarrecus=0
    #bloquant jusqu'à recu=True
    while nbcarrecus<nbcarframe:
        try:
            nb=serial.inWaiting()        
            if nb>0:
                incomming=serial.read(nb)                    
                #bufferRec.extend(incomming)  
                if debug_display: print(incomming)
                nbcarrecus=nbcarrecus+nb
        except ValueError:           
            print(" ValueError")
    time_end=time.time()    
    latency=time_end-time_begin
    latency_list.append(latency)
    print("latence:"+str(latency))     

#print(latency_list)
print("latency mean: "+str(mean(latency_list))+ " s  stdev: "+str(stdev(latency_list)))


#Pour baudrate=1000000 bits/s
#obtenu avec clone chinois CH340: latency mean: 0.0027470083236694337 s  stdev: 0.00019057053680861285
#obtenu avec vrai arduino UNO R3: latency mean: 0.00407370376586914 s    stdev: 0.00041990298008214916
#obtenu avec vrai carte arduino UNO R3 mais en enlevant le ATMEGA et en bridgant RX et TX:
#                                 latency mean: 0.0040566914081573485 s  stdev: 0.00021642200912601025
