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

    
    b = struct.pack('>B', cptcar%256) # b de classe bytes

    ba=bytearray()
    for n in range(nbcarframe):
        ba.extend(b)
    if debug_display: print(ba)    
    time_begin=time.time()
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
#obtenu avec adaptateur FTDI232 blanc latency mean: 0.01597547125816345 s  stdev: 0.00015723879877159837    
#  |__ Port 4: Dev 20, If 0, Class=Vendor Specific Class, Driver=ftdi_sio, 12M
#[36385.608489] usb 2-3.4: new full-speed USB device number 20 using xhci_hcd
# [36385.718916] usb 2-3.4: New USB device found, idVendor=0403, idProduct=6015, bcdDevice=10.00
# [36385.718934] usb 2-3.4: New USB device strings: Mfr=1, Product=2, SerialNumber=3
# [36385.718941] usb 2-3.4: Product: Chipi-X
# [36385.718946] usb 2-3.4: Manufacturer: FTDI
# [36385.718950] usb 2-3.4: SerialNumber: FT5V7INI
# [36386.302072] usbcore: registered new interface driver ftdi_sio
# [36386.302096] usbserial: USB Serial support registered for FTDI USB Serial Device
# [36386.302155] ftdi_sio 2-3.4:1.0: FTDI USB Serial Device converter detected
# [36386.302180] usb 2-3.4: Detected FT-X
# [36386.303123] usb 2-3.4: FTDI USB Serial Device converter now attached to ttyUSB0
