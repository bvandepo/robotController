# B.Vandeportaele 2024
#Si c'était à refaire: fonction de communications communes Arduino et PC en C++ + bindings pour python...

import random
import time
import serial,sys
import threading
import struct

#décommenter la ligne suivane pour faire des tests des fonctions
#ProgrammeDeTest=True

#################################################################################################################################
#DOC
#pack et unpack https://docs.python.org/3/library/struct.html
#type bytes: https://www.toppr.com/guides/python-guide/references/methods-and-functions/methods/built-in/bytes/python-bytes/
#byte est immutable, j'utilise bytearray pour construire le message: https://stackoverflow.com/questions/27001419/how-to-append-to-bytes-in-python-3  
#################################################################################################################################
def generateBinaryFrameIqControl(cpt,iqControl):
  #cpt 8 bits non signé pour compter les trames%256 et pouvoir associer requetes et réponses
  #iqControl liste de 4 valeurs de couple  sur 16bits signé: //int16_t iqControl  0.01A/LSB
  if len(iqControl)!=4: return False
  ba=bytearray()
  ba.extend(b'\x55') #headerCouple
  ba.extend(b'\x55') 
  b = struct.pack('>B', cpt%256) # b de classe bytes
  ba.extend(b)
  cksum=cpt
  for i in range(4):
    b = struct.pack('>h', iqControl[i]) # b de classe bytes
    ba.extend(b)
    cksum=cksum+b[0]+b[1]
  b= struct.pack('>B', cksum%256)
  ba.extend(b) #fin message cksum
  print(ba)
  return ba
#################################################################################################################################
if 'ProgrammeDeTest' in vars():
  generateBinaryFrameIqControl(128,[0,-1,2,32767])
  generateBinaryFrameIqControl(3,[0,0,0,0])
  generateBinaryFrameIqControl(250,[0,128,3,1])
  ba=generateBinaryFrameIqControl(128,[0,128,3,1])
#print(len(ba))
#exit()
#################################################################################################################################
def generateBinaryFrameAngleControl(cpt,angleControl):
  #cpt 8 bits non signé pour compter les trames%256 et pouvoir associer requetes et réponses
  #angleControl liste de 4 valeurs de position 32 bits signés
  if len(angleControl)!=4: return False
  ba=bytearray()  
  ba.extend(b'\x56') #headerPosition   
  ba.extend(b'\x56') 
  b = struct.pack('>B', cpt%256) # b de classe bytes
  ba.extend(b) 
  cksum=cpt
  for i in range(4):
    b = struct.pack('>i', angleControl[i]) # b de classe bytes
    ba.extend(b)
    cksum=cksum+b[0]+b[1]+b[2]+b[3]
  b= struct.pack('>B', cksum%256)
  ba.extend(b) #fin message cksum
  print(ba)
  return ba

#################################################################################################################################
if 'ProgrammeDeTest' in vars():
  generateBinaryFrameAngleControl(128,[0,-1,2,32767]) #crc 0xfc 
  
#################################################################################################################################
def decodeBinaryFrameIqMotorAngle(b):
  #cpt 8 bits non signé pour compter les trames%256 et pouvoir associer requetes et réponses  
  #print(len(b));  print(type(b));  print(type(b[0]));  print((b[0]))  
  CKSUMcalc=0 #checker le CRC
  for n in range(2,27):
     CKSUMcalc=CKSUMcalc+b[n]
  HEAD1,HEAD2,CPT,IQ0,ANGLE0,IQ1,ANGLE1,IQ2,ANGLE2,IQ3,ANGLE3,CKSUM=struct.unpack('>BBBhihihihiB', b)
  if (CKSUMcalc%256) == CKSUM:
    print(HEAD1,HEAD2,CPT,IQ0,ANGLE0,IQ1,ANGLE1,IQ2,ANGLE2,IQ3,ANGLE3,CKSUM)
    iQ=[IQ0,IQ1,IQ2,IQ3]
    motorAngle=[ANGLE0,ANGLE1,ANGLE2,ANGLE3]
    return True,CPT,iQ,motorAngle  # True pour decodage ok
  else:
     return False,CPT,[0,0,0,0],[0,0,0,0]  # False pour decodage Nok

#################################################################################################################################
if 'ProgrammeDeTest' in vars():
  ok,cpt,iQ,motorAngle=decodeBinaryFrameIqMotorAngle(b'UU\xd8\x00\x04\x00\x00\xd8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xb4')  
  ok,cpt,iQ,motorAngle=decodeBinaryFrameIqMotorAngle(b'UU\x13\x00\x04\x00\x00\xd8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xef')
  #TRES PEU TESTEE!!!!!  
  exit()
#################################################################################################################################
class CommunicationClass(threading.Thread):
######################################  #####
    def __init__(self, dataClass):
      threading.Thread.__init__(self)
      self._dataClass = dataClass # Pour gérer les données échangées entre CommunicationClass et d'autre classe
      self._typeCommunication=0 #config par defaut lent, 1 pour commande rapide couple, 2 pour commande rapide angle
      #self._period = 0.003 #periodicité d'envoi des commandes
      self._period = 1 #periodicité d'envoi des commandes
      self._nextCall = time.time()
      self._continuer=True  #pour tuer le thread depuis l'exterieur
      self._cptFrame=0 #identifiant du numero de trame
      self._bufferRec=bytearray() #pour stocker les octets reçus sur serial en attente de reception


      #TODO: mettre ces données dans self._dataClass 
      self._iqControl=[0,0,0,0] #valeurs souhaitées pour la commande rapide en courant (Torque)
      self._angleControl=[0,0,0,0] #valeurs souhaitées pour la commande rapide en position (Angle)
      
      #TODO: passer le nom du port, le débit etc en argv
      #tentative d'ouverture du port, 2 noms possibles
      port='/dev/ttyACM0'
      #baudrate=9600
      baudrate=1000000
      try:
        self.serial = serial.serial_for_url(port, baudrate,timeout=0,  parity=serial.PARITY_NONE,    stopbits=serial.STOPBITS_ONE,   bytesize=serial.EIGHTBITS) 
      except:
        try: 
          port='/dev/ttyUSB0'
          self.serial = serial.serial_for_url(port, baudrate,timeout=0,  parity=serial.PARITY_NONE,    stopbits=serial.STOPBITS_ONE,   bytesize=serial.EIGHTBITS) 
        except:
            print("erreur ouverture port")
            exit(0)       
      #vider les buffers de reception avant de commencer!!!
      try:
          nb=self.serial.inWaiting()
          print("nb bytes flushed: "+str(nb))
          incomming=self.serial.read(nb)  #data = self.serial.read(1).decode("utf8")
      except ValueError:
          incomming=''
###########################################
    def stop(self):
        self._continuer=False
###########################################
    def send(self):    
        #TODO il faut former un message indiquant quel numéro de moteur est ciblé et donner la liste d'octets à lui envoyer
        '''    
        print("send\n");sys.stdout.flush()        
        s='q' 
        self.serial.write(s.encode(encoding='UTF-8',errors='strict'))        
        '''
###########################################
    def receive(self):
        #TODO il faut décoder le message indiquant quel numéro de moteur est ciblé et récupérer la liste d'octets
        '''
        print("receive\n");sys.stdout.flush()
        try:
            nb=self.serial.inWaiting()
            #print("nb:"+str(nb)
            incomming=self.serial.read(nb)            
        #data = self.serial.read(1).decode("utf8")
        except ValueError:
            incomming=''
        if incomming:                 
          incomming=str(incomming.decode("utf-8",errors='ignore')) #read all char in buffer, ignore les caractères non utf8          
          print(incomming)     
         '''                
###########################################
    def sendFast(self):        
        print("sendFast\n");sys.stdout.flush()
        if self._typeCommunication==1:
          ba=generateBinaryFrameIqControl(self._cptFrame,self._iqControl)         
        if self._typeCommunication==2:
          ba=generateBinaryFrameAngleControl(self._cptFrame,self._angleControl)        
        self.serial.write(ba)
        self._cptFrame=self._cptFrame+1
###########################################
    def receiveFast(self):
        #gère la resynchro sur début de trame reçue
        #TODO faire reception bloquante jusqu'à timeout
        print("receiveFast\n");sys.stdout.flush()
        
        #bloquant jusqu'à recu=True
        recu=False
        while not recu:
          try:
              nb=self.serial.inWaiting()
              #print("nb:"+str(nb))
              incomming=self.serial.read(nb)            
              #print(type(incomming))
              self._bufferRec.extend(incomming) 
              #data = self.serial.read(1).decode("utf8")
          except ValueError:           
              print(" ValueError")
          #TODO; gerer les trames autres que UU
        #chercher le premier entête suivi d'un nombre suffisant d'octets              
          while( len(self._bufferRec)>27):
            if ( ( self._bufferRec[0]!=85) and (self._bufferRec[1]!=85) or ( self._bufferRec[0]!=86) and (self._bufferRec[1]!=86)) : #reponse angle ou torque
              ok,cpt,iQ,motorAngle=decodeBinaryFrameIqMotorAngle(self._bufferRec[0:28])                
              if ok:
                print(self._bufferRec[0:27])
                del self._bufferRec[0:27]
                recu=True
                #TODO: exploiter cpt,iQ,motorAngle ->dataclass

              else: #c'était peut être un faux header, effacer le 1° caractère du buffer seulement et retenter
                del self._bufferRec[0]   
            elif ( self._bufferRec[0]!=84) and (self._bufferRec[1]!=84): #reponse config
              print("Trame config pas encore gérée")
            else:
              del self._bufferRec[0]
                      
        #print("nb:"+str(len(self._bufferRec)))
        #print(self._bufferRec)


##########################mettre ici les données échangées entre CommunicationClass et d'autre classe#################
    def run(self):
        while self._continuer:            
          if self._typeCommunication==0: #communication lente pour config
            #TODO
            print("slow")
            comm.send()
            comm.receive()
          if self._typeCommunication==1 or self._typeCommunication==2: #communication rapide
            print("fast")
            comm.sendFast()
            comm.receiveFast()
        # sleep until next execution
          self._nextCall = self._nextCall + self._period
          if (self._nextCall - time.time())>0:             
            time.sleep(self._nextCall - time.time())
          else: #si jamais la tache unitaire est plus longue que la periodicité souhaitée, il ne faut pas faire un sleep <0
            time.sleep(0.001)
#################################################################################################################################

#classe préve pour l'échange des données entre les threads de communication et d'application/calcul/affichage
#################################################################################################################################
class MyDataClass():
    def __init__(self):
        self.reinitialize()        
    def reinitialize(self):
        #TODO: mettre ici les données échangées entre CommunicationClass et d'autres classes
        self.cadence=0
        self.lastTimeDisplayed=time.time() #horodatage mis à jour par la fonction d'affichage, pour detecter quand la fenêtre est fermée et que l'affichage n'est plus fait
        self.firstSampleTime=time.time() #utilisé pour mesurer la cadence des mesures
#################################################################################################################################

data = MyDataClass()
comm = CommunicationClass(data)

comm._typeCommunication=1 #passe en mode rapide

try:
  comm.start()  
  comm.join()
 
except KeyboardInterrupt:  #quitter avec CTRL+C
  print("bye")
  comm.stop() 
  exit(0)


'''

            #print("incomming:"+str(incomming))
            #print("self.chaineadecoder:"+str(self.chaineadecoder))
            #sys.stdout.write(incomming)
            #sys.stdout.flush()
            #nbvar=0           
            #time.sleep(0.01)   #do not monopolize CPU
        



            
            #if not a:
            #  print("List is empty")
            #Using the implicit booleanness of the empty list is quite Pythonic.





            while '\n' in self.chaineadecoder: #split data line by line and store it in var
              var, self.chaineadecoder = self.chaineadecoder.split('\n', 1)
              #print("var:"+str(var))
              nbvar=nbvar+1
              #print("nbvar:"+str(nbvar))
              if 'reset' in var:  #if str(var)==str('reset'):
                  print("self._dataClass.reinitialize()")
                  self._dataClass.reinitialize()
                  listeval=[] #reinitialise la liste à partir du reset                   
              else:
                  try:
                    val=float(var)
                    listeval.append(val)                      
                  except ValueError:
                    print("Not a float")                
            '''

'''


            #print("len(listeval):"+str(len(listeval)))
            #for i in range(len(listeval)):
            #  #print("i:"+str(i))           
            #  self._dataClass.XData.append(len(self._dataClass.XData) + 1)
            #  val=listeval[i]
            #  self._dataClass.YData.append(val)
              #pour debug,valeurs constantes:
              #self._dataClass.YData=[1,2,3,4,5,6,7,8,9,10,10,10]
          
          #nbmesures=len(self._dataClass.YData)
          #if nbmesures>0:
          #  self._dataClass.moyenne=np.mean(self._dataClass.YData)
          #  self._dataClass.ecartType=np.std(self._dataClass.YData)
          #  self._dataClass.cadence=1/((time.time()-self._dataClass.firstSampleTime)/nbmesures)
#https://www.python-course.eu/python3_formatted_output.php
          #  if displayFetch: print("nb mesures: ", nbmesures," cadence: " ,"{0:4.2f}".format(self._dataClass.cadence),  "moyenne: {0:6.3f}".format(self._dataClass.moyenne), " écart type: {0:6.3f}".format(self._dataClass.ecartType))

'''

'''import numpy as np
def bitsToFloat(b):
    s = struct.pack('>l', b)
    return struct.unpack('>f', s)[0]

def floatToBits(f):
    s = struct.pack('>f', f)
    return struct.unpack('>l', s)[0]

def twos_comp_two_bytes(msb, lsb):
    return struct.unpack('>f', bytes([msb, lsb]))[0]  

########

val_infloat=100.5
val_inbits=floatToBits(val_infloat)

print(hex(val_inbits))

#get msb and lsb
msb, lsb= divmod(val_inbits, 1<<8)

#  redo the conversion to check if it is ok
val_inbits_res=(msb<<8) | (lsb>>8)

print(val_inbits_res)

val_infloat_res=bitsToFloat(val_inbits_res)


print("initial value "+ str(val_infloat))
print("final value "+ str(val_infloat_res))
'''