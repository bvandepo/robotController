// B.Vandeportaele 2024
#include "robotcontroller.h"
////////////////////////////////////////////////////////////////////////////
void RobotController::getInfo()
{
    for (uint8_t n = 0; n < nbmotor; n++)
        motor[n].getInfo();
}
////////////////////////////////////////////////////////////////////////////
RobotController::RobotController() // uint8_t nbmotorinit)
{
    controllerState = enumControllerState::FastTorque; // pour l'instant défaut
    cptUARTFrame = 0;
    /*
    nbmotor = nbmotorinit;
    if (nbmotor > NBMOTORMAX)
        while (true)
            ; // bloque le programme
    motor = new MotorController[nbmotor];
    */
    nbmotor = NBMOTORMAX;
    motor = new MotorController[NBMOTORMAX];
}
////////////////////////////////////////////////////////////////////////////
void RobotController::task()
{
    if (receiveUARTTask()) // peut faire changer controllerState
    {
        switch (controllerState)
        {
        case enumControllerState::Config:
            // TODO rediriger commande vers un moteur en particulier, on n'est pas pressé...
            break;
        case enumControllerState::FastTorque:
        case enumControllerState::FastAngle:
            sendFastCANtask();
            receiveFastCANtask(); // fonction bloquante  , je ne peux pas tester sans les controleurs moteurs... sauf en config ou tous les moteurs sont non existant
            sendUARTTask();
            break;
        }
    }
}
////////////////////////////////////////////////////////////////////////////
void RobotController::receiveFastCANtask()
{
    // bloquant jusqu'à avoir obtenu la réponse
    // réponse à la requette  TorqueClosedloopControlCommand() ou PositionTrackingControlCommandWithSpeedLimit();
    for (uint8_t n = 0; n < nbmotor; n++)
        while (!motor[n].receiveCAN())
            ;
    // TODO gérer timeout
    //  bloquant jusqu'à avoir obtenu la réponse
    //  réponse à la requette  ReadMultiturnAngleCommand()
    for (uint8_t n = 0; n < nbmotor; n++)
        while (!motor[n].receiveCAN())
            ;
    // TODO gérer timeout
}
////////////////////////////////////////////////////////////////////////////
void RobotController::sendFastCANtask()
{
    uint32_t tim_debut = micros(); // TODO: gérer rollover de la variable
    uint32_t delaius = 1200;
    switch (controllerState)
    {
    case enumControllerState::FastTorque:
        for (uint8_t n = 0; n < nbmotor; n++)
            motor[n].TorqueClosedloopControlCommand();
        break;
    case enumControllerState::FastAngle:
        for (uint8_t n = 0; n < nbmotor; n++)
            motor[n].PositionTrackingControlCommandWithSpeedLimit();
        break;
    }
    while ((micros() - tim_debut) < delaius)
        ;
    tim_debut = micros();
    for (uint8_t n = 0; n < nbmotor; n++)
        motor[n].ReadMultiturnAngleCommand();
    while ((micros() - tim_debut) < delaius)
        ;
}
////////////////////////////////////////////////////////////////////////////
void RobotController::shutdownMotorCANtask()
{
    // TODO: gérer si comm perdue avec le programme python au bout d'un certain temps, arrêter les moteurs
    for (uint8_t n = 0; n < nbmotor; n++)
    {
        motor[n].iqControl = 0;
        motor[n].TorqueClosedloopControlCommand();
    }
}
////////////////////////////////////////////////////////////////////////////
void RobotController::sendUARTTask()
{
    // #define BUF_UART_SND_SIZE 32
    //     static unsigned char emitBuff[BUF_UART_SND_SIZE]; // au moins de quoi stocker 1 messages
    unsigned char CKSUM;
    unsigned char c;
    // Serial.println("sendUARTTask()");
    if (controllerState == enumControllerState::FastTorque)
    {
        Serial.write(0x55);
        Serial.write(0x55);
    }
    else if (controllerState == enumControllerState::FastAngle)
    {
        Serial.write(0x56);
        Serial.write(0x56);
    }

    if ((controllerState == enumControllerState::FastTorque) || (controllerState == enumControllerState::FastAngle))
    {
        Serial.write(cptUARTFrame);
        CKSUM = cptUARTFrame;
        for (uint8_t n = 0; n < nbmotor; n++)
        {
            c = motor[n].iq >> 8;
            Serial.write(c);
            CKSUM += c;
            c = motor[n].iq & 0xff;
            Serial.write(c);
            CKSUM += c;
            c = (motor[n].motorAngle >> 24); //&0xFF;
            Serial.write(c);
            CKSUM += c;
            c = (motor[n].motorAngle >> 16); //&0xFF;
            Serial.write(c);
            CKSUM += c;
            c = (motor[n].motorAngle >> 8); //&0xFF;
            Serial.write(c);
            CKSUM += c;
            c = (motor[n].motorAngle >> 0); //&0xFF;
            Serial.write(c);
            CKSUM += c;
        }
        Serial.write(CKSUM);
    }
    // TODO mode config
    if (controllerState == enumControllerState::Config)
    {
    }
}
////////////////////////////////////////////////////////////////////////////
bool RobotController::receiveUARTTask()
{ //  bool retval = false pour trame non décodée
    /*
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'q') { //q pour arreter/redemmarrer le moteur
          etattache1  = (etattache1  + 1) % 2;
          //for (uint8_t i = 0; i < 28; i++)  Serial.print(etattache1 );
          char  tab[2][28]={"chaine123456789","blabla123456789"};
          Serial.print(tab[etattache1 ]);
        }
      }
    */

#define BUF_UART_REC_SIZE 64
    static unsigned char receiveBuff[BUF_UART_REC_SIZE]; // au moins de quoi stocker 2 messages
    static unsigned char receiveBuffIndex = 0;
    static unsigned char prevByte = 0;
    static unsigned char newByte = 0;
    static unsigned char receiveUARTState = 0;
    static unsigned char CKSUM;

    // il y a une probabilité faible de synchroniser le début de trame sur un faux header, mais le cksum permettra de détecter et normalement la synchro se fera sur la trame suivante

    while (Serial.available())
    { // traite tous les caractères disponibles dans le buffer
        if (receiveBuffIndex >= (BUF_UART_REC_SIZE - 1))
        {
            receiveBuffIndex = 0; // repart au début du buffer pour éviter fuite mémoire
            receiveUARTState = 0;
            return false;
        }
        newByte = Serial.read();
        switch (receiveUARTState)
        {
        case 0:
            if ((prevByte == 0x55) && (newByte == 0x55))
            {
                // Serial.println("rapide couple");
                receiveUARTState = 1;
                receiveBuffIndex = 0;
                CKSUM = 0;
            }
            else if ((prevByte == 0x56) && (newByte == 0x56))
            {
                // Serial.println("rapide angle");
                receiveUARTState = 2;
                receiveBuffIndex = 0;
                CKSUM = 0;
            }
            else if ((prevByte == 0x54) && (newByte == 0x54))
            {
                // Serial.println("lent config");
                receiveUARTState = 3;
                receiveBuffIndex = 0;
                CKSUM = 0;
            }
            break;
        case 1: // trame  rapide couple
            receiveBuff[receiveBuffIndex] = newByte;
            if (receiveBuffIndex >= 9)
            { // newByte est le Cksum reçu
                if (newByte == CKSUM)
                {
                    cptUARTFrame = receiveBuff[0];
                    motor[0].iqControl = (receiveBuff[1] << 8) | receiveBuff[2]; // MSB First
                    motor[1].iqControl = (receiveBuff[3] << 8) | receiveBuff[4]; // MSB First
                    motor[2].iqControl = (receiveBuff[5] << 8) | receiveBuff[6]; // MSB First
                    motor[3].iqControl = (receiveBuff[7] << 8) | receiveBuff[8]; // MSB First
                    receiveUARTState = 0;
                    controllerState = enumControllerState::FastTorque;
                    // Serial.println("rapide couple OK");
                    return true;
                }
                else
                { // probleme cksum
                    receiveUARTState = 0;
                    // Serial.println("rapide couple NOK");
                    return false;
                }
            }
            else
            {
                receiveBuffIndex++;
                CKSUM += newByte;
            }
            break;
        case 2: // trame  rapide angle
            receiveBuff[receiveBuffIndex] = newByte;
            if (receiveBuffIndex >= 9 + 8)
            { // newByte est le Cksum reçu
                if (newByte == CKSUM)
                {
                    cptUARTFrame = receiveBuff[0];
                    motor[0].angleControl = (receiveBuff[1] << 24) | receiveBuff[2] << 16 | (receiveBuff[3] << 8) | receiveBuff[4];     // MSB First
                    motor[1].angleControl = (receiveBuff[5] << 24) | receiveBuff[6] << 16 | (receiveBuff[7] << 8) | receiveBuff[8];     // MSB First
                    motor[2].angleControl = (receiveBuff[9] << 24) | receiveBuff[10] << 16 | (receiveBuff[11] << 8) | receiveBuff[12];  // MSB First
                    motor[3].angleControl = (receiveBuff[13] << 24) | receiveBuff[14] << 16 | (receiveBuff[15] << 8) | receiveBuff[16]; // MSB First
                    receiveUARTState = 0;
                    controllerState = enumControllerState::FastAngle;
                    // Serial.println("rapide position OK");
                    return true;
                }
                else
                { // probleme cksum
                    receiveUARTState = 0;
                    // Serial.println("rapide position NOK");
                    return false;
                }
            }
            else
            {
                receiveBuffIndex++;
                CKSUM += newByte;
            }
            break;
        case 3: // trame  lent config
            // TODO définit format pour envoyer des commandes specifique à un moteur et renvoyer la réponse,
            // en mode bourrin, ne pas chercher à décoder, mais juste à aiguiller
            //     controllerStat=enumControllerState::Config;
            break;
        }
        prevByte = newByte;
    }
    return false; // si sortie de la boucle c'est qu'il n'y a plus de caractères dans la fifo réception uart
}
////////////////////////////////////////////////////////////////////////////

// code poubelle pour aller y chercher si besoin:

// tests pour mesurer les chronogrammes à l'oscillo
//   MotorModelReadingCommand(0); //read un moteur non présent sur le bus
//   MotorModelReadingCommand(ID_controleur); //read un moteur présent sur le bus  pour lire le temps de réponse
//   PositionTrackingControlCommandWithSpeedLimit(0,0,0);
//   PositionTrackingControlCommandWithSpeedLimit(ID_controleur,0,0);
// for (int i=0;i<6;i++) PositionTrackingControlCommandWithSpeedLimit(ID_controleur,0,0);
// bidon();

// for (uint8_t i = 0; i < 28; i++) Serial.print(" ");

// Serial.write(0x55);

// tester avec des vraies données

// robot.motor[1].receive();
// robot.motor[0].receive();

/*

uint8_t etattache1 = 1;

////////////////////////////////////////////////////////////////////////////
void tache1()
{
  static uint16_t cpt_msg_send = 0;
    if (debug) {
      Serial.print("Send    ");
      Serial.print(cpt_msg_send);
      Serial.print(": ");
    }

  if (etattache1 == 1)
  {
    // valeurs du programme de vincent:
    // PositionTrackingControlCommandWithSpeedLimit(ID_controleur,0x4342,0x00005A00+cpt_msg_send*10);
    // 0x4342=17218degrés par seconde?
    // 0x00005A00=230,40 degrés?
    // parquer à 10degrés par seconde vers position 0 degrés:
    // PositionTrackingControlCommandWithSpeedLimit(ID_controleur, 10, 0 * 100);
    // essuie glace
    // PositionTrackingControlCommandWithSpeedLimit(ID_controleur, 180, 180 * (cpt_msg_send % 2) * 100);
    // commande en position sinus
    float pos = 90 * sin(millis() * 2 * 3.14159 / 1000);
    // PositionTrackingControlCommandWithSpeedLimit(ID_controleur, 180, pos * 100);
    // ReadMultiturnEncoderOriginalPositionDataCommand(ID_controleur);
    // ReadMultiturnAngleCommand(ID_controleur);
    // ReadMotorStatus1AndErrorFlagCommand(ID_controleur);
    // ReadMotorStatus2Command(ID_controleur);
    // MotorStopCommand(ID_controleur);
    // commande en courant
    float pos2 = 40 * sin(millis() * 2 * 3.14159 / 1000);
    //  TorqueClosedloopControlCommand (ID_controleur, (int16_t)pos2 );

    //  motor[0].TorqueClosedloopControlCommand ((int16_t)pos2 );
    //  motor[1].TorqueClosedloopControlCommand ((int16_t)pos2 );

    //    float pos3 =90+ 90 * sin(millis() * 2 * 3.14159 / 1000);
    // pos3=0;
    // motor[0].PositionTrackingControlCommandWithSpeedLimit(36000,pos*100);
    // Serial.print("pos3: ");
    // Serial.println(pos3);

    uint32_t pos4 = 100 * 180 * ((uint32_t)(millis() / 1000) % 2);
    // Serial.print("pos4: ");
    // Serial.println(pos4);

    uint32_t tim_debut = micros();
    // TODO: gérer rollover de la variable
    uint32_t delaius = 1200; // 2500;

    robot.motor[0].maxSpeed = 720;
    robot.motor[0].angleControl = pos4;
    robot.motor[1].maxSpeed = 720;
    robot.motor[1].angleControl = pos4;
    robot.motor[0].PositionTrackingControlCommandWithSpeedLimit();
    robot.motor[1].PositionTrackingControlCommandWithSpeedLimit();
    while ((micros() - tim_debut) < delaius)
      ;
    // delay(2);

    tim_debut = micros();
    robot.motor[0].ReadMultiturnAngleCommand();
    robot.motor[1].ReadMultiturnAngleCommand();
    while ((micros() - tim_debut) < delaius)
      ;
    // delay(2);

    //  motor[1].ReadMultiturnAngleCommand();

    // motor[0].PositionTrackingControlCommandWithSpeedLimit(720, pos4);
    // motor[0].ReadMultiturnAngleCommand();

    // motor[0].ReadMultiturnEncoderOriginalPositionDataCommand();
  }
  else
  {
    //  TorqueClosedloopControlCommand (ID_controleur, (int16_t)0);
    robot.motor[0].iqControl = 0;
    robot.motor[1].iqControl = 0;
    robot.motor[0].TorqueClosedloopControlCommand();
    robot.motor[1].TorqueClosedloopControlCommand();
  }
  cpt_msg_send++;
}
  */
////////////////////////////////////////////////////////////////////////////
// génère une valeur de comptage pour le 1° octet de données (change aussi le CRC à la fin)
/*void bidon() {
  static uint8_t cpt1 = 0;
  static uint8_t cpt2 = 0;
  cpt1++;
  if (cpt1 >= 100) {
    cpt1 = 0;
    cpt2++;
  }
  struct can_frame canMsgSend;
  canMsgSend.can_id  = 0 ; // 0x140 + 0;         //CAN id as ...
  canMsgSend.can_dlc = 8; //0x01;               //CAN data length as 8
  canMsgSend.data[0] = cpt2;
  for (uint8_t i = 1; i < 8; i++) canMsgSend.data[i] = 0x00;
  mcp2515.sendMessage(&canMsgSend);     //Sends the CAN message
  printserial(canMsgSend);
  }
*/

////////////////////////////////////////////////////////////////////////////
