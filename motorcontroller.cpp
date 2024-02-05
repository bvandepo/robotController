// B.Vandeportaele 2024

#include "motorcontroller.h"

////////////////////////////////////////////////////////////////////////////
/*struct can_frame {
    uint32_t can_id;  // 32 bit CAN_ID + EFF/RTR/ERR flags
    uint8_t  can_dlc;
    uint8_t  data[8];
  };
  documentation sur: https://www.kernel.org/doc/Documentation/networking/can.txt
*/

MotorController::MotorController()
{
    mcp2515 = NULL;
    iqControl = 0;
    maxSpeed = 100; // DPS par défaut
    angleControl = 0;
    debug = false;
};
// CAN_SPEED et CAN_CLOCK définis dans mcp2515.h
void MotorController::setConf(uint8_t IDinit, uint8_t ARDUINOPINinit, CAN_SPEED canSpeedinit, CAN_CLOCK canClockinit)
{
    ID = IDinit;
    ARDUINOPIN = ARDUINOPINinit;
    canSpeed = canSpeedinit;
    canClock = canClockinit;
    mcp2515 = new MCP2515(ARDUINOPIN);
    mcp2515->reset();
    // pour carte aliexpress //  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); //Sets CAN at speed 1MBPS and Clock 8MHz
    // pour carte seedstudio //mcp2515.setBitrate(CAN_1000KBPS,MCP_16MHZ);
    mcp2515->setBitrate(canSpeed, canClock);
    mcp2515->setNormalMode();
}
////////////////////////////////////////////////////////////////////////////
void MotorController::printserial(can_frame canMsg)
{
    if (debug)
    {
        uint16_t id = canMsg.can_id;
        if (id < 0x1000)
            Serial.print('0');
        if (id < 0x100)
            Serial.print('0');
        if (id < 0x10)
            Serial.print('0');
        Serial.print(id, HEX);
        Serial.print(": ");
        for (uint8_t i = 0; i < canMsg.can_dlc; i++)
        {
            uint8_t b = canMsg.data[i];
            if (b < 0x10)
                Serial.print('0');
            Serial.print(b, HEX);
            if (i < 7)
                Serial.print(":");
            else
                Serial.println();
        }
    }
}
////////////////////////////////////////////////////////////////////////////
// page 45
// ID= numero du moteur par exemple 6
// int16_t iqControl  0.01A/LSB
void MotorController::TorqueClosedloopControlCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0xA1;      // Torque closed-loop control command
        canMsgSend.data[1] = 0x00;
        canMsgSend.data[2] = 0x00;
        canMsgSend.data[3] = 0x00;
        canMsgSend.data[4] = (uint8_t)(iqControl);
        canMsgSend.data[5] = (uint8_t)(iqControl >> 8);
        canMsgSend.data[6] = 0x00;
        canMsgSend.data[7] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
// n'a pas l'air d'arreter le moteur :(
void MotorController::MotorStopCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0x81;      // p44
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::ReadMotorStatus2Command()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0x9C;      // p39
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::ReadMotorStatus1AndErrorFlagCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0x9A;      // p36
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::ReadMultiturnAngleCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0x92;      // p34
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::ReadMultiturnEncoderOriginalPositionDataCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0x61;      // p24
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::SystemOperatingModeAcquisition()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0x70;      // p69  System operating mode acquisition
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::SystemRuntimeReadCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0xB1;      // System runtime read command
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::SystemSoftwareVersionDateReadCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0xB2;      // p 76
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
void MotorController::MotorModelReadingCommand()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0xB5;      // p 86
        for (uint8_t i = 1; i < 8; i++)
            canMsgSend.data[i] = 0x00;
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
// page 62 de https://www.myactuator.com/_files/ugd/cab28a_9d344907863b4033ada6d9318d511b6e.docx?dn=RMD-X%20Servo%20Motor%20Control%20Protocol%20V3.6.docx
// maxSpeed 1dps/LSB
// angleControl: 0.01degree/LSB
void MotorController::PositionTrackingControlCommandWithSpeedLimit()
{
    /*
      Serial.print("uint16_t maxSpeed: ");
      Serial.print(maxSpeed);
      Serial.print("   int32_t angleControl:");
      Serial.println(angleControl);
    */
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgSend;
        canMsgSend.can_id = 0x140 + ID; // CAN id as ...
        canMsgSend.can_dlc = 0x08;      // CAN data length as 8
        canMsgSend.data[0] = 0xA5;      // Position tracking control command with speed limit
        canMsgSend.data[1] = 0x00;
        canMsgSend.data[2] = (uint8_t)(maxSpeed);
        canMsgSend.data[3] = (uint8_t)(maxSpeed >> 8);
        canMsgSend.data[4] = (uint8_t)(angleControl);
        canMsgSend.data[5] = (uint8_t)(angleControl >> 8);
        canMsgSend.data[6] = (uint8_t)(angleControl >> 16);
        canMsgSend.data[7] = (uint8_t)(angleControl >> 24);
        mcp2515->sendMessage(&canMsgSend); // Sends the CAN message
        printserial(canMsgSend);
    }
}
////////////////////////////////////////////////////////////////////////////
bool MotorController::parser(can_frame canMsg)
{
    if (mcp2515 != NULL)
    {
        // Gérer l'ID du message recu, la réponse contient l'ID du moteur donc il est possible de différentier les réponses de plusieurs moteurs sur un bus
        uint8_t idmotor = canMsg.can_id - 0x240;
        if (idmotor >= INDICEMAX)
            return false;
        // Checker que le bon nombre d'octets est disponible avant d'y accéder
        if (canMsg.can_dlc != 8)
            return false;
        // ok, c'est bon on peut décoder
        switch (canMsg.data[0])
        {
        ///////////////////
        case 0xA1: // page 46
        case 0xA5: // page 63
        case 0x9C: // page 39 ce sont les mêmes réponses
            temperature = canMsg.data[1];
            iq = (uint16_t)canMsg.data[2] | ((uint16_t)canMsg.data[3] << 8);
            speed = (uint16_t)canMsg.data[4] | ((uint16_t)canMsg.data[5] << 8);
            degree = (uint16_t)canMsg.data[6] | ((uint16_t)canMsg.data[7] << 8);
            if (debug)
            {
                Serial.print("temperature: ");
                Serial.print(temperature);
                Serial.print(" iq: ");
                Serial.print(iq);
                Serial.print(" speed: ");
                Serial.print(speed);
                Serial.print(" degree: ");
                Serial.print(degree);
                Serial.println();
            }
            // pour plotter sans debug
            //       Serial.println(iq);

            break;
        ///////////////////
        case 0x9A: ////page 37 Read Motor Status 1 and Error Flag Command
            temperature = canMsg.data[1];
            rlyCtrlRslt = (uint8_t)canMsg.data[3];
            voltage = (uint16_t)canMsg.data[4] | ((uint16_t)canMsg.data[5] << 8);
            errorState = (uint16_t)canMsg.data[6] | ((uint16_t)canMsg.data[7] << 8);
            if (debug)
            {
                Serial.print("temperature: ");
                Serial.print(temperature);
                Serial.print(" rlyCtrlRslt: ");
                Serial.print(rlyCtrlRslt);
                Serial.print(" voltage: ");
                Serial.print(voltage);
                Serial.print(" errorState: ");
                Serial.print(errorState, HEX);
                if ((errorState & 0x0002) != 0)
                    Serial.print("  Motor stall");
                if ((errorState & 0x0004) != 0)
                    Serial.print("  low pressure");
                if ((errorState & 0x0008) != 0)
                    Serial.print("  overvoltage");
                if ((errorState & 0x0010) != 0)
                    Serial.print("  overcurrent");
                if ((errorState & 0x0040) != 0)
                    Serial.print("  Power overrun");
                if ((errorState & 0x0100) != 0)
                    Serial.print("  speeding");
                if ((errorState & 0x1000) != 0)
                    Serial.print("  Motor temperature over temperature");
                if ((errorState & 0x2000) != 0)
                    Serial.print("  Encoder calibration error");
                Serial.println();
            }
            // pour plot sans debug
            //       Serial.println(iq);
            break;
        ///////////////////
        case 0xB1: // page 76 System runtime read command
            sysRunTime = (uint32_t)canMsg.data[4] | ((uint32_t)canMsg.data[5] << 8) | ((uint32_t)canMsg.data[6] << 16) | ((uint32_t)canMsg.data[7] << 24);
            if (debug)
            {
                Serial.print("sysRunTime: ");
                Serial.println(sysRunTime);
            }
            break;
        ///////////////////
        case 0x81: // page 44 Motor stop command
            if (debug)
            {
                Serial.println("Motor stop command ");
            }
            break;
        ///////////////////
        case 0xB2: // page 78 System software version date read command
            versionDate = (uint32_t)canMsg.data[4] | ((uint32_t)canMsg.data[5] << 8) | ((uint32_t)canMsg.data[6] << 16) | ((uint32_t)canMsg.data[7] << 24);
            if (debug)
            {
                Serial.print("versionDate: ");
                Serial.println(versionDate);
            }
            break;
        ///////////////////
        case 0xB5: // p86 Motor model reading command (0xB5)
            for (uint8_t i = 0; i < 7; i++)
                model[i] = canMsg.data[i + 1];
            model[7] = 0; // fin de chaine
            if (debug)
            {
                Serial.print("model: ");
                Serial.println(model);
            }
            break;
        ///////////////////////////////
        case 0x70: // p69 System operating mode acquisition
            operatingMode = canMsg.data[7];
            if (debug)
            {
                Serial.print("operatingMode: ");
                switch (operatingMode)
                {
                case 1:
                    Serial.println("Current loop mode");
                    break;
                case 2:
                    Serial.println("Speed loop mode");
                    break;
                case 3:
                    Serial.println("Position loop mode");
                    break;
                default:
                    Serial.println("ERROR");
                    break;
                }
            }
            break;
        ///////////////////
        case 0x61: // page 25 Read multi-turn encoder original position data command
            encoderRaw = (uint32_t)canMsg.data[4] | ((uint32_t)canMsg.data[5] << 8) | ((uint32_t)canMsg.data[6] << 16) | ((uint32_t)canMsg.data[7] << 24);
            if (debug)
            {
                Serial.print("encoderRaw: ");
                Serial.println(encoderRaw);
            }
            break;
        ///////////////////
        case 0x92: // page 34 Read multi-turn angle command (0x92)
            motorAngle = (uint32_t)canMsg.data[4] | ((uint32_t)canMsg.data[5] << 8) | ((uint32_t)canMsg.data[6] << 16) | ((uint32_t)canMsg.data[7] << 24);
            if (debug)
            {
                Serial.print("motorAngle: ");
                Serial.println(motorAngle);
            }
            break;
        ///////////////////////////////continuer avec les autres messages à décoder
        default:
            break;
        }
    }
    return true; // si le moteur n'existe pas...
}
////////////////////////////////////////////////////////////////////////////
void MotorController::getInfo()
{ // TODO checker si les buffers sont assez grands pour stocker les différentes requettes et réponse..
  //  NON Cela ne marche pas, il faut envoyer une requette et attendre que le driver de moteur réponde
    /*
      SystemOperatingModeAcquisition();
      SystemRuntimeReadCommand();
      SystemSoftwareVersionDateReadCommand();
      MotorModelReadingCommand();
    */
}
////////////////////////////////////////////////////////////////////////////
bool MotorController::receiveCAN()
{
    if (mcp2515 != NULL)
    {
        struct can_frame canMsgRcv;
        static uint16_t cpt_msg_rcv = 0;
        if (mcp2515->readMessage(&canMsgRcv) == MCP2515::ERROR_OK)
        {
            if (debug)
            {
                Serial.print("Rcv     ");
                Serial.print(cpt_msg_rcv);
                Serial.print(": ");
                printserial(canMsgRcv);
            }
            parser(canMsgRcv);
            cpt_msg_rcv++;
            return true;
        }
        else
            return false;
    }
    else //(mcp2515 == NULL), pas de moteur, rien à recevoir
        return true;
}
////////////////////////////////////////////////////////////////////////////
