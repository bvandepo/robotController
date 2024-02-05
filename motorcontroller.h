// B.Vandeportaele 2024

#include <SPI.h>     //Library for using SPI Communication
#include <mcp2515.h> //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)

#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H


//indice max de moteur sur un robot (étiquette sur le moteur)
#define INDICEMAX 20



////////////////////////////////////////////////////////////////////////////
//TODO: passer en private les attributs et méthodes et gérer set et get
class MotorController
{
public:
 // paramètres controlables
  int16_t iqControl;    // pour commande 0xA1 0.01A/LSB
  uint16_t maxSpeed;    // pour commande 0xA5 1dps/LSB
  int32_t angleControl; // pour commande 0xA5 0.01degree/LSB

  // valeurs lues depuis les drivers de moteur
  int16_t iq;           // torque current  0.01A/LSB
  int32_t motorAngle;  // 0.01°/LSB.

  uint8_t temperature;  // 1°C/LSB
  int16_t speed;        // 1dps/LSB
  int16_t degree;       // 1degree/LSB, maximum range ±32767degree.
  uint32_t sysRunTime;  // ms depuis mise sous tension du controleur
  uint32_t versionDate; // The date format is in the format of year, month, and day, such as 20211126. -> bizarre, bertrand obtient : 2022090601
  char model[8];        // ascii
  uint8_t operatingMode;
  int32_t encoderRaw;
  uint8_t rlyCtrlRslt; // 1 represents the brake release command, and 0 represents the brake lock command.
  uint16_t voltage;    // 0.1V/LSB
  uint16_t errorState;
  MotorController();
  uint8_t ID;
  uint8_t ARDUINOPIN;
  CAN_SPEED canSpeed;
  CAN_CLOCK canClock;
  MCP2515 *mcp2515; // NULL indique que le moteur n'existe pas et donc ne peux pas être piloté, change dans setConf si appelée
  
  bool debug;

  void setConf(uint8_t IDinit, uint8_t ARDUINOPINinit, CAN_SPEED canSpeedinit, CAN_CLOCK canClockinit);
  
  
  void TorqueClosedloopControlCommand();
  void MotorStopCommand();
  void ReadMotorStatus2Command();
  void ReadMotorStatus1AndErrorFlagCommand();
  void ReadMultiturnAngleCommand();
  void ReadMultiturnEncoderOriginalPositionDataCommand();
  void SystemOperatingModeAcquisition();
  void SystemRuntimeReadCommand();
  void SystemSoftwareVersionDateReadCommand();
  void MotorModelReadingCommand();
  void PositionTrackingControlCommandWithSpeedLimit();
  bool parser(can_frame canMsg);
  void getInfo();
  bool receiveCAN();
  void printserial(can_frame canMsg);
};
////////////////////////////////////////////////////////////////////////////



#endif