// B.Vandeportaele 2024
#include "motorcontroller.h"


#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H


//nombre max de moteur sur un robot
#define NBMOTORMAX 4

///////////////////////////////////////////////////////////////////////////
enum class enumControllerState
{
  Config,
  FastTorque,
  FastAngle
};
///////////////////////////////////////////////////////////////////////////
//TODO: passer en private les attributs et méthodes et gérer set et get
class RobotController
{
public:
  uint8_t nbmotor;
  MotorController *motor;
  enumControllerState controllerState;
  unsigned char cptUARTFrame; //compteur de nombre d'échanges sur UART
  void getInfo();
  RobotController(); //uint8_t nbmotorinit); //finalement figé à NBMOTORMAX
  void task();
  void sendFastCANtask();
  void receiveFastCANtask();
  void shutdownMotorCANtask();
  bool receiveUARTTask();
  void sendUARTTask();
  // devra gérer les changements de mode enumControllerState
};
///////////////////////////////////////////////////////////////////////////
#endif