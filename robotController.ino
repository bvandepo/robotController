// B.Vandeportaele 2024
// TODO: tester plusieurs moteurs sur le même bus can et mesurer à l'oscillo delai de réponse de chaque moteur: 1.3ms
// vérifier comment le MCP2515 gère la réémission d'un message effacé du bus CAN
// ajouter compteur de requete/réponse pour vérifier si il y a des pertes
// quand tout fonctionnera piloter depuis l'arduino, définir un protocole UART pour dialoguer avec le PC, en
// gérant dans le protocole les différents moteurs dans un même message pour requette et réponse

// documentation du driver de moteur: https://www.myactuator.com/_files/ugd/cab28a_9d344907863b4033ada6d9318d511b6e.docx?dn=RMD-X%20Servo%20Motor%20Control%20Protocol%20V3.6.docx
////////////////////////////////
// activer le define suivant pour afficher les infos sur Serial
// #define DEBUG
////////////////////////////////

#include "robotcontroller.h"

RobotController robot;
////////////////////////////////////////////////////////////////////////////
void tache1()
{
}
////////////////////////////////////////////////////////////////////////////
void tache2()
{
}
////////////////////////////////////////////////////////
void tache3()
{
}
////////////////////////////////////////////////////////////////////////////
void setup()
{
  // pour config des differents matériels, utiliser une broche Analogique pontée à GND avec un fil dupont
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);

  while (!Serial)
    ;
  Serial.begin(1000000);

  SPI.begin(); // Begins SPI communication
  //  motor[0].setConf(4, 9, CAN_1000KBPS, MCP_16MHZ);//ID4 sur pin 9 carte seedstudio
  // moteur 4 foire pour la commande en position
  uint8_t identifiantCarte = 0;
  identifiantCarte |= digitalRead(A0);
  identifiantCarte |= digitalRead(A1) << 1;
  identifiantCarte |= digitalRead(A2) << 2;
  identifiantCarte |= digitalRead(A3) << 3;
  identifiantCarte |= digitalRead(A4) << 4;

  // REGLAGE BOURRIN à virer après dev:
  identifiantCarte = 0;

  switch (identifiantCarte)
  {
  case 0: // aucun cavalier
          // aucun moteur n'est présent: pour tester comm entre arduino et pc
    break;
  case 1:                                                  // cavalier sur A0
    robot.motor[0].setConf(5, 9, CAN_1000KBPS, MCP_16MHZ); // ID4 sur pin 9 carte seedstudio
    robot.motor[1].setConf(6, 10, CAN_1000KBPS, MCP_8MHZ); // ID6 sur pin 10 carte module aliexpress

    break;
  case 2: // cavalier sur A1
    break;
  case 4: // cavalier sur A2
    break;
  default:
    break;
  }
  // Serial.print("Setup done\n\n");
  // robot.getInfo();
}
////////////////////////////////////////////////////////////////////////////
void loop()
{
  /*

    const uint32_t periodiciteTache1 = 3; //ms
    static uint32_t timerTache1 = millis();
    if (millis() - timerTache1 >= periodiciteTache1) {
      timerTache1 += periodiciteTache1;
      tache1();
    }
    const uint32_t periodiciteTache2 = 0;  //ms
    static uint32_t timerTache2 = millis();
    if (millis() - timerTache2 >= periodiciteTache2) {
      timerTache2 += periodiciteTache2;
      tache2();
    }
    const uint32_t periodiciteTache3 = 3; //ms
    static uint32_t timerTache3 = millis();
    if (millis() - timerTache3 >= periodiciteTache3) {
      timerTache3 += periodiciteTache3;
      tache3();
    }
  */

  robot.task();
}

////////////////////////////////////////////////////////////////////////////
