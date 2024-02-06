// B.Vandeportaele 2024
// quand ce programme recoit 28 caractères identiques à la suite, il renvoie 28 fois le même caractère
#define NBCAR 28
////////////////////////////////
void setup()
{
  while (!Serial)
    ;
  Serial.begin(1000000);
  Serial.println("PONG");
}
////////////////////////////////////////////////////////////////////////////
void loop()
{
  static uint8_t lastcar = 0;
  static uint8_t cpt_lastcar = 0;

  if (Serial.available()) {
    uint8_t car = Serial.read();
    if (car != lastcar) {
      lastcar = car;
      cpt_lastcar = 1;
    }
    else
      cpt_lastcar++;
    if (cpt_lastcar >= NBCAR)
      for (uint8_t n = 0; n < NBCAR; n++)
        Serial.write(lastcar);

  }

}

////////////////////////////////////////////////////////////////////////////
