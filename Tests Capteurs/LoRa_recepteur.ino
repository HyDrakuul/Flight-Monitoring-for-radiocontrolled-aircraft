#include <SoftwareSerial.h>
SoftwareSerial Radio(2, 3); // RX, TX

void setup() {
  Serial.begin(9600);   // Pour voir la réception
  Radio.begin(9600);    // Vers GAMMA62M

  Serial.println("En attente de message...");
}

void loop() {
  if (Radio.available()) {
    char c = Radio.read();
    Serial.print(c); // Affiche le message reçu
  }
}
