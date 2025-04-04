#include <SoftwareSerial.h>

SoftwareSerial SoftSerial(2, 3); // RX, TX
unsigned char buffer[64];        // Buffer de stockage
int count = 0;                   // Compteur de buffer

void setup() {
    Serial.begin(9600);    // Port série pour le PC
    SoftSerial.begin(9600); // Vérifie si ton module GPS est bien à 9600 bauds, sinon adapte
}

void loop() {
    // Lecture des données GPS
    if (SoftSerial.available()) {  
        count = 0; // Réinitialisation du compteur

        while (SoftSerial.available() && count < 64) {  
            buffer[count++] = SoftSerial.read();  
        }

        Serial.write(buffer, count);  // Envoi des données au PC
        clearBufferArray();           // Nettoyage du buffer
    }

    // Transmission PC vers GPS (pas forcément utile ici)
    if (Serial.available()) {  
        SoftSerial.write(Serial.read());  
    }

    delay(10); // Petit délai pour éviter une surcharge CPU
}

// Nettoyage du buffer
void clearBufferArray() {
    memset(buffer, 0, sizeof(buffer));  
}
