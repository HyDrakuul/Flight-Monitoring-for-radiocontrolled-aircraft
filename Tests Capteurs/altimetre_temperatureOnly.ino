#include <Wire.h>

#define MS5607_ADDRESS 0x76  // Adresse I2C du capteur

uint16_t C[7];  // Coefficients de calibration (C1 à C6)

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Lecture des coefficients de la PROM (C1 à C6)
  for (uint8_t i = 1; i <= 6; i++) {
    Wire.beginTransmission(MS5607_ADDRESS);
    Wire.write(0xA2 + (i - 1) * 2);  // Correction de l'adresse : 0xA2, 0xA4, ...
    Wire.endTransmission();

    Wire.requestFrom(MS5607_ADDRESS, 2);
    if (Wire.available() == 2) {
      C[i] = (Wire.read() << 8) | Wire.read();
    }
  }
}

void loop() {
  // --- Conversion température D2 ---
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x58);  // OSR = 4096 pour température
  Wire.endTransmission();
  delay(10);

  // Lecture de D2
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(MS5607_ADDRESS, 3);
  uint32_t D2 = 0;
  if (Wire.available() == 3) {
    D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  }

  // --- Calcul de température compensée avancée ---
  int32_t dT = D2 - ((uint16_t)C[5] << 8);  // dT = D2 - C5 * 256
  int32_t TEMP = 2000 + ((uint32_t)dT * C[6]) / 8388608;  // TEMP = 2000 + dT * C6 / 2^23

  // Compensation de température secondaire (corrige la dérive)
  int32_t T2 = (dT * dT) / 2147483648L; // Température seconde correction
  TEMP -= T2;

  // Affichage
  //Serial.print("Température compensée (°C) : ");// à commenter pour serial plotter
  

  Serial.println((float)TEMP / 100.0);  // Valeur en degrés Celsius
  
  

  delay(1000);
}