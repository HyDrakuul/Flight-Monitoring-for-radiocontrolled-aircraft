#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#define MS5607_ADDRESS 0x76  // Adresse I2C du capteur


SoftwareSerial SoftSerial(3, 2); // RX, TX
unsigned char buffer[64];        // Buffer de stockage
int count = 0;                   // Compteur de buffer

// Définitions pour la carte SD
const int chipSelect = 4;        // Broche CS pour la carte SD
File dataFile;                   // Fichier pour stocker les données


SoftwareSerial Radio(2, 3); // RX, TX

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
File myFile;

float time = 0;
bool enregistrementTermine = false;




uint16_t C[7];  // Coefficients de calibration (C1 à C6)

int sensorPin = 6;              // 220 resistance pour le capteur d'humidité
long result = 0;
void setup()                    // run once, when the sketch starts
{
  Serial.begin(9600);
  Wire.begin();
  Radio.begin(9600);     // Vers GAMMA62M
  delay(2000); // Laisse le module démarrer
  SoftSerial.begin(9600);      // Port série pour le GPS 

  // Initialisation de la carte SD
  Serial.print("Initialisation de la carte SD..."); 
  if (!SD.begin(chipSelect)) {
      Serial.println("Erreur d'initialisation de la carte SD!");
      return;
    }
  Serial.println("Carte SD initialisée.");
    
  // Création ou ouverture du fichier de données
  dataFile = SD.open("gpsdata.txt", FILE_WRITE);
  if (dataFile) {
    Serial.println("Fichier gpsdata.txt ouvert ou créé.");
    dataFile.println("=== Début des données GPS ===");
    dataFile.close();
    } else {
      Serial.println("Erreur lors de l'ouverture de gpsdata.txt!");
    }
  while (!Serial); // attendre le port série
  Serial.print("Initialisation carte SD...");
  if (!SD.begin(4)) {
    Serial.println("Échec !");
    while (1);
  }
  Serial.println("Carte SD OK.");
  if (!lis.begin(0x19)) {
    Serial.println("Capteur LIS3DH non detecte !");
    while (1);
  }
  Serial.println("Capteur detecte.");
  lis.setRange(LIS3DH_RANGE_4_G);

  // Création nom de fichier (acc1.txt, acc2.txt, etc.)
  int index = 1;
  String nomFichier;
  do {
    nomFichier = "acc" + String(index) + ".txt";
    index++;
  } while (SD.exists(nomFichier));
  myFile = SD.open(nomFichier.c_str(), FILE_WRITE);
  if (!myFile) {
    Serial.println("Erreur ouverture fichier !");
    while (1);
  }
  Serial.print("Enregistrement dans : ");
  Serial.println(nomFichier);
  Serial.println("Temps (s), X (m/s2), Y (m/s2), Z (m/s2)");
  myFile.println("Temps (s), X (m/s2), Y (m/s2), Z (m/s2)");

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
  Serial.println("Envoi vers GAMMA62M...");
  Radio.println("Hello depuis Arduino 1 !");
  Serial.println("start");      // a personal quirk 
}
void loop()                   
{
  Radio.println("----------------------------------------------------------"); 
  Serial.println(RCtime(sensorPin));
  Radio.print("Taux d'humidité:");
  Radio.print( 1-((((RCtime(sensorPin))*pow(10,-6)/1e6)-163.33e-12)/3.33e-11));
  Radio.println("%");
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
  Radio.print("Température compensée (°C) : ");// à commenter pour serial plotter
  Radio.println((float)TEMP / 100.0);  // Valeur en degrés Celsius
  if (!enregistrementTermine && myFile) {
    sensors_event_t event;
    lis.getEvent(&event);
    time += 0.0075;

    // Affichage série
    Radio.print("Coordonnées d'acc");
    Radio.print(time, 4);
    Radio.print(", ");
    Radio.print(event.acceleration.x);
    Radio.print(", ");
    Radio.print(event.acceleration.y);
    Radio.print(", ");
    Radio.println(event.acceleration.z);

    // Écriture fichier
    myFile.print(time, 4);
    myFile.print(", ");
    myFile.print(event.acceleration.x);
    myFile.print(", ");
    myFile.print(event.acceleration.y);
    myFile.print(", ");
    myFile.println(event.acceleration.z);
    myFile.flush();

    if (time >= 10.0) {
      myFile.close();
      Radio.println(">>> Enregistrement terminé et fichier fermé !");
      enregistrementTermine = true;
    }
    delay(1);
  }
   // Lecture des données GPS
    if (SoftSerial.available()) {  
        count = 0; // Réinitialisation du compteur

        while (SoftSerial.available() && count < 64) {  
            buffer[count++] = SoftSerial.read();  
        }

    // Envoi des données au PC
    Serial.write(buffer, count);
        
    // Enregistrement des données sur la carte SD
    dataFile = SD.open("gpsdata.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.write(buffer, count);
      dataFile.close();
    } else {
      Serial.println("Erreur d'écriture sur la carte SD!");
    }
        
    clearBufferArray(); // Nettoyage du buffer
    }

    // Transmission PC vers GPS (optionnel)
    if (Serial.available()) {  
      SoftSerial.write(Serial.read());  
    }

  delay(10); // Petit délai pour éviter une surcharge CPU

  Radio.println("----------------------------------------------------------"); 
  delay(1000);
}

long RCtime(int sensPin){
   long result = 0;
   pinMode(sensPin, OUTPUT);       // make pin OUTPUT
   digitalWrite(sensPin, HIGH);    // make pin HIGH to discharge capacitor - study the schematic
   delay(1);                       // wait a  ms to make sure cap is discharged
   unsigned long StartTime =micros();
   pinMode(sensPin, INPUT);        // turn pin into an input and time till pin goes low
   digitalWrite(sensPin, LOW);     // turn pullups off - or it won't work
   while(digitalRead(sensPin)){    // wait for pin to go low
      if(micros()-StartTime > 300000) break;
   }

   return micros()-StartTime;                   // report results   
}
// Nettoyage du buffer
void clearBufferArray() {
    memset(buffer, 0, sizeof(buffer));  
}
