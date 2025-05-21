#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#define MS5607_ADDRESS 0x76
#define SENSOR_PIN 6
#define CHIP_SELECT 53
#define GPS_BUFFER_SIZE 100
unsigned long previousMillis = 0;
const long interval = 2000;

char gpsBuffer[GPS_BUFFER_SIZE];
int gpsBufferIndex = 0;
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
TinyGPSPlus gps;
unsigned char buffer[64];
int count = 0;
uint16_t C[7];
unsigned long startime=millis();
float time;
char nfgps[] = "ngptest1.txt";
char ddv[] = "ddv1.txt"; // 8 caractères max pour le nom
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
  delay(2000);
  File f;
  Serial.print(F("Initialisation de la carte SD..."));
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println(F("Erreur d'initialisation!"));
    return;
  }
  Serial.println(F("Carte SD OK."));

  f = SD.open(nfgps, FILE_WRITE);
  if (f) {
    Serial.println("Création/ouverture fichier gps réussie!");
    f.println(F("=== Debut des donnees GPS (trames brutes) ==="));
    f.close();
  } else {
    Serial.println(F("Erreur ouverture fichier gps !"));
  }
  f = SD.open(ddv, FILE_WRITE);
  if (f) {
    Serial.println("Création/ouverture fichier de données de vol réussie!");
    f.println(F("=== Debut des donnees de vol==="));
    f.close();
  } else {
    Serial.println(F("Erreur ouverture fichier de données de vol !"));
  }
  if (!lis.begin(0x19)) {
    Serial.println(F("LIS3DH non detecte!"));
    while (1);
  }
  lis.setRange(LIS3DH_RANGE_4_G);
  Serial.println("Acceleromètre OK.");


  for (uint8_t i = 1; i <= 6; i++) {
    Wire.beginTransmission(MS5607_ADDRESS);
    Wire.write(0xA2 + (i - 1) * 2);
    Wire.endTransmission();
    Wire.requestFrom(MS5607_ADDRESS, 2);
    if (Wire.available() == 2) {
      C[i] = (Wire.read() << 8) | Wire.read();
    }
  }
  Serial2.println(F("Hello depuis Arduino!"));
  Serial.println(F("start"));
}

void loop() {
  
  //Serial2.println(F("--------------------------"));
  //Serial.println(F("--------------------------"));

  float humidity = ((((RCtime(SENSOR_PIN)) *pow(10,-6)/1e6) - 163.33e-12) / 3.33e-11);
 
  Serial.print(F("Humidite: ")); Serial.print(humidity);
  Serial.println(" %");



  // Température
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x58);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MS5607_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(MS5607_ADDRESS, 3);
  uint32_t D2 = 0;
  if (Wire.available() == 3) {
    D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  }
  int32_t dT = D2 - ((uint16_t)C[5] << 8);
  int32_t TEMP = 2000 + ((uint32_t)dT * C[6]) / 8388608;
  int32_t T2 = (dT * dT) / 2147483648L;
  TEMP -= T2;

  float temperature = TEMP / 100.0;
  
  Serial.print(F("Temp (C): ")); Serial.println(temperature);

  // Accéléromètre

  sensors_event_t event;
  lis.getEvent(&event);
  time = (millis()-startime)/1000;
  Serial.print("Temps écoulé depuis le lancement du programme: ");
  Serial.print(time, 4); 
  Serial.println(F(" s"));
  Serial.println("===Accéléromètre===");
  Serial.print("X:");
  Serial.print(event.acceleration.x); 
  Serial.print(" m/s^2");
  Serial.print(F(", "));
  Serial.print("Y:");
  Serial.print(event.acceleration.y); 
  Serial.print(" m/s^2");
  Serial.print(F(", "));
  Serial.print("Z:");
  Serial.print(event.acceleration.z);
  Serial.println(" m/s^2");
  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;

  Serial.print("Accélération totale: ");Serial.print(sqrt(ax*ax +ay*ay +az*az));
  Serial.println(" m/s^2");

  

  // GPS

   while (Serial1.available()) {
    char c = Serial1.read();
    gps.encode(c);

    // Stocke le caractère dans le buffer si pas plein
    if (gpsBufferIndex < GPS_BUFFER_SIZE - 1) {
      gpsBuffer[gpsBufferIndex++] = c;
    }

    // Fin de trame détectée (fin de ligne)
    if (c == '\n') {
      gpsBuffer[gpsBufferIndex] = '\0'; // termine la chaîne

      // Écrit la trame complète dans le fichier SD
      File dataFile = SD.open(nfgps, FILE_WRITE);
      if (dataFile) {
        dataFile.print(gpsBuffer);
        dataFile.close();
        Serial.println("Trame NMEA enregistrée !");
      } else {
        Serial.println(F("Erreur ouverture fichier GPS"));
      }

      gpsBufferIndex = 0; // réinitialise le buffer
    }
  }
    //formate sous forme de trames pour faciliter le traitement.
    String trame = "";
            
    trame += "Humidite:" + String(humidity, 2) + ",";
    trame += "Temp (C):" + String(temperature, 2) + ",";
    trame += "Time:"+String(time, 2) + ",";
    trame += "X:"+String(event.acceleration.x, 2) + ",";
    trame += "Y:"+String(event.acceleration.y, 2) + ",";
    trame += "Z:"+String(event.acceleration.z, 2) + ",";

    if (gps.location.isUpdated() && gps.time.isValid()) {
      trame += "Lat:" + String(gps.location.lat(), 6) + ",";
      trame += "Lng:" + String(gps.location.lng(), 6) + ",";
      trame += "Altitude:" + String(gps.altitude.meters(), 2) + ",";
      trame += "Vitesse:" + String(gps.speed.kmph(), 2) + ",";
      trame += "Sat:" + String(gps.satellites.value()) + ",";
      trame += "Fix: OK,";
      trame += "Heure:" + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      trame +="\n";
      Serial2.println(trame);
      Serial.println("Trame envoyée:");
      Serial.println(trame);
    } else {
      Serial.println("Données GPS non valides. Trame non envoyée.");
    }

    
  
  File myFile=SD.open(ddv, FILE_WRITE);
  if (myFile) {
    myFile.println(" ");
    myFile.print("Temps écoulé depuis le démarrage du programme:");
    myFile.print(time, 4);myFile.println(" secondes");
    myFile.println("---Température et Humidité---");
    myFile.print("Température (C): "); myFile.print(temperature, 2);
    myFile.print(";");
    myFile.print("Taux d'humidité:");myFile.print(humidity, 2);
    myFile.println(" %");
    myFile.println("---Accéléromètre---");
    myFile.print("Selon x: ");myFile.print(event.acceleration.x);
    myFile.print("m/(s)^2; ");
    myFile.print("Selon y: ");myFile.print(event.acceleration.y);
    myFile.print("m/(s)^2; ");
    myFile.print("Selon z: ");myFile.print(event.acceleration.z);
    myFile.print("m/(s)^2; ");
    myFile.print("Accélération totale: ");myFile.print(sqrt(ax*ax +ay*ay +az*az));
    myFile.println(" m/(s)^2");
    myFile.close();
    Serial.println("Données de vol enregistrées !");
  } else {
    Serial.println("Erreur ouverture fichier données de vol.");
  }
      
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    
  }
  
  
  
  
}

unsigned long RCtime(uint8_t sensPin) {
  pinMode(sensPin, OUTPUT);
  digitalWrite(sensPin, HIGH);
  delay(1);
  unsigned long startTime = micros();
  pinMode(sensPin, INPUT);
  digitalWrite(sensPin, LOW);
  while (digitalRead(sensPin)) {
    if (micros() - startTime > 300000) break;
  }
  return micros() - startTime;
}
