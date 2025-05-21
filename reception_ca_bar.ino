#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// Ecran LCD
LiquidCrystal lcd(12, 11, 6, 4, 3, 2);
SoftwareSerial Radio(5, 7);  // RX, TX

String buffer = "";
unsigned long previousMillis = 0;
int displayPage = 0;
String hm, te, lt, lg, at, v, st, h, dp, sdp_temp; // Ajout des variables SDP
bool newData = false;

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);
  Radio.begin(9600);
  Serial.println("Attente données...");
}

void loop() {
  // Réception non bloquante
  while (Radio.available()) {
    char c = Radio.read();
    if (c == '\n') {
      Serial.println("TRAME COMPLETE");
      parseData(buffer);
      
      // Affichage console données reçues
      Serial.println("====Données reçues du planeur====");
      Serial.print("Humidité: "); Serial.print(hm); Serial.println(" %");
      Serial.print("Temperature (C): "); Serial.println(te);
      Serial.print("Pression diff (Pa): "); Serial.println(dp);
      Serial.print("Temp SDP (C): "); Serial.println(sdp_temp);
      Serial.print("Latitude: "); Serial.println(lt);
      Serial.print("Longitude: "); Serial.println(lg);
      Serial.print("Altitude: "); Serial.print(at); Serial.println(" m");
      Serial.print("Vitesse: "); Serial.print(v); Serial.println(" km/h");
      Serial.print("Sattelites: "); Serial.println(st);
      Serial.print("Heure de réception (UTC): "); Serial.println(h);
      
      newData = true;
      buffer = "";  // reset après traitement
    } else {
      buffer += c;
    }
  }

  // Affichage cyclique sur LCD
  if (millis() - previousMillis > 3000) {
    previousMillis = millis();

    if (newData) {
      lcd.clear();  // Effacer uniquement si de nouvelles données ont été reçues
      newData = false;
    }

    // Permet de changer d'écran
    switch (displayPage) {
      case 0:
        lcd.setCursor(0, 0); lcd.print("HM:" + hm + "%");
        lcd.setCursor(0, 1); lcd.print("TEMP:" + te + "dC");
        break;
      case 1:
        lcd.setCursor(0, 0); lcd.print("LAT:" + lt + "dN");
        lcd.setCursor(0, 1); lcd.print("LNG:" + lg + "dE");
        break;
      case 2:
        lcd.setCursor(0, 0); lcd.print("ALT:" + at + "m");
        lcd.setCursor(0, 1); lcd.print("SPD:" + v + "km/h");
        break;
      case 3:
        lcd.setCursor(0, 0); lcd.print("SAT:" + st);
        lcd.setCursor(0, 1); lcd.print("TIME:" + h);
        break;
      case 4:  // Nouvel écran pour les données SDP
        lcd.setCursor(0, 0); lcd.print("DP:" + dp + "Pa");
        lcd.setCursor(0, 1); lcd.print("SDP T:" + sdp_temp + "dC");
        break;
    }

    displayPage = (displayPage + 1) % 5;  // Changé à 5 pour inclure le nouvel écran
  }
}

void parseData(String data) {
  int heureIndex = data.lastIndexOf("Heure     :");
  if (heureIndex == -1) return;

  int startTrame = data.lastIndexOf("Humidite:", heureIndex);
  if (startTrame == -1) return;

  String lastTrame = data.substring(startTrame);

  Serial.println(">>> TRAME EXTRAITE:");
  Serial.println(lastTrame);

  hm = extractValue(lastTrame, "Humidite: ");
  te = extractValue(lastTrame, "Temp (C): ");
  dp = extractValue(lastTrame, "Diff Pressure: ");       // Nouvelle extraction
  sdp_temp = extractValue(lastTrame, "SDP Temp: ");      // Nouvelle extraction
  lt = extractValue(lastTrame, "Lat: ");
  lg = extractValue(lastTrame, "Lng: ");
  at = extractValue(lastTrame, "Altitude  : ");
  v = extractValue(lastTrame, "Vitesse   : ");
  st = extractValue(lastTrame, "Sat: ");
  h = extractValue(lastTrame, "Heure     :");
}

String extractValue(String data, String label) {
  int start = data.indexOf(label);
  if (start == -1) return "";
  start += label.length();
  int end = data.indexOf(',', start);
  if (end == -1) end = data.length();
  String val = data.substring(start, end);
  val.trim();
  return val;
}