#include <EEPROM.h>
#include <ChainableLED.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <TinyGPS++.h>
#define BME280_ADDRESS 0x76

TinyGPSPlus gps;
const int chipSelect = 4;  // Pin pour CS de la carte SD
RTC_DS1307 rtc;            // Création d'une instance pour le RTC
char fileName[13];
int registre=1;
Adafruit_BME280 bme;
unsigned long lastActivityTime = 0;  // Stocke le dernier moment d'activité
int i_etat_systeme;
int i_etat_BtR=1;
int i_etat_BtG=1;
unsigned long currentTimeBt=0;
const int clkPin = 7; // Pin de l'horloge (CLK)
const int dataPin = 8; // Pin des données (DATA)
ChainableLED leds(clkPin, dataPin, 1);  // '1' pour 1 LED connectée
const int redButtonPin = 3;  // Broche du bouton rouge
const int greenButtonPin = 2; // Broche du bouton bleu
const int lightSensorPin = A0;
bool etat_LED=0;
bool acquisGPS=0;
unsigned long previousMillis = millis();
unsigned long currentMillis = millis();
unsigned long previousMillisLED = 0;
bool b_erreur_RTC;
bool b_erreur_GPS;
bool b_erreur_capteur;
bool b_erreur_incorrect_capteur;

uint8_t LOG_INTERVAL = 10;
uint16_t  FILE_MAX_SIZE =4096;
uint8_t  TIMEOUT=30;
bool LUMIN=1;
uint16_t  LUMIN_LOW=255;
uint16_t LUMIN_HIGH=868;
bool TEMP_AIR=1;
int8_t  MIN_TEMP_AIR=-10;
int8_t  MAX_TEMP_AIR=60;
bool HYGR=1;
int8_t HYGR_MINT=0;
int8_t HYGR_MAXT=50;
bool PRESSURE=1;
uint16_t  PRESSURE_MIN=850;
uint16_t  PRESSURE_MAX=1080;
String VERSION = "1.0.0";
int etat_erreur=0;

void initialisationVariable(){
  Serial.println("initialisation variable");
  EEPROM.put(0, LOG_INTERVAL);
  EEPROM.put(1, FILE_MAX_SIZE);
  EEPROM.put(3, TIMEOUT);
  EEPROM.put(4, LUMIN);
  EEPROM.put(5, LUMIN_LOW);
  EEPROM.put(7, LUMIN_HIGH);
  EEPROM.put(9, TEMP_AIR);
  EEPROM.put(10, MIN_TEMP_AIR);
  EEPROM.put(11, MAX_TEMP_AIR);
  EEPROM.put(12, HYGR);
  EEPROM.put(13, HYGR_MINT);
  EEPROM.put(14, HYGR_MAXT);
  EEPROM.put(15, PRESSURE);
  EEPROM.put(16, PRESSURE_MIN);
  EEPROM.put(18, PRESSURE_MAX);
  if (!SD.begin(chipSelect)) {
    Serial.println("Erreur d'initialisation de la carte SD !");
    return;
  }
  Serial.println("Carte SD initialisée.");
  if (!rtc.begin()) {
    
    Serial.println("Erreur acces horloge RTC");
    return;
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
}

void setup() {
  Serial.begin(9600);
  initialisationVariable();
  Serial.println("Démarrage du système");
  pinMode(redButtonPin, INPUT); // Initialisation bouton
  pinMode(greenButtonPin, INPUT); // Initialisation bouton
  i_etat_systeme=0;
  if (!digitalRead(redButtonPin)){
    i_etat_systeme=1;
  }
  attachInterrupt(digitalPinToInterrupt(redButtonPin), startPressR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(greenButtonPin), startPressG, CHANGE);
}

void loop() {
  erreur();
  switch (i_etat_systeme){
  case 0:
    modeStandard();
    break;
  case 1:
    modeConfiguration();
    break;
  case 2:
    modeEconomie();
    break;
  case 3:
    modeMaintenance();
    break;
  default:
    Serial.println("ErreurEtatSysteme");
    break;
  }
}

void erreurCapteur(bool b_erreur_capteur){
  unsigned long currentMillisLED = millis();
  if (b_erreur_capteur){
    if (currentMillisLED-previousMillisLED>500){
      if (currentMillisLED-previousMillisLED>1000){
        leds.setColorRGB(0, 0, 255, 0);
        previousMillisLED=currentMillisLED;
      }
      else{
        leds.setColorRGB(0, 255, 0, 0);
      }
    }
  }
}

void erreurGPS(bool b_erreur_GPS){
  unsigned long currentMillisLED = millis();
  if (b_erreur_GPS){
    if (currentMillisLED-previousMillisLED>500){
      if (currentMillisLED-previousMillisLED>1000){
        leds.setColorRGB(0, 255, 255, 0);
        previousMillisLED=currentMillisLED;
      }
      else{
        leds.setColorRGB(0, 255, 0, 0);
      }
    }
  }
}

void erreurIncorrectCpatuer(bool b_erreur_incorrect_capteur){
  unsigned long currentMillisLED = millis();
  if (b_erreur_incorrect_capteur){
    if (currentMillisLED-previousMillisLED>333){
      if (currentMillisLED-previousMillisLED>1000){
        leds.setColorRGB(0, 255, 0, 0);
        previousMillisLED=currentMillisLED;
      }
      else{
        leds.setColorRGB(0, 0, 255, 0);
      }
    }
  }
}
void erreur(){
  if (!b_erreur_capteur && !b_erreur_GPS && !b_erreur_incorrect_capteur){
    systemeLed(i_etat_systeme);
  }
  else{
    erreurCapteur(b_erreur_capteur);
    erreurGPS(b_erreur_GPS);
    erreurIncorrectCpatuer(b_erreur_incorrect_capteur);
  }
}

void systemeLed(int i_etat_systeme){
  switch (i_etat_systeme){
  case 0:
    leds.setColorRGB(0, 0, 255, 0);
    break;
  case 1:
    leds.setColorRGB(0, 255, 255, 0);
    break;
  case 2:
    leds.setColorRGB(0, 0, 0, 255);
    break;
  case 3:
    leds.setColorRGB(0, 255, 50, 0);
    break;
  }
}

void modeStandard(){
  uint8_t interval;
  EEPROM.get(0,interval);
  currentMillis=millis();
  if (currentMillis - previousMillis >= interval*1000){
    previousMillis = currentMillis;
    float* mesure = (recupereMesure());
    char* heureActuelle = obtenirHeure();
    float* coordonnees = obtenirCoordonneesGPS();
    //sauvegarde(mesure,heureActuelle, coordonnees);
  }
}

unsigned int hash(String str) {
    unsigned int hash = 0;
    for (int i = 0; i < str.length(); i++) {
        hash = 31 * hash + str[i];
    }
    return hash;
}

void modeConfiguration(){
  Serial.println("---Configuration---");
  lastActivityTime=millis();
  while (millis()-lastActivityTime<30000){
    if (Serial.available()>0){
      String input = Serial.readString();
      int equalIndex = input.indexOf('=');
      input.trim();
      if  (equalIndex > 0){
        String variableName = input.substring(0, equalIndex);
        String valueString = input.substring(equalIndex + 1);
        int value = valueString.toInt();
        switch (hash(variableName)){
          case 42880:
          EEPROM.update(0,value);
          break;
          case 16767:
          EEPROM.update(1,value);
          break;
          case 55617:
          EEPROM.update(3,value);
          break;
          case 51209:
          EEPROM.update(4,value);
          break;
          case 61950:
          EEPROM.update(5,value);
          break;
          case 25624:
          EEPROM.update(7,value);
          break;
          case 24127:
          EEPROM.update(9,value);
          break;
          case 48556:
          EEPROM.update(10,value);
          break;
          case 37594:
          EEPROM.update(11,value);
          break;
          case 4540:
          EEPROM.update(12,value);
          break;
          case 30981:
          EEPROM.update(13,value);
          break;
          case 23603:
          EEPROM.update(14,value);
          break;
          case 1605:
          EEPROM.update(15,value);
          break;
          case 46392:
          EEPROM.update(16,value);
          break;
          case 46154:
          EEPROM.update(18,value);
          break;
        }
        Serial.println(variableName);
        Serial.println(value);
      }
      if (input.equals("VERSION")){
        Serial.println(VERSION);
      }
      if (input==("RESET")){
        initialisationVariable();
      }
    lastActivityTime=millis();
    } 
  }
  i_etat_systeme=0;  
}


void modeEconomie(){
  //Serial.println("---Economie---");
  uint8_t interval;
  EEPROM.get(0,interval);
  currentMillis=millis();
  if (currentMillis - previousMillis >= interval*1000*2){
    float* a = (recupereMesure());
    char* heureActuelle = obtenirHeure();
    Serial.println(heureActuelle);
    previousMillis = currentMillis;
    if (acquisGPS==0){
      float* coordonnees = obtenirCoordonneesGPS();
      acquisGPS=1;}
    else{
      acquisGPS=0;
    }
    //sauvegarde(a,heureActuelle, coordonnees);
  }
}

void modeMaintenance(){
  //Serial.println("---Maintenance---");
  currentMillis=millis();
  if (currentMillis - previousMillis >= 1000){
    previousMillis = currentMillis;
    float* a = (recupereMesure());
    char* heureActuelle = obtenirHeure();
    Serial.println(heureActuelle);
    float* coordonnees = obtenirCoordonneesGPS();
    Serial.println(a[0]);
    Serial.println(a[1]);
    Serial.println(a[2]);
    Serial.println(a[3]);
  }
}



float* obtenirCoordonneesGPS() {
  static float coordonnees[2]={100.0,100.0}; // Tableau statique pour stocker latitude et longitude
  if (Serial.available() > 0) {
    b_erreur_GPS=0;
    gps.encode(Serial.read());
    if (gps.location.isUpdated()) { // Vérifie si les données GPS sont mises à jour
      coordonnees[0] = gps.location.lat();  // Latitude
      coordonnees[1] = gps.location.lng();  // Longitude
    }
    else{
      Serial.println("Data GPS nondisponible");
    }
  }
  else{
    b_erreur_GPS=1;
    Serial.println("Erreur GPS");
  }
  return coordonnees;
}

char* obtenirHeure() {
  static char heure[9]; // Format HH:MM:SS + '\0'
  DateTime now = rtc.now(); // Obtenir la date et l'heure actuelles
  snprintf(heure, sizeof(heure), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  return heure; // Retourner l'heure formatée
}

float* recupereMesure(){
  static float tableauMesure[4];
  if (bme.begin(BME280_ADDRESS)){
    b_erreur_capteur=0;
    b_erreur_incorrect_capteur=0;
    tableauMesure[0]=mesureTemperature();
    tableauMesure[1]=0;
    int8_t borne;
    EEPROM.get(13,borne);
    if (borne<tableauMesure[0]){
      EEPROM.get(14,borne);
      if (tableauMesure[0]<borne){
        tableauMesure[1]=mesureHygrometrie();
      }
    }  
    tableauMesure[2]=mesurePression();
    tableauMesure[3]=mesureLuminosite();
    return(tableauMesure);
  }
  b_erreur_capteur=1;
  Serial.println("Erreur acces cpateurs");
}

float mesureTemperature(){
  bool valeur;
  if (EEPROM.get(9,valeur)){
    float f_temperature = bme.readTemperature();
    int8_t borne;
    EEPROM.get(10,borne);
    if (borne<f_temperature){
      EEPROM.get(11,borne);
      if (f_temperature<borne){
        Serial.println("Temp ok");
        return(f_temperature);
      }
    } 
    b_erreur_incorrect_capteur=1;
    Serial.println("Temp HB");
    etat_erreur=4;
    return(0);
  }
  Serial.println("Capteur off");
  return(0);
}

float mesureHygrometrie(){
  bool valeur;
  if (EEPROM.get(12,valeur)){
    float f_hygrometrie = bme.readHumidity();
    Serial.println("Hygr ok");
    return(f_hygrometrie);
  }
  Serial.println("Capteur off");
  return(0);
}

float mesurePression(){
  bool valeur;
  if (EEPROM.get(15,valeur)){
    float f_pression = bme.readPressure() / 100.0F;
    uint16_t borne;
    EEPROM.get(16,borne);
    if (borne<f_pression){
      EEPROM.get(18,borne);
      if (f_pression<borne){
        Serial.println("Press ok");
        return(f_pression);
      }
    } 
    b_erreur_incorrect_capteur=1;
    Serial.println("Press HB");
    etat_erreur=4;
    return(0);
  }
  Serial.println("Capteur off");
  return(0);
}

float mesureLuminosite(){
  bool valeur;
  if (EEPROM.get(4,valeur)){
    float f_luminosite = analogRead(lightSensorPin);
    uint16_t borne;
    EEPROM.get(5,borne);
    if (borne<f_luminosite){
      EEPROM.get(7,borne);
      if (f_luminosite<borne){
        Serial.println("Lum ok");
        return(f_luminosite);
      }
    }
    b_erreur_incorrect_capteur=1;
    Serial.println("Lum HB");
    return(0);
  }
  Serial.println("Capteur off");
  return(0);
}

void startPressR() {
  if (digitalRead(redButtonPin)==0 && digitalRead(greenButtonPin)==1){
    currentTimeBt = millis();
    i_etat_BtR=0;
    }
  if (digitalRead(redButtonPin)==1 && (millis()-currentTimeBt)>3000 && i_etat_BtR==0){
    i_etat_BtR=1;
    if (i_etat_systeme==0){
      i_etat_systeme=3;
    }
    else{
      if (i_etat_systeme==2 || i_etat_systeme==3){
        i_etat_systeme=0;
      }
    }
  } 
}

void startPressG() {
  if (digitalRead(greenButtonPin)==0 && digitalRead(redButtonPin)==1){
    currentTimeBt = millis();
    i_etat_BtG=0;
    }
  if (digitalRead(greenButtonPin)==1 && (millis()-currentTimeBt)>3000 && i_etat_BtG==0){
    i_etat_BtG=1;
      if (i_etat_systeme==0){
      i_etat_systeme=2;
    }
  }  
}