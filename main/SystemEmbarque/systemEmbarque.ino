#include <ChainableLED.h>
#define NUM_LEDS  5
ChainableLED leds(6, 5, NUM_LEDS);

#include "configuration"

#include "RTClib.h"

// Initalisation des librairies
RTC_DS1307 rtc;
char daysOfTheWeek[7][4]={"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
#include <SPI.h>
#include <SD.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial SoftSerial(7, 8);

BME280I2C bme;
float Rsensor; //Resistance of sensor in K
File myFile;

int boutonrouge = 0;
int boutonvert = 0;
int LOG_INTERVAL = 10;
int inversion = 1;
String ModeActuel = "";
String ModePrecedent = "";
unsigned long time_start,time_stop;


struct capteur{
  
    int LUMIN;
    int LUMIN_LOW; 
    int LUMIN_HIGH; 

    int TEMP_AIR;
    int MIN_TEMP_AIR; 
    int MAX_TEMP_AIR;

    int HYGR;
    int HYGR_MINT; 
    int HYGR_MAXT;

    int PRESSURE;
    int PRESSURE_MIN; 
    int PRESSURE_MAX;

    int LOG_INTERVALL;
    int FILE_MAX_SIZE;
    String VERSION;
    
};

capteur cpt;

////////////////////
//   Void Setup   //
////////////////////
void setup() {
  SoftSerial.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  delay(2000);

 
  initialisation_capteur(&cpt);

  if (!bme.begin()) {
    Serial.println(F("Le capteur de température et de pression n'est pas branché correctement."));
    while (1);
  }

  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println(F("Found BMP280 sensor! No Humidity available."));
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }

  /////////////////////////////////
  //Initialisation de la carte SD//
  /////////////////////////////////
  while (!Serial);
  //  Serial.print(F("Initialisaation de la carte SD..."));
  if (!SD.begin(4)) {
    Serial.println(F("Erreur d'initialisation!"));
    while (1);
  }
  Serial.println(F("Initialisation terminée !"));

  if (SD.exists("data.txt")) {
    Serial.println(F("Removing data.txt..."));
    SD.remove("data.txt");
  }


  leds.init();

  attachInterrupt(digitalPinToInterrupt(2),ChangeModeRouge,CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),ChangeModeVert,CHANGE);
  
  pinMode(2, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(3, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  

  boutonrouge = digitalRead(2);
  boutonvert = digitalRead(3);
  
  if(ModeActuel == ""){
      if(boutonrouge == LOW){
          ModeActuel = "configuration";
          leds.setColorRGB(0, 255, 255, 0);
          Configuration();   
      }else{
          ModeActuel = "standard";
          leds.setColorRGB(0,0,255,0);  
      }
    }


  if (!rtc.begin()) {
    //Erreur(1);
  }
  else{
    // Mise à jour de l'horloge
    //rtc.adjust(DateTime(2022,10,19,5,53,20));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
 
 }
}


void loop() {
  if(ModeActuel=="standard"){ModeStandard(&cpt,0);}
  else if(ModeActuel=="maintenance"){Serial.println("maintenance");ModeStandard(&cpt,2);}
  else if(ModeActuel=="economique"){Serial.println("eco");ModeStandard(&cpt,1);}
  delay(1000);
}


//Fonction du mode Standard
void ModeStandard(capteur* cpt,int typeMode) {
 
  int sensorValue = analogRead(0);
  Rsensor = (float)(1023 - sensorValue) * 10 / sensorValue;

  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  if(typeMode == 2){
  Serial.print(F("\nSauvegarde des données du capteur BME280 : "));
  Serial.print(F("Temperature : "));
  Serial.print(temp);
  Serial.print(F("°C / "));
  Serial.print(F("Pression : "));
  Serial.print(pres);
  Serial.print(F("Pa / "));
  Serial.print(F("Humidité : "));
  Serial.print(hum);
  Serial.println(F("%"));
  Serial.print(F("Sauvegarde des données du capteur Light Grove Sensor : "));
  Serial.print(F("Intensité lumineuse : "));
  Serial.print(Rsensor, DEC);
  Serial.println(F("J"));
  


  String lapositiondugps = DataGPS();
  Serial.print(F("Sauvegarde des données du capteur Grove GPS : "));
  Serial.println(lapositiondugps);
  }else{
  if(!SD.begin(4)){Erreur(6);}else{
  String lapositiondugps = DataGPS();
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    Serial.print(F("Ecriture sur : data.txt..."));
    //myFile.println(Data);
    if(cpt->TEMP_AIR == 1){
      if(temp >= cpt->MIN_TEMP_AIR and temp <= cpt->MAX_TEMP_AIR){
    myFile.print("Temperature : ");
    myFile.print(temp);
    myFile.print("°C / ");
      }else{
      myFile.print("Temperature : ");
      myFile.print("NA");
      myFile.println(" / ");
      /*Erreur(4);*/}
    }
    if(cpt->PRESSURE == 1){
      if(temp >= cpt->PRESSURE_MIN and temp <= cpt->PRESSURE_MAX){
    myFile.print("Pression : ");
    myFile.print(pres);
    myFile.print("hPa / ");
      }else{
      myFile.print("Pression : ");
      myFile.print("NA");
      myFile.print(" / ");
      /*Erreur(4);*/}
    }
    //    myFile.print("Altitude : ");
    //    myFile.print(Altitude);
    //    myFile.print("m / ");
    if(cpt->HYGR == 1){
      if(temp >= cpt->HYGR_MINT and temp <= cpt->HYGR_MAXT){
    myFile.print("Humidité : ");
    myFile.print(hum);
    myFile.print("% / ");
      }
    }

    if (typeMode == 0 or inversion == 1 ){
    myFile.print("GPS : ");
    myFile.print(lapositiondugps);
    inversion = inversion * -1;}
    else if(typeMode == 1 and inversion == -1){
      inversion = inversion * -1;}
    
    if(cpt->LUMIN == 1){
      if(temp >= cpt->LUMIN_LOW and temp <= cpt->LUMIN_HIGH){
    myFile.print("Intensité lumineuse : ");
    myFile.print(Rsensor, DEC);
    myFile.print("J / ");
      }else{
      myFile.print("Intensité lumineuse : ");
      myFile.print("NA");
      myFile.println(" / ");
      /*Erreur(4);*/}
    }
    myFile.close();
    Serial.println(F("Terminer."));
    if(typeMode == 1){delay((cpt->LOG_INTERVALL)*2);}
    else{delay(cpt->LOG_INTERVALL);}
    
  } else {
    Serial.println(F("Erreur d'ouverture de filesavedata.txt\n"));
  }
}}}


String DataGPS() {
  String gpsData;
  bool t;
  gpsData = "";
  if (SoftSerial.available()) {
    t = true;
    while (t) {
      gpsData = SoftSerial.readStringUntil('\n');
      if (gpsData.startsWith("$GPGGA", 0)) {
        t = false;
      }
    }
  }
  return gpsData;
}

void ChangeModeRouge(){                                                                                         //BOUTON BLANC
  boutonrouge = digitalRead(2);

   
  if(boutonrouge == LOW){
    time_start = millis();
  }
  if(boutonrouge == HIGH){
    time_stop = millis();                                                                                                 
    if(ModeActuel == "standard" and time_stop-time_start > 2000){

      leds.setColorRGB(0, 255, 200, 0);
      ModeActuel = "maintenance";
      ModePrecedent = "standard";
      
      }else if(ModeActuel == "economique" and time_stop-time_start > 2000){
        
      
        leds.setColorRGB(0, 255, 200, 0);
        ModeActuel = "maintenance";
        ModePrecedent = "economique";
        
    }else if(ModeActuel == "maintenance" and ModePrecedent == "standard" and time_stop-time_start > 2000){

          ModeActuel = "standard";
          ModePrecedent = "maintenance";
          leds.setColorRGB(0, 0, 255, 0);

    }else if(ModeActuel == "maintenance" and ModePrecedent == "economique" and time_stop-time_start > 2000){
 
          ModeActuel = "economique";
          ModePrecedent = "maintenance";
          leds.setColorRGB(0, 0, 0, 255);
    }
  }
}

void ChangeModeVert(){                                                                                                    //BOUTON BLEU
  
    boutonvert = digitalRead(3);

  
  if(boutonvert == LOW){
    time_start = millis();
  }
  if(boutonvert == HIGH){
    time_stop = millis();                                                                                                   
    if (ModeActuel == "standard" and time_stop-time_start > 2000){
      
          ModeActuel = "economique";
          ModePrecedent = "standard";
          leds.setColorRGB(0, 0, 0, 255);
      
    }else if(ModeActuel == "economique" and time_stop-time_start > 2000){

          ModeActuel = "standard";
          ModePrecedent = "economique";
          leds.setColorRGB(0, 0, 255, 0);

    }
  }
}


void Configuration(){
/*
String commande = "";
int valeur;
int Attente=0;

  delay(1000);
while(Attente <= 20 and Serial.available() <= 0){Serial.println(Attente);Attente++;delay(1000);}
  if(Attente >= 20){ModeActuel = "standard";leds.setColorRGB(0, 0, 255, 0);}
  
  Attente = 0;
  Serial.println(F("Attente de commande : "));
  if (Serial.available() > 0){
  commande = Serial.readString();
  if(commande.indexOf("LUMIN=") == 0){
   commande = commande.substring(6);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){}
   
   }
    if (commande.indexOf("LUMIN_LOW=") == 0){
    commande = commande.substring(10);
    valeur = commande.toFloat();
    if(valeur >= 0 and valeur <= 1023){}
    }
    

  if(commande.indexOf("LUMIN_HIGH=") == 0){
    commande = commande.substring(11);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){}
    else{}
    }
  else if (commande.indexOf("TEMP_AIR=") == 0){
    commande = commande.substring(9);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){}
    else{}
    }
  else if (commande.indexOf("MIN_TEMP_AIR=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur <= -40 and valeur >= -85){}
    else{}
    }
 else if(commande.indexOf("MAX_TEMP_AIR=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur <= -40 or valeur >= -85){}
    else{}
    }
  else if (commande.indexOf("HYGR=") == 0){
    commande = commande.substring(4);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){}
    else{}
    }
  else if(commande.indexOf("HYGR_MINT=") == 0){
    commande = commande.substring(10);
    valeur = commande.toFloat();
    if(valeur <= -40 or valeur >= -85){}
    else{}
    }
  else if (commande.indexOf("HYGR_MAXT=") == 0){
    commande = commande.substring(10);
    valeur = commande.toFloat();
    if(valeur <= -40 or valeur >= -85){}
    else{}
    }
  else if (commande.indexOf("PRESSURE=") == 0){
    commande = commande.substring(9);
    valeur = commande.toFloat();
    if(valeur == 0 and valeur == 1){}
    else{}
    }
 else if(commande.indexOf("PRESSURE_MIN=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur >= 300 or valeur <= 1100){}
    else{}
    }
  else if (commande.indexOf("PERSSURE_MAX=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur >= 300 or valeur <= 1100){}
    else{}
    }
  }*/
}




void Erreur(int code_erreur){
  int i;

    /*  1-Erreur d'accès à l'horloge RTC
     *  2-Erreur d'accès aux données du GPS
     *  LOG_INTERVAL-Erreur d'accès au données d'un capteur
     *  4-Données reçues d'un capteur inchoérentes - vérification matérielle requise
     *  5-Carte SD pleine
     *  6-Erreur d'accès ou d'écriture sur la carte SD
     */




    if(code_erreur == 1){
      for(i=0;i<LOG_INTERVAL;i++){
        leds.setColorRGB(0, 255, 0, 0);
        delay(500);
        leds.setColorRGB(0, 0, 0, 255);
        delay(500);
      }
    }else if(code_erreur == 2){
      for(i=0;i<LOG_INTERVAL;i++){
        leds.setColorRGB(0, 255, 0, 0);
        delay(500);
        leds.setColorRGB(0, 255, 255, 0);
        delay(500);
      }
    }else if(code_erreur == 3){
      for(i=0;i<LOG_INTERVAL;i++){
        leds.setColorRGB(0, 255, 0, 0);
        delay(500);
        leds.setColorRGB(0, 0, 0, 255);
        delay(500);
        }
    }else if(code_erreur == 4){
     for(i=0;i<LOG_INTERVAL;i++){
        leds.setColorRGB(0, 255, 0, 0);
        delay(500);
        leds.setColorRGB(0, 0, 255, 0);
        delay(500*2);
     }
    }else if(code_erreur == 5){
      for(i=0;i<LOG_INTERVAL;i++){
        leds.setColorRGB(0, 255, 0, 0);
        delay(500);
        leds.setColorRGB(0, 255, 255, 255);
        delay(500);
        }
    }else if(code_erreur == 6){
      while(!SD.begin(4)){
        leds.setColorRGB(0, 255, 0, 0);
        delay(500);
        leds.setColorRGB(0, 255, 255, 255);
        delay(500*2);
        }ModeActuel = "standard";leds.setColorRGB(0, 0, 255, 0);
      
      
  }
}

void initialisation_capteur(capteur* cpt){


  cpt->LUMIN = 1;
  cpt->LUMIN_LOW = 255;
  cpt->LUMIN_HIGH = 768;

  cpt->TEMP_AIR = 1;
  cpt->MIN_TEMP_AIR = -10;
  cpt->MAX_TEMP_AIR = 60;

  cpt->HYGR = 1;
  cpt->HYGR_MINT = 0;
  cpt->HYGR_MAXT = 50;

  cpt->PRESSURE = 1;
  cpt->PRESSURE_MIN = 850;
  cpt->PRESSURE_MAX = 1080;

  cpt->LOG_INTERVALL = 600;
  cpt->FILE_MAX_SIZE = 4096;
  
  
  
  
  }
