#include <ChainableLED.h>
#define NUM_LEDS  5
ChainableLED leds(6, 5, NUM_LEDS);


String ModeActuel = "";


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
    
};

capteur cpt;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initialisation_capteur(&cpt);
}

void loop() {
  // put your main code here, to run repeatedly:
  Configuration(&cpt);
}

void Configuration(capteur *cpt){

String commande = "";
int valeur;
int Attente=0;

  Serial.println(F("Attente de commande : "));
  delay(1000);
  while(Attente <= 20 and Serial.available() <= 0){Serial.println(Attente);Attente++;delay(1000);}
  if(Attente >= 20){ModeActuel = "standard";leds.setColorRGB(0, 0, 255, 0);}
  Attente = 0;
  if (Serial.available() > 0){
  commande = Serial.readString();
  if(commande.indexOf("LUMIN=") == 0){
   commande = commande.substring(6);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){cpt->LUMIN=valeur;}
   
   }
    if (commande.indexOf("LUMIN_LOW=") == 0){
    commande = commande.substring(10);
    valeur = commande.toFloat();
    if(valeur >= 0 and valeur <= 1023){cpt->LUMIN_LOW=valeur;}
    }
    

  if(commande.indexOf("LUMIN_HIGH=") == 0){
    commande = commande.substring(11);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){cpt->LUMIN_HIGH=valeur;}
    else{}
    }
  else if (commande.indexOf("TEMP_AIR=") == 0){
    commande = commande.substring(9);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){cpt->TEMP_AIR=valeur;}
    else{}
    }
  else if (commande.indexOf("MIN_TEMP_AIR=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur <= -40 and valeur >= -85){cpt->MIN_TEMP_AIR=valeur;}
    else{}
    }
 else if(commande.indexOf("MAX_TEMP_AIR=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur <= -40 or valeur >= -85){cpt->MAX_TEMP_AIR=valeur;}
    else{}
    }
  else if (commande.indexOf("HYGR=") == 0){
    commande = commande.substring(4);
    valeur = commande.toFloat();
    if(valeur == 0 or valeur == 1){cpt->HYGR=valeur;}
    else{}
    }
  else if(commande.indexOf("HYGR_MINT=") == 0){
    commande = commande.substring(10);
    valeur = commande.toFloat();
    if(valeur <= -40 or valeur >= -85){cpt->HYGR_MINT=valeur;}
    else{}
    }
  else if (commande.indexOf("HYGR_MAXT=") == 0){
    commande = commande.substring(10);
    valeur = commande.toFloat();
    if(valeur <= -40 or valeur >= -85){cpt->HYGR_MAXT=valeur;}
    else{}
    }
  else if (commande.indexOf("PRESSURE=") == 0){
    commande = commande.substring(9);
    valeur = commande.toFloat();
    if(valeur == 0 and valeur == 1){cpt->PRESSURE=valeur;}
    else{}
    }
 else if(commande.indexOf("PRESSURE_MIN=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur >= 300 or valeur <= 1100){cpt->PRESSURE_MAX=valeur;}
    else{}
    }
  else if (commande.indexOf("PERSSURE_MAX=") == 0){
    commande = commande.substring(13);
    valeur = commande.toFloat();
    if(valeur >= 300 or valeur <= 1100){cpt->PRESSURE_MIN=valeur;}
    else{}
    }
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
  
  
  
  }
