/*
Ce code est une version pour 4 imprimantes connectées à 4 prises.
Eléments nécessaires:
- 1 carte Arduino Mega
- 1 module 4 relais pour Arduino
- 8 LED (4 vertes et 4 rouges) 
- 8 résistances 1K ohms pour les LED
- 4 résistances 1M ohms pour les microswitch
- 4 microswitch
- 1 écran LCD 20x4 (facultatif)
- 4 prises éléctriques

Liens des vidéos:

==================================

This code is a version for 4 printers connected to 4 sockets.
Necessary items:
- 1 Arduino Mega board
- 1 4 relay module for Arduino
- 8 LEDs (4 green and 4 red) 
- 8 1K ohm resistors for the LEDs
- 4 1M ohm resistors for the microswitches
- 4 microswitches
- 1 20x4 LCD screen (optional)
- 4 electrical outlets

Video links:


*/

//Programme: Printer_AutoPower_Off_4_Printer_30-06-2024
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // pour l'écran LCD 16x2 / for 16x2 LCD screen
                                                                // pour l'écran LCD 16x2 / for 16x2 LCD screen
unsigned long currentMillisPrinter_1;
unsigned long previousMillisPrinter_1;

unsigned long currentMillisPrinter_2;
unsigned long previousMillisPrinter_2;

unsigned long currentMillisPrinter_3;
unsigned long previousMillisPrinter_3;

unsigned long currentMillisPrinter_4;
unsigned long previousMillisPrinter_4;

long printer_TurnOn_Confirmation = 10000;     // délai de confirmation de l'état du capteur pour éviter des mises sous tension intempestives = 10 secondes (10000)
                                              // delay to confirm the state of the sensor to avoid untimely power on = 10 seconds (10000)
long printer_TurnOff_Confirmation = 2700000;  // délai maximum de mise hors tension = 45 minutes (2700000) (en prévision du temps de nivellement automatique)
                                              // maximum delay to power off = 45 minutes (2700000) (in anticipation of auto-leveling time)

// long printer_TurnOn_Confirmation = 2000;   // delay to confirm the state of the sensor to avoid untimely power on = 10 seconds (10000)
// long printer_TurnOff_Confirmation = 4000;

const int my_buzzer = 10;
const int bipDuration = 500;
const int pauseDuration = 250;

const int LED_Red_Printer_1 = 23;     // pin LED rouge pour imprimante 1 / pin for red LED for printer 1
const int LED_Green_Printer_1 = 27;   // pin LED verte pour imprimante 1 / pin for green LED for printer 1

const int LED_Red_Printer_2 = 31;     // pin LED rouge pour imprimante 2 / pin for red LED for printer 2
const int LED_Green_Printer_2 = 35;   // pin LED verte pour imprimante 2 / pin for green LED for printer 2

const int LED_Red_Printer_3 = 39;     // pin LED rouge pour imprimante 3 / pin for red LED for printer 3
const int LED_Green_Printer_3 = 43;   // pin LED verte pour imprimante 3 / pin for green LED for printer 3

const int LED_Red_Printer_4 = 49;     // pin LED rouge pour imprimante 4 / pin for red LED for printer 4
const int LED_Green_Printer_4 = 53;   // pin LED verte pour imprimante 4 / pin for green LED for printer 4

const int relay_Printer_1 = 51;       // pin du relai imprimante 1 / relay pin for printer 1
const int relay_Printer_2 = 50;       // pin du relai imprimante 2 / relay pin for printer 2
const int relay_Printer_3 = 52;       // pin du relai imprimante 3 / relay pin for printer 3
const int relay_Printer_4 = 32;       // pin du relai imprimante 4 / relay pin for printer 4

const int sensor_Printer_1 = 6;       // pin micro-switch pour imprimante 1 / pin micro-switch for printer 1
const int sensor_Printer_2 = 5;       // pin micro-switch pour imprimante 2 / pin micro-switch for printer 2
const int sensor_Printer_3 = 4;       // pin micro-switch pour imprimante 3 / pin micro-switch for printer 3
const int sensor_Printer_4 = 3;      // pin micro-switch pour imprimante 4 / pin micro-switch for printer 4

int stateSensorPrinter_1;  // le microswitch imprimante 1 est-il appuyé ou pas ? / is the microswitch for printer 1 pressed or not?
int stateSensorPrinter_2;  // le microswitch imprimante 2 est-il appuyé ou pas ? / is the microswitch for printer 2 pressed or not?
int stateSensorPrinter_3;  // le microswitch imprimante 3 est-il appuyé ou pas ? / is the microswitch for printer 3 pressed or not?
int stateSensorPrinter_4;  // le microswitch imprimante 4 est-il appuyé ou pas ? / is the microswitch for printer 4 pressed or not?

int relay_Printer_1State = LOW;  // état du relais pour l'imprimante 1 / state of the relay for printer 1
int relay_Printer_2State = LOW;  // état du relais pour l'imprimante 2 / state of the relay for printer 2
int relay_Printer_3State = LOW;  // état du relais pour l'imprimante 3 / state of the relay for printer 3
int relay_Printer_4State = LOW;  // état du relais pour l'imprimante 4 / state of the relay for printer 4

String Printer_1_OnOff;  // texte à afficher pour l'état de l'imprimante 1 (ON ou OFF) / text to display for the state of printer 1 (ON or OFF)
String Printer_2_OnOff;  // texte à afficher pour l'état de l'imprimante 2 (ON ou OFF) / text to display for the state of printer 2 (ON or OFF)
String Printer_3_OnOff;  // texte à afficher pour l'état de l'imprimante 3 (ON ou OFF) / text to display for the state of printer 3 (ON or OFF)
String Printer_4_OnOff;  // texte à afficher pour l'état de l'imprimante 4 (ON ou OFF) / text to display for the state of printer 4 (ON or OFF)

// délais des LED allumées/éteintes  / LED on/off delays
long OnTime_redLED = 1500;
long OffTime_redLED = 800;
long OnTime_greenLED = 2000;  
long OffTime_greenLED = 500; 

//================================================
// création de la classe pour le clignotement des LED
// creating the class for LED blinking
class Flasher {
  int ledPin;                         
  long OnTime;                        
  long OffTime;                       
  int ledState;                        
  unsigned long previousMillis_blink; 
public:
  Flasher(int pin, long on, long off) {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);
    OnTime = on;
    OffTime = off;
    ledState = LOW;
    previousMillis_blink = 0;
  }
  void Update() {
    // vérifie s'il est temps de changer l'état de la LED
    // check to see if it's time to change the state of the LED
    unsigned long currentMillis_blink = millis();
    if ((ledState == HIGH) && (currentMillis_blink - previousMillis_blink >= OnTime)) {
      ledState = LOW;                             
      previousMillis_blink = currentMillis_blink;  
      digitalWrite(ledPin, ledState);              
    } else if ((ledState == LOW) && (currentMillis_blink - previousMillis_blink >= OffTime)) {
      ledState = HIGH;                             
      previousMillis_blink = currentMillis_blink; 
      digitalWrite(ledPin, ledState);             
    }
  }
};

// création de l'objet Flasher pour les LED rouges des imprimantes
// creation of the Flasher object for the red LEDs of the printers
Flasher led_Printer_1_Red(LED_Red_Printer_1, OnTime_redLED, OffTime_redLED);
Flasher led_Printer_2_Red(LED_Red_Printer_2, OnTime_redLED, OffTime_redLED);
Flasher led_Printer_3_Red(LED_Red_Printer_3, OnTime_redLED, OffTime_redLED);
Flasher led_Printer_4_Red(LED_Red_Printer_4, OnTime_redLED, OffTime_redLED);

// création de l'objet Flasher pour les LED vertes des imprimantes
// creation of the Flasher object for the green LEDs of the printers
Flasher led_Printer_1_Green(LED_Green_Printer_1, OnTime_greenLED, OffTime_greenLED);
Flasher led_Printer_2_Green(LED_Green_Printer_2, OnTime_greenLED, OffTime_greenLED);
Flasher led_Printer_3_Green(LED_Green_Printer_3, OnTime_greenLED, OffTime_greenLED);
Flasher led_Printer_4_Green(LED_Green_Printer_4, OnTime_greenLED, OffTime_greenLED);

//================================================

void setup() {

  Serial.begin(9600); 
  lcd.begin(20, 4);   
  lcd.clear();       

  pinMode(my_buzzer, OUTPUT);

  pinMode(relay_Printer_1, OUTPUT);
  pinMode(relay_Printer_2, OUTPUT);
  pinMode(relay_Printer_3, OUTPUT);
  pinMode(relay_Printer_4, OUTPUT);

  pinMode(LED_Green_Printer_1, OUTPUT);
  pinMode(LED_Red_Printer_1, OUTPUT);

  pinMode(LED_Green_Printer_2, OUTPUT);
  pinMode(LED_Red_Printer_2, OUTPUT);

  pinMode(LED_Green_Printer_3, OUTPUT);
  pinMode(LED_Red_Printer_3, OUTPUT);

  pinMode(LED_Green_Printer_4, OUTPUT);
  pinMode(LED_Red_Printer_4, OUTPUT);

  pinMode(sensor_Printer_1, INPUT);
  pinMode(sensor_Printer_2, INPUT);
  pinMode(sensor_Printer_3, INPUT);
  pinMode(sensor_Printer_4, INPUT);

  // éteindre les relais au démarrage / turning off the relays on start up
  digitalWrite(relay_Printer_1, LOW);
  digitalWrite(relay_Printer_2, LOW);
  digitalWrite(relay_Printer_3, LOW);
  digitalWrite(relay_Printer_4, LOW);

  // éteindre les LED vertes au démarrage / turning off the green LEDs on start up
  digitalWrite(LED_Green_Printer_1, LOW);
  digitalWrite(LED_Green_Printer_2, LOW);
  digitalWrite(LED_Green_Printer_3, LOW);
  digitalWrite(LED_Green_Printer_4, LOW);

  // allumer les LED rouges au démarrage / turning on the red LEDs on start up
  digitalWrite(LED_Red_Printer_1, HIGH);
  digitalWrite(LED_Red_Printer_2, HIGH);
  digitalWrite(LED_Red_Printer_3, HIGH);
  digitalWrite(LED_Red_Printer_4, HIGH);

  // affichage de l'état des imprimantes sur l'écran LCD / displaying the status of the printers on the lcd
  lcd.setCursor(0, 0);
  lcd.print(" STATES OF PRINTERS ");

  lcd.setCursor(0, 1);
  lcd.print("     ----------     ");

  lcd.setCursor(2, 2);
  lcd.print("PR1:");    // imprimante 1 / printer 1

  lcd.setCursor(12, 2);
  lcd.print("PR2:");    // imprimante 2 / printer 2

  lcd.setCursor(2, 3);
  lcd.print("PR3:");    // imprimante 3 / printer 3

  lcd.setCursor(12, 3);
  lcd.print("PR4:");    // imprimante 4 / printer 4

  Serial.println("Programme: Printer_AutoPower_Off_4_Printer_30-06-2024"); // affiche le nom du programme sur le moniteur série
                                                                           // display the program name on the serial monitor

  delay(2000);

}  // end of setup

//=========================

void loop() {

   // pour l'imprimante 1 / for printer 1
  currentMillisPrinter_1 = millis();
  stateSensorPrinter_1 = digitalRead(sensor_Printer_1);

  if (stateSensorPrinter_1 == LOW) {  
    Serial.println("Sensor printer 1 v3SE openned");

    led_Printer_1_Green.Update();  // clignotement de la LED verte de l'imprimante 1 / printer 1 green LED flashing

    currentMillisPrinter_1 = millis();
    if (currentMillisPrinter_1 - previousMillisPrinter_1 >= printer_TurnOn_Confirmation) { 
      stateSensorPrinter_1 = digitalRead(sensor_Printer_1);
      if (stateSensorPrinter_1 == LOW) {  
        digitalWrite(LED_Green_Printer_1, HIGH);
        digitalWrite(LED_Red_Printer_1, LOW);
        digitalWrite(relay_Printer_1, HIGH);
        Printer_1_OnOff = "ON ";
        previousMillisPrinter_1 = currentMillisPrinter_1;
      }                                  
    }                                   
  }                                      
  else if (stateSensorPrinter_1 == HIGH) {  
    Serial.println("Sensor printer 1 triggered");

    led_Printer_1_Red.Update();  // clignotement de la LED rouge de l'imprimante 1 / printer 1 red LED flashing

    currentMillisPrinter_1 = millis();

    if (currentMillisPrinter_1 - previousMillisPrinter_1 >= printer_TurnOff_Confirmation) {  
      stateSensorPrinter_1 = digitalRead(sensor_Printer_1);
      if (stateSensorPrinter_1 == HIGH) {  
        digitalWrite(LED_Green_Printer_1, LOW);
        digitalWrite(LED_Red_Printer_1, HIGH);
        digitalWrite(relay_Printer_1, LOW);
        Printer_1_OnOff = "OFF";
        
        previousMillisPrinter_1 = currentMillisPrinter_1;
      }  
    }    
  }      

  //---------------------------------------------------------------
 // pour l'imprimante 2 / for printer 2
  currentMillisPrinter_2 = millis();
  stateSensorPrinter_2 = digitalRead(sensor_Printer_2); 

  if (stateSensorPrinter_2 == LOW) {  
    Serial.println("Sensor printer 2 openned");

    led_Printer_2_Green.Update();  // clignotement de la LED verte de l'imprimante 2 / printer 2 green LED flashing

    currentMillisPrinter_2 = millis();                                                  
    if (currentMillisPrinter_2 - previousMillisPrinter_2 >= printer_TurnOn_Confirmation) {  
      stateSensorPrinter_2 = digitalRead(sensor_Printer_2);                                 
      if (stateSensorPrinter_2 == LOW) {                                               
        digitalWrite(LED_Green_Printer_2, HIGH);                                       
        digitalWrite(LED_Red_Printer_2, LOW);                                          
        digitalWrite(relay_Printer_2, HIGH);                                           
        Printer_2_OnOff = "ON ";                                                      
        previousMillisPrinter_2 = currentMillisPrinter_2;                                
      }                                                                            
    }                                                                            
  }                                                                                
  else if (stateSensorPrinter_2 == HIGH) {                                            
    Serial.println("Sensor printer 2 triggered");

    led_Printer_2_Red.Update();  // clignotement de la LED rouge de l'imprimante 2 / printer 2 red LED flashing

    currentMillisPrinter_2 = millis(); 

    if (currentMillisPrinter_2 - previousMillisPrinter_2 >= printer_TurnOff_Confirmation) {  
      stateSensorPrinter_2 = digitalRead(sensor_Printer_2);                                 
      if (stateSensorPrinter_2 == HIGH) {                                                
        digitalWrite(LED_Red_Printer_2, LOW);
        digitalWrite(LED_Green_Printer_2, LOW);  
        digitalWrite(LED_Red_Printer_2, HIGH);   
        digitalWrite(relay_Printer_2, LOW);      
        Printer_2_OnOff = "OFF";     
        
        previousMillisPrinter_2 = currentMillisPrinter_2;
      }                                 
    }                                 
  }                                         

  //---------------------------------------------------------------
  // pour l'imprimante 3 / for printer 3
  currentMillisPrinter_3 = millis();
  stateSensorPrinter_3 = digitalRead(sensor_Printer_3);

  if (stateSensorPrinter_3 == LOW) {  
    Serial.println("Sensor printer 3 openned");

    led_Printer_3_Green.Update();  // clignotement de la LED verte de l'imprimante 3 / printer 3 green LED flashing

    currentMillisPrinter_3 = millis();
    if (currentMillisPrinter_3 - previousMillisPrinter_3 >= printer_TurnOn_Confirmation) {  
      stateSensorPrinter_3 = digitalRead(sensor_Printer_3);
      if (stateSensorPrinter_3 == LOW) {  
        digitalWrite(LED_Green_Printer_3, HIGH);
        digitalWrite(LED_Red_Printer_3, LOW);
        digitalWrite(relay_Printer_3, HIGH);
        Printer_3_OnOff = "ON ";
        previousMillisPrinter_3 = currentMillisPrinter_3;
      }                                
    }                                  
  }                                    
  else if (stateSensorPrinter_3 == HIGH) {  
    Serial.println("Sensor printer 3 triggered");

    led_Printer_3_Red.Update();  // clignotement de la LED rouge de l'imprimante 3 / printer 3 red LED flashing

    currentMillisPrinter_3 = millis();

    if (currentMillisPrinter_3 - previousMillisPrinter_3 >= printer_TurnOff_Confirmation) {  
      stateSensorPrinter_3 = digitalRead(sensor_Printer_3);
      if (stateSensorPrinter_3 == HIGH) {  
        digitalWrite(LED_Green_Printer_3, LOW);
        digitalWrite(LED_Red_Printer_3, HIGH);
        digitalWrite(relay_Printer_3, LOW);
        Printer_3_OnOff = "OFF";
        
        previousMillisPrinter_3 = currentMillisPrinter_3;
      }  
    }    
  }      

  //---------------------------------------------------------------
  // pour l'imprimante 4 / for printer 4
  currentMillisPrinter_4 = millis();
  stateSensorPrinter_4 = digitalRead(sensor_Printer_4);

  if (stateSensorPrinter_4 == LOW) {  
    Serial.println("Sensor printer 4 openned");

    led_Printer_4_Green.Update();  // clignotement de la LED verte de l'imprimante 4 / printer 4 green LED flashing

    currentMillisPrinter_4 = millis();
    if (currentMillisPrinter_4 - previousMillisPrinter_4 >= printer_TurnOn_Confirmation) {  
      stateSensorPrinter_4 = digitalRead(sensor_Printer_4);
      if (stateSensorPrinter_4 == LOW) {  
        digitalWrite(LED_Green_Printer_4, HIGH);
        digitalWrite(LED_Red_Printer_4, LOW);
        digitalWrite(relay_Printer_4, HIGH);
        Printer_4_OnOff = "ON ";
        previousMillisPrinter_4 = currentMillisPrinter_4;
      }                                
    }                                  
  }                                    
  else if (stateSensorPrinter_4 == HIGH) {  
    Serial.println("Sensor printer 4 triggered");

    led_Printer_4_Red.Update();  // clignotement de la LED rouge de l'imprimante 4 / printer 4 red LED flashing

    currentMillisPrinter_4 = millis();

    if (currentMillisPrinter_4 - previousMillisPrinter_4 >= printer_TurnOff_Confirmation) {  
      stateSensorPrinter_4 = digitalRead(sensor_Printer_4);
      if (stateSensorPrinter_4 == HIGH) {  
        digitalWrite(LED_Green_Printer_4, LOW);
        digitalWrite(LED_Red_Printer_4, HIGH);
        digitalWrite(relay_Printer_4, LOW);
        Printer_4_OnOff = "OFF";
        
        previousMillisPrinter_4 = currentMillisPrinter_4;
      }  
    }    
  }      
  
  //=====================================================
  lcd.setCursor(6, 2);     
  lcd.print(Printer_1_OnOff);   // affiche l'état de l'imprimante 1 sur le LCD  / displays the status of printer 1 on the LCD

  lcd.setCursor(16, 2);  
  lcd.print(Printer_2_OnOff);   // affiche l'état de l'imprimante 2 sur le LCD  / displays the status of printer 2 on the LCD

  lcd.setCursor(6, 3);    
  lcd.print(Printer_3_OnOff);   // affiche l'état de l'imprimante 3 sur le LCD  / displays the status of printer 3 on the LCD
  
  lcd.setCursor(16, 3);    
  lcd.print(Printer_4_OnOff);   // affiche l'état de l'imprimante 4 sur le LCD  / displays the status of printer 4 on the LCD

  // afficher l'état des imprimantes au moniteur série / display the status of the printers on the serial monitor
  Serial.print("Imprimante 1 / Printer 1: ");
  Serial.println(Printer_1_OnOff);
  Serial.println("---------------------");

  Serial.print("Imprimante 2 / Printer 2: ");
  Serial.println(Printer_2_OnOff);
  Serial.println("---------------------");

  Serial.print("Imprimante 3 / Printer 3: ");
  Serial.println(Printer_3_OnOff);
  Serial.println("---------------------");

  Serial.print("Imprimante 4 / Printer 4: ");
  Serial.println(Printer_4_OnOff);
  Serial.println("=====================");

}  // end of loop


