/*
Ce code est une version simplifiée pour 1 imprimante connectée à 1 prise.
Eléments nécessaires:
- 1 carte Arduino Nano
- 1 module relais pour Arduino
- 2 LED (1 verte et 1 rouge) 
- 2 résistances 1K ohms pour les LED
- 1 résistance 1M ohms pour le microswitch
- 1 microswitch
- 1 écran LCD 16x2 (facultatif)
- 1 prise éléctrique

Liens des vidéos:

==================================

This code is a simplified version for 1 printer connected to 1 outlet.
Necessary items:
- 1 Arduino Nano board
- 1 relay module
- 2 LEDs (1 green and 1 red) 
- 2 1K ohm resistors for the LEDs
- 1 1M ohm resistor for the microswitch
- 1 microswitch
- 1 16x2 LCD screen (optional)
- 1 electrical outlet

Video links:

*/

//Programme: Printer_AutoPower_Off_1_Printer_30-06-2024
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

// LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // pour l'écran LCD 16x2 / for 16x2 LCD screen
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);    // pour l'écran LCD 16x2 / for 16x2 LCD screen

unsigned long currentMillisMyPrinter;
unsigned long previousMillisMyPrinter;

long printer_TurnOn_Confirmation = 10000;     // délai de confirmation de l'état du capteur pour éviter des mises sous tension intempestives = 10 secondes (10000)
                                              // delay to confirm the state of the sensor to avoid untimely power on = 10 seconds (10000)
long printer_TurnOff_Confirmation = 2700000;  // délai maximum de mise hors tension = 45 minutes (2700000) (en prévision du temps de nivellement automatique)
                                              // maximum delay to power off = 45 minutes (2700000) (in anticipation of auto-leveling time)

bool relay_MyPrinterState = LOW;  // stockera l'état du relais pour l'imprimante   
                                  // will store the state of the relay for the printer 

String MyPrinter_OnOff;  // stockera le texte à afficher pour l'état de l'imprimante (ON ou OFF) 
                         // will store the text to display for the state of the printer (ON or OFF)

const int LED_Green_MyPrinter = 2;  // pin LED verte / pin for green LED
const int LED_Red_MyPrinter = 3;    // pin LED rouge / pin for red LED 
const int sensor_MyPrinter = 4;     // pin micro-switch
const int relay_MyPrinter = 5;      // pin du relai / relay pin 

bool stateSensorMyPrinter;  // le microswitch est-il appuyé ou pas ? / is the microswitch pressed or not?

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

// création de l'objet Flasher pour les LED rouge et verte de l'imprimante
// creation of the Flasher object for the red and green LEDs of the printer
Flasher led_MyPrinter_Red(LED_Red_MyPrinter, OnTime_redLED, OffTime_redLED);
Flasher led_MyPrinter_Green(LED_Green_MyPrinter, OnTime_greenLED, OffTime_greenLED);

//================================================

void setup() {

  Serial.begin(9600);  // initialisation du moniteur série / initializing the serial monitor
  lcd.begin(16, 2);    // initialisation du LCD / initializing the LCD
  lcd.clear();       

  pinMode(relay_MyPrinter, OUTPUT);
  pinMode(LED_Green_MyPrinter, OUTPUT);
  pinMode(LED_Red_MyPrinter, OUTPUT);
  pinMode(sensor_MyPrinter, INPUT);

  // éteindre le relais au démarrage / turning off the relay on start up
  digitalWrite(relay_MyPrinter, LOW);

  // éteindre les LED vertes au démarrage / turning off the green LEDs on start up
  digitalWrite(LED_Green_MyPrinter, LOW);

  // allumer les LED rouges au démarrage / turning on the red LEDs on start up
  digitalWrite(LED_Red_MyPrinter, HIGH);

  // affichage de l'état de l'imprimante sur l'écran LCD / displaying the status of the printer on the lcd
  lcd.setCursor(0, 0);
  lcd.print("STATE OF PRINTER");

  lcd.setCursor(0, 1);
  lcd.print("Printer: ");

  Serial.println("Program name: Printer_AutoPower_Off_1_Printer_30-06-2024");

  delay(2000);

}  // end of setup

//=========================

void loop() {

  currentMillisMyPrinter = millis();
  stateSensorMyPrinter = digitalRead(sensor_MyPrinter);

  if (stateSensorMyPrinter == LOW) {  

    led_MyPrinter_Green.Update();  // clignotement de la LED verte / green LED flashing

    currentMillisMyPrinter = millis();
    if (currentMillisMyPrinter - previousMillisMyPrinter >= printer_TurnOn_Confirmation) { 
      stateSensorMyPrinter = digitalRead(sensor_MyPrinter);
      if (stateSensorMyPrinter == LOW) {  
        digitalWrite(LED_Green_MyPrinter, HIGH);
        digitalWrite(LED_Red_MyPrinter, LOW);
        digitalWrite(relay_MyPrinter, HIGH);
        MyPrinter_OnOff = "ON ";    // statut de l'imprimante / state of the printer
        previousMillisMyPrinter = currentMillisMyPrinter;
      }                                 
    }                               
  }                                    
  else if (stateSensorMyPrinter == HIGH) {  

    led_MyPrinter_Red.Update();  // clignotement de la LED rouge / red LED flashing

    currentMillisMyPrinter = millis();

    if (currentMillisMyPrinter - previousMillisMyPrinter >= printer_TurnOff_Confirmation) {  
      stateSensorMyPrinter = digitalRead(sensor_MyPrinter);
      if (stateSensorMyPrinter == HIGH) {  
        digitalWrite(LED_Green_MyPrinter, LOW);
        digitalWrite(LED_Red_MyPrinter, HIGH);
        digitalWrite(relay_MyPrinter, LOW);
        MyPrinter_OnOff = "OFF";    // statut de l'imprimante / state of the printer
        
        previousMillisMyPrinter = currentMillisMyPrinter;
      } 
    }   
  }     

  //=====================================================
 
  lcd.setCursor(9, 1);    
  lcd.print(MyPrinter_OnOff);  // afficher l'état de l'imprimante à l'écran / display the status of the printer on the LCD

  // afficher l'état de l'imprimante au moniteur série / display the status of the printer on the serial monitor
  Serial.print("Imprimante /Printer: ");
  Serial.println(MyPrinter_OnOff);

}  // end of loop



