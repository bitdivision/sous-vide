/*****************************************************************************
 *
 * Sous-Vide Temperature controller
 * Richard Fortescue-Webb
 *
 * Largely based on code from Bill Earl (Adafruit)
 * http://github.com/adafruit/Sous_Viduino
 *
 *
 * **************************************************************************/

//Library for the LCD
#include <LiquidCrystal.h>

//Necessary since encoder and button are not on interrupt pins.
//This will only work for certain processors.
#include <PinChangeInt.h>

//To implement the PID loop and autotuning functionality
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

//For the DS18B20 Temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

//To save PID tunings and temperatures
#include <EEPROM.h>

#define LED_RED 3
#define LED_YELLOW A1
#define LED_GREEN A0

#define SSR A6

#define ENCODER_ROT1 0
#define ENCODER_ROT2 1
#define ENCODER_SWITCH A5

#define BUZZER 4

#define LCD_RS 5
#define LCD_E 6
#define LCD_D4 7
#define LCD_D5 8
#define LCD_D6 9
#define LCD_D7 10

#define DS_TEMP A7

#define ADC_SCK A2
#define ADC_SDO A3
#define ADC_CS A4

//The Rotary encoder and switch are on port C and port D so there's no need to
//check port B for interrupts
#define NO_PORTB_PINCHANGES

//Initialise library classes
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


volatile int test_state = LOW;


void setup()
{
    lcd.begin(16,2);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    pinMode(ENCODER_SWITCH, INPUT);
    digitalWrite(ENCODER_SWITCH, HIGH);
    pinMode(ENCODER_ROT2, INPUT);
    digitalWrite(ENCODER_ROT2, HIGH);
    PCintPort::attachInterrupt(ENCODER_SWITCH, &push_switch, CHANGE);
    PCintPort::attachInterrupt(ENCODER_ROT2, &_switch, CHANGE);


    //Make sure the ADC is sleeping
    pinMode(ADC_SCK, OUTPUT);
    digitalWrite(ADC_SCK, HIGH);

}

void loop()
{


}

void push_switch()
{
    test_state=!test_state;
    digitalWrite(LED_RED, test_state);
    digitalWrite(LED_GREEN, test_state);
    digitalWrite(LED_YELLOW, test_state);
}


