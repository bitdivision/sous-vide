#include <LiquidCrystal.h>
#include <PinChangeInt.h>
#include <TinySoftwareSPI.h>


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

//The Rotary encoder and switch is on port C and port D so there's no need to
//check port B for interrupts
#define NO_PORTB_PINCHANGES

//Initialise library classes
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


volatile int test_state = HIGH;
unsigned long adc_val = 0;
float currentTemp = 0;
unsigned long test = 0;


void setup()
{
    lcd.begin(16,2);

    pinMode(LED_RED, OUTPUT);
    pinMode(ENCODER_SWITCH, INPUT);
    digitalWrite(ENCODER_SWITCH, HIGH);
    pinMode(ENCODER_ROT2, INPUT);
    digitalWrite(ENCODER_ROT2, HIGH);
    PCintPort::attachInterrupt(ENCODER_SWITCH, &push_switch, CHANGE);
    PCintPort::attachInterrupt(ENCODER_ROT2, &push_switch, CHANGE);


    //Try to set up the ADC
    //Pull the CS pin low to enter 2-wire mode
    pinMode(ADC_CS, OUTPUT);
    digitalWrite(ADC_CS, LOW);

    //Set up SCK and SDO pins
    pinMode(ADC_SCK, OUTPUT);
    pinMode(ADC_SDO, INPUT);

    //Set SCK high to begin with
    digitalWrite(ADC_SCK, HIGH);

    delayMicroseconds(100);

    //check if SDO has gone low
    if (digitalRead(ADC_SDO)!=LOW)
    {
        //Send an SCK pulse and see if that works
        for (int i =0; i<17;i++)
        {
            digitalWrite(ADC_SCK, LOW);
            delayMicroseconds(20);
            digitalWrite(ADC_SCK, HIGH);
            delayMicroseconds(20);
            if (digitalRead(ADC_SDO)==LOW)
            {
                lcd.setCursor(6,0);
                lcd.print("GOOD");
                break;
            }
            else
            {
                lcd.setCursor(6,0);
                lcd.print("NO");
            }
        }
    }
    else
    {
        lcd.setCursor(10,1);
        lcd.print("HIGH");
    }
}

void loop()
{
    //lcd.setCursor(0,1);
    //lcd.print(millis()/1000);
    


    if (digitalRead(ADC_SDO)==LOW)
    {
        lcd.setCursor(0,0);
        lcd.print("SDO");

        for (uint8_t _bit = 0; _bit < 16; _bit++)
        {
            digitalWrite(ADC_SCK, LOW);
            delayMicroseconds(50);
            digitalWrite(ADC_SCK, HIGH);

            //Read the bit. ADC is MSB first, bitWrite is LSB first.
            bitWrite(adc_val,(15-_bit), digitalRead(ADC_SDO));
            delayMicroseconds(50);
        }

        //Then start another conversion
        digitalWrite(ADC_SCK, LOW);
        delayMicroseconds(50);
        digitalWrite(ADC_SCK, HIGH);
        
        test = adc_val<<15;
        test = test - (32768<<15);
        test = test>>15;
        //currentTemp = ((adc_val-32768.0)/32768.0)*5.0;
        test = test * 5;
        currentTemp = test/32768.0;
        lcd.setCursor(0,1);
        lcd.print("        ");
        lcd.setCursor(0,1);
        lcd.print(currentTemp,4);

        lcd.setCursor(8,1);
        lcd.print("      ");
        lcd.setCursor(8,1);
        lcd.print(adc_val);

    }
    else
    {
        lcd.setCursor(0,0);
        lcd.print("HIGH");
    }


}

void push_switch()
{
    test_state=!test_state;
    digitalWrite(LED_RED, test_state);
}


