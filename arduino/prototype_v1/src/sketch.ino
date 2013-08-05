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

//The Rotary encoder and switch are on port C and port D so there's no need to
//check port B for interrupts
#define NO_PORTB_PINCHANGES

#define NO_PIN_NUMBER

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
//#include <EEPROM.h>

#define LED_RED 3
#define LED_YELLOW A1
#define LED_GREEN A0

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


#define SSR A2
#define DS_TEMP A3


double tempSet, currentTemp, PIDOutput;

double Kp, Ki, Kd;

//10 second window (i.e PWM frequency = 0.1Hz)
int windowSize = 10000;

unsigned long windowStartTime;

volatile long onTime = 0;

//Autotune vars
byte ATuneModeRemember = 2;

double aTuneStep = 500;
double aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

boolean tuning = false;

volatile int encoderPos = 0;
int lastEncoderPos = 0;

byte switchState = 0;
unsigned long switchStartTime;

unsigned long timerSetTime = 0;

boolean timerIsOn = false;

boolean A_set = false, B_set = false;

boolean isRunning = false;

//Initialise library classes
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

//Set up the PID with the params
PID tempPID(&currentTemp, &PIDOutput, &tempSet, Kp, Ki, Kd, DIRECT);

//PID_ATune aTune(&currentTemp, &PIDOutput);


//Temperature sensor stuff
OneWire oneWire(DS_TEMP);
DallasTemperature tempSense(&oneWire);
DeviceAddress sensorAddress;


enum operatingState {OFF=0, RUN, TIMER_MENU, TIMER_SET, AUTOTUNE, DO_AUTOTUNE, TUNE_MENU, TUNE_KP, TUNE_KI, TUNE_KD};
operatingState currentState = OFF, lastState = OFF;


byte arrow_left[8] = {
    B00011,
    B00111,
    B01111,
    B11111,
    B01111,
    B00111,
    B00011,
};

byte arrow_right[8] = {
    B11000,
    B11100,
    B11110,
    B11111,
    B11110,
    B11100,
    B11000,
};





void setup()
{
    lcd.createChar(0, arrow_left);
    lcd.createChar(1, arrow_right);
    lcd.begin(16,2);

    //Initialise the LED pins
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    //Initialise the encoder/switch and set them to interrupt
    pinMode(ENCODER_SWITCH, INPUT);
    digitalWrite(ENCODER_SWITCH, HIGH);
    pinMode(ENCODER_ROT2, INPUT);
    digitalWrite(ENCODER_ROT2, HIGH);
    pinMode(ENCODER_ROT1, INPUT);
    digitalWrite(ENCODER_ROT1, HIGH);
    PCintPort::attachInterrupt(ENCODER_SWITCH, &push_switch, CHANGE);
    PCintPort::attachInterrupt(ENCODER_ROT1, &doEncoderA, CHANGE);
    PCintPort::attachInterrupt(ENCODER_ROT2, &doEncoderB, CHANGE);

    //Set up SSR and turn it off
    pinMode(SSR, OUTPUT);
    digitalWrite(SSR, LOW);

    //Initalise the Temperature sensor
    tempSense.begin();

    //Get the sensor's address
    if (!tempSense.getAddress(sensorAddress, 0))
    {
        /*        lcd.setCursor(0,0);
                  lcd.print(F("NO TEMP SENSOR!"));
                  lcd.setCursor(0,1);
                  lcd.print(F("PLUG IN + REBOOT"));
                  while (1)
                  {
                  digitalWrite(LED_RED, HIGH);
                  digitalWrite(LED_GREEN, HIGH);
                  digitalWrite(LED_YELLOW, HIGH);
                  delay(500);
                  digitalWrite(LED_RED, LOW);
                  digitalWrite(LED_GREEN, LOW);
                  digitalWrite(LED_YELLOW, LOW);
                  delay(500);
                  }*/
    }

    tempSense.setResolution(sensorAddress, 12);
    tempSense.setWaitForConversion(false);

    //Start a conversion
    tempSense.requestTemperaturesByAddress(sensorAddress);

    //Start-up screen
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print(F("SOUS  VIDE"));
    lcd.setCursor(3,1);
    lcd.print(F("CONTROLLER"));

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, HIGH);


    delay(2000);

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);

    lcd.clear();

    //Initialise some PID stuff
    windowStartTime = millis();
    tempSet = 50;

    //Get the current Temperature to start off with
    currentTemp = tempSense.getTempC(sensorAddress);

    LoadPIDParams();
    tempPID.SetTunings(Kp, Ki, Kd);

    tempPID.SetSampleTime(1000);
    tempPID.SetOutputLimits(0, windowSize);

    //Timer2 interrupt every 15ms
    TCCR2A = 0;
    TCCR2B = 1<<CS22 | 1<<CS21| 1<<CS20;
    TIMSK2 |= 1<<TOIE2;
}



SIGNAL(TIMER2_OVF_vect)
{
    if (currentState == OFF)
    {
        digitalWrite(SSR, LOW);
    }
    else
    {
        DriveOutput();
    }
}

void loop()
{
    lcd.clear();

    switch(currentState)
    {
        case OFF:
            Off();
            break;
        case RUN:
            Run();
            break;
        case TIMER_MENU:
            Timer();
            break;
        case TIMER_SET:
            Timer_Set();
            break;
        case AUTOTUNE:
            Autotune();
            break;
        case TUNE_MENU:
            Tune();
            break;
        case TUNE_KI:
            Tune_Ki();
            break;
        case TUNE_KP:
            Tune_Kp();
            break;
        case TUNE_KD:
            Tune_Kd();
            break;
    }
}

//Displays the setpoint and temperature
void displayTemps(byte inputState)
{

    lcd.setCursor(0,0);
    lcd.print(F("SET:      "));
    lcd.setCursor(5,0);
    lcd.print(tempSet,1);
    lcd.setCursor(13,0);

    //If we were called from OFF
    if (inputState == 0)
        lcd.print(F("OFF"));
    else
        lcd.print(F("ON"));

    lcd.setCursor(0,1);
    lcd.print(F("TMP:"));
    //Wait for an input and keep the temperature updated
    while(checkInputs() == 0)
    {
        tempSense.requestTemperaturesByAddress(sensorAddress);
        currentTemp = tempSense.getTempC(sensorAddress);

        lcd.setCursor(5,1);
        lcd.print(currentTemp,2);
        lcd.print(F("   "));
        //        delay(100);
    }
}

void doControl()
{
    if (tempSense.isConversionAvailable(sensorAddress))
    {
        currentTemp = tempSense.getTempC(sensorAddress);
        tempSense.requestTemperaturesByAddress(sensorAddress);
    }

    if (tuning)
    {
        /*if (aTune.Runtime())
          {
          finishAutoTune();
          }*/
    }
    else
    {
        tempPID.Compute();
    }
    onTime = PIDOutput;
}

void finishAutoTune()
{

}



void Off()
{
    //Save the state so we can return to the correct one when in the menus
    isRunning = false;
    tempPID.SetMode(MANUAL);
    PIDOutput = 0;

    digitalWrite(SSR, LOW);



    lcd.setCursor(0,0);
    lcd.print(F("SET:      "));
    lcd.setCursor(5,0);
    lcd.print(tempSet,1);

    lcd.setCursor(13,0);
    lcd.print(F("OFF"));

    lcd.setCursor(0,1);
    lcd.print(F("TMP:"));

    //Wait for an input and keep the temperature updated
    while(checkInputs() == 0)
    {
        tempSense.requestTemperaturesByAddress(sensorAddress);
        currentTemp = tempSense.getTempC(sensorAddress);

        lcd.setCursor(5,1);
        lcd.print(currentTemp,2);
        lcd.print(F("   "));
        //        delay(100);
    }

    //Something happened. Check what and change state
    if (lastEncoderPos != encoderPos)
    {
        //Temperature was changed, stay in off state and get delta
        tempSet += ((encoderPos - lastEncoderPos)/2.0);
        lastEncoderPos = encoderPos;
        //TODO: Save the new set temp to EEPROM
    }
    if (switchState == 1)
    {
        //Short press. Go to run state
        currentState = RUN;
        switchState = 0;

        //Set up the PID before changing state
        tempPID.SetMode(AUTOMATIC);
        windowStartTime = millis();

    }
    else if (switchState == 2)
    {
        //Long press. Go to config menu i.e timer
        currentState = TIMER_MENU;
        switchState = 0;
    }

}

void Run()
{
    isRunning = true;


    tempPID.SetTunings(Kp,Ki,Kd);

    lcd.setCursor(0,0);
    lcd.print(F("SET:      "));
    lcd.setCursor(5,0);
    lcd.print(tempSet,1);
    lcd.setCursor(13,0);

    lcd.print(F("ON"));

    lcd.setCursor(0,1);
    lcd.print(F("TMP:"));

    while(checkInputs() == 0)
    {
        doControl();

        //Print the current Temperature
        lcd.setCursor(5,1);
        lcd.print(currentTemp,2);
        lcd.print(F("   "));


        //Print the duty cycle of the heater
        float pct = map(PIDOutput, 0, windowSize, 0, 1000);
        lcd.setCursor(12,1);
        lcd.print(F("   "));
        lcd.setCursor(12,1);
        lcd.print(int(pct/10));
        lcd.print("%");



    }

    //Something happened. Check what and change state
    if (lastEncoderPos != encoderPos)
    {
        //Temperature was changed, modify setpoint accordingly
        tempSet += ((encoderPos - lastEncoderPos)/2.0);
        lastEncoderPos = encoderPos;
        //TODO: Save the new set temp to EEPROM
    }
    if (switchState == 1)
    {
        //Short press. Go to off state
        currentState = OFF;
        switchState = 0;
    }
    else if (switchState == 2)
    {
        //Long press. Go to config menu i.e timer
        currentState = TIMER_MENU;
        switchState = 0;
    }



}
void Timer()
{
    unsigned long timeRemaining;
    while(checkInputs() == 0)
    {
        //Print the arrows
        lcd.setCursor(0,0);
        lcd.write(byte(0));
        lcd.setCursor(15,0);
        lcd.write(byte(1));
        lcd.setCursor(5,0);

        lcd.print(F("TIMER"));

        if (timerIsOn)
        {
            //Get the remaining time in minutes
            timeRemaining = (millis() - timerSetTime)/60000;
            lcd.setCursor(3,1);
            lcd.print(F("ON: "));
            lcd.print(int(timeRemaining/60));
            lcd.print(":");
            lcd.print(timeRemaining % 60);
        }
        else
        {
            lcd.setCursor(5,1);
            lcd.print(F("OFF"));
        }

        delay(100);
    }


    if (switchState != 0)
    {
        //Short press. Go to set timer.
        currentState = TIMER_SET;
        switchState = 0;
    }

    if (lastEncoderPos != encoderPos)
    {
        if (encoderPos > lastEncoderPos)
            //Move right
            currentState = AUTOTUNE;
        else
            //Move left
            currentState = TUNE_MENU;

        lastEncoderPos = encoderPos;
    }

}

void Timer_Set()
{

    while(checkInputs() == 0)
    {
        //Print the arrows
        lcd.setCursor(0,0);
        lcd.write(byte(0));
        lcd.setCursor(15,0);
        lcd.write(byte(1));
        lcd.setCursor(5,0);

        lcd.setCursor(7,0);
        if (timerIsOn)
            lcd.print(F("ON"));
        else
            lcd.print(F("OFF"));

        delay(100);
    }

    if (lastEncoderPos != encoderPos)
    {
        timerIsOn = !timerIsOn;
        lastEncoderPos = encoderPos;
    }

    if (switchState != 0)
    {
        switchState = 0;

        lcd.clear();

        if (timerIsOn)
        {
            while(currentState == TIMER_SET)
            {
                lcd.setCursor(0,0);
                lcd.print(F("Time: "));

                //Work out time in 00:00 format

                lcd.print(int(timerSetTime/3600000));
                lcd.print(":");
                lcd.print((timerSetTime/60000) % 60);
                lcd.print("    ");
                delay(100);

                if (switchState != 0)
                {
                    switchState = 0;
                    //Change state
                    if (isRunning)
                        currentState = RUN;
                    else
                        currentState = OFF;
                }

                if (lastEncoderPos != encoderPos)
                {
                    timerSetTime += ((encoderPos - lastEncoderPos)*60000);
                    lastEncoderPos = encoderPos;
                }
            }
        }
        else 
        {
            if (isRunning)
                currentState = RUN;
            else
                currentState = OFF;
        }


    }
}

void Autotune()
{

    while(checkInputs() == 0)
    {
        //Print the arrows
        lcd.setCursor(0,0);
        lcd.write(byte(0));
        lcd.setCursor(15,0);
        lcd.write(byte(1));

        lcd.setCursor(4,0);
        lcd.print(F("AUTOTUNE!"));
        delay(100);
    }

    if (switchState != 0);
    {
        currentState = DO_AUTOTUNE;
        switchState = 0;
    }

    if (lastEncoderPos != encoderPos)
    {
        if (encoderPos > lastEncoderPos)
            //Move right
            currentState = TUNE_MENU;
        else
            //Move left
            currentState = TIMER_MENU;

        lastEncoderPos = encoderPos;
    }
}

void Do_Autotune()
{
    currentState = OFF;
}


void Tune()
{

    while(checkInputs() == 0)
    {
        //Print the arrows
        lcd.setCursor(0,0);
        lcd.write(byte(0));
        lcd.setCursor(15,0);
        lcd.write(byte(1));

        lcd.setCursor(3,0);
        lcd.print(F("MANUAL TUNE"));
        delay(100);
    }

    if (switchState != 0)
    {
        //Short press. Go to Kp
        currentState = TUNE_KP;
        switchState = 0;
    }

    if (lastEncoderPos != encoderPos)
    {
        if (encoderPos > lastEncoderPos)
            //Move right
            currentState = TIMER_MENU;
        else
            //Move left
            currentState = AUTOTUNE;

        lastEncoderPos = encoderPos;
    }

}

void Tune_Kp()
{

    while(checkInputs() == 0)
    {
        lcd.setCursor(0,0);
        lcd.print(F("Kp:       "));
        lcd.print(Kp);

        delay(100);
    }
    //Something happened. check what and change state
    if (lastEncoderPos != encoderPos)
    {
        //Kp was changed, modify setpoint accordingly
        Kp += (encoderPos - lastEncoderPos);
        lastEncoderPos = encoderPos;
    }
    if (switchState != 0)
    {
        //TODO: save kp
        //Short press. go to off state
        currentState = TUNE_KI;
        switchState = 0;
    }
}

void Tune_Ki()
{
    while(checkInputs() == 0)
    {
        lcd.setCursor(0,0);
        lcd.print(F("Ki:       "));
        lcd.print(Ki);

        delay(100);
    }
    //Something happened. Check what and change state
    if (lastEncoderPos != encoderPos)
    {
        //Kp was changed, modify setpoint accordingly
        Ki += (encoderPos - lastEncoderPos)/100.0;
        lastEncoderPos = encoderPos;
    }
    if (switchState != 0)
    {
        //TODO: Save Kp
        //Short press. Go to Kd
        currentState = TUNE_KD;
        switchState = 0;
    }

}

void Tune_Kd()
{
    while(checkInputs() == 0)
    {
        lcd.setCursor(0,0);
        lcd.print(F("Kd:       "));
        lcd.print(Kd);

        delay(100);
    }
    //Something happened. Check what and change state
    if (lastEncoderPos != encoderPos)
    {
        //Kp was changed, modify setpoint accordingly
        Kd += (encoderPos - lastEncoderPos)/100.0;
        lastEncoderPos = encoderPos;
    }
    if (switchState != 0)
    {
        //TODO: Save Kp
        //Short press. Go to previous state
        if (isRunning)
            currentState = RUN;
        else
            currentState = OFF;
        switchState = 0;
    }

}


byte checkInputs()
{
    if (lastEncoderPos != encoderPos)
    {
        return 1;
    }
    else if (switchState != 0)
    {
        return 2;
    }

    else
        return 0;
}

void push_switch()
{
    //Debouncing could be a problem here.
    delay(1);
    //If we have a falling edge (button is pushed down)
    if(PCintPort::pinState == LOW)
    {
        //start a timer
        switchStartTime = millis();
    }
    //Else rising edge
    else if (PCintPort::pinState == HIGH)
    {
        //Check if we had a short or long press
        if ((millis()-switchStartTime) < 1000)
            switchState = 1;
        else
            switchState = 2;
    }
}
void doEncoderA(){
    delay(1);
    //If the saved states are the same, this interrupt was called first
    if (A_set == B_set)
    {
        //A was called first so we're going anti-clockwise
        encoderPos = encoderPos--;
    }
    A_set = !A_set;
}

// Interrupt on B changing state
void doEncoderB(){
    //Add a little debounce
    delay(1);
    //If the vars are the same, this interrupt was called first
    if (B_set == A_set)
    {
        //Going clockwise
        encoderPos = encoderPos++;
    }
    B_set = !B_set;
}

void DriveOutput()
{
    long now = millis();
    if (now - windowStartTime > windowSize)
    {
        windowStartTime += windowSize;
    }
    if ((onTime > 100)  && (onTime > (now - windowStartTime)))
    {
        digitalWrite(SSR,HIGH);
    }
    else
    {
        digitalWrite(SSR,LOW);    
    }
}

void LoadPIDParams()
{
    tempSet = 55.5;
    Kp = 1300;
    Ki = 1;
    Kd = 0.1;
}
