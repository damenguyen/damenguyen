#include <dummy.h>

/*Damian Nguyen
  Bathroom Scale
  The Home Physician
  Team 7
  4 - 21 -21*/

/*Display_SCL -- GPIO1
  Display_SDA -- GPIO5
  Scale Input -- GPIO36
  Rotary Encoder OutputA -- GPIO34
  Rotary Encoder OutputB -- GPIO35
  Button -- GPIO39*/

  #include <LiquidCrystal.h>
  #include "Arduino.h"
  
const int resolution = 4095; //12 bit ADC
const int systemVoltage = 3.3; //3.3V Max
const int ScaleOutputPin = 36; // GPIO pin 36 is ADC1
const int PushButton = 33; //pinNumber for button is 39
const int OutputA = 34; // rotaryEncoder outputA is pin 34
const int OutputB = 35; // rotaryEncoder outputB is pin 35
int counter = 50; //Height in cm or in
int aPosition;
int aLastPosition;
float voltageMetric;
float loadcells; //Unit for Loacell reading
float weightMetric; //Metric weight value
float weightImperial; //Imperial weight value
const float sensitivity = .001; //1.0 mV / V
const float VoltageIn = 3.3; //Voltage is from ESP32 Rail
float MetricBMI;
float ImperialBMI;
int ADCMetric; //digital reading for Metric
int ADCImperial; //digital reading for Imperial
float AnalogVoltageMetric; 
float AnalogVoltageImperial;
char unitSelection;
int buttoncounter = 0; 
char unit;
int state = 0;
int previous;
int weightprompt;
int kg = 100;

int lb = 220;

char answer;

LiquidCrystal lcd(2, 4, 5, 15, 18, 19); //choosing pins for LCD

void setup() 
{
  lcd.begin(16,2);
  pinMode(PushButton, INPUT);
  pinMode(OutputA, INPUT);
  pinMode(OutputB, INPUT);
  aLastPosition = digitalRead(OutputA);
  Serial.begin(115200);
  Serial.println ( "do you want to enter weight? y/n: ");
  
  answer = Serial.read();
}

void loop() {
  int ButtonStatus;
  //this is to check the unit of preference
  ButtonStatus = digitalRead(PushButton);
  delay(150);

    if (ButtonStatus == 1)
    {
      unitDetermine();
    }

    previous = ButtonStatus; 

            aPosition = digitalRead(OutputA);
            if ( aPosition != aLastPosition) 
            {
              getHeight();
            }
          
   // will begin checking for weight

if (digitalRead(ScaleOutputPin) != 0 )
  {
            if (unit == 'imperial')
            {
              getImperial();
            }
          
            else if (unit == 'metric')
            {
              getMetric();
            }

  }

 // using this for testing bmi

 if (answer == 'y')
 {
    delay(5000);
    Serial.print("enter your weight sire: "); 
    weightprompt = Serial.read();
    Serial.print(weightprompt);
    delay(5000);
    Serial.print(unit);
    
    if (unit == 'imperial')
    {
      promptimperial();
    }

    else if (unit == 'metric')
    {
      promptmetric();
    }

 }
}

void promptimperial()
{
  float denominator = pow (counter, 2);
  ImperialBMI = (703 * lb) / denominator;
  Serial.println(ImperialBMI);
}

void promptmetric()
{
  float denominator = pow (counter * .01, 2);
  MetricBMI = kg / denominator;
  Serial.println(MetricBMI);
}

void getMetric()
{
  ADCMetric = analogRead(ScaleOutputPin);
  AnalogVoltageMetric = ADCMetric / (resolution * systemVoltage);
  loadcells = 200 * AnalogVoltageMetric; 
  weightMetric = loadcells/ (sensitivity * VoltageIn);
  
            float denominator = pow (counter * .01, 2);
            MetricBMI = weightMetric / denominator; 
  
  Serial.print(weightMetric);
  Serial.print(MetricBMI);
  lcd.print(weightMetric);
  lcd.setCursor(2,0); 
  lcd.print(MetricBMI);
  delay(500);
  lcd.clear();
  delay(500);
}

void getImperial()
{
  // reading pin, taking ADC to voltage, then 
  ADCImperial = analogRead(ScaleOutputPin);
  AnalogVoltageImperial = ADCImperial / (resolution * systemVoltage);
  loadcells = 440 * AnalogVoltageImperial;
  weightImperial = loadcells/ (sensitivity * VoltageIn);
  
            float denominator = pow (counter, 2);
            ImperialBMI = weightImperial* 703/ denominator;
 
  Serial.print(weightImperial);
  Serial.print(ImperialBMI);
  lcd.print(weightImperial);
  lcd.setCursor(2,0); 
  lcd.print(ImperialBMI);
  delay(500);
  lcd.clear();
  delay(500);
  
}

void unitDetermine()
{
  if(state == 1)
  {
    state = 0;
    unit = 'metric';
    Serial.print("metric");
    promptmetric();
  }
  else if(state == 0)
  {
    state = 1;
    unit = 'imperial';
    Serial.print("imperial");
    promptimperial();
  }
}

void getHeight()
{    
    delay(500); 
            if(digitalRead(OutputB) != aPosition)
            {
              counter ++;
            }
            else
            {
              counter --;
            }
       
  Serial.print(counter);
  if (state == 0)
  {
  Serial.print("cm");
  Serial.println();
  }
  else if (state == 1)
  {
  Serial.print("in");
  Serial.println();
  }
  lcd.print(counter);
  lcd.print(unitSelection);
 }
