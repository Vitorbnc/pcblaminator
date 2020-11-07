#include <Arduino.h>
#include "Nokia_5110.h"
#include <math.h>
#include<EEPROM.h>

#define LCD_BACKLIGHT PC13
#define BTN PA15

#define ENCODER_CLK PA11
#define ENCODER_DT PA12

#define THERMISTOR PA0
#define SS_RELAY PA2

#define ADC_BITS 12

bool serialEnabled = false;

#define SAMPLING_PERIOD 100

unsigned long lastSampleTime  = 0;

unsigned long currentSample = 0;


// Constants for Steinhart-Hart Equation, got from SciDaVis fit
const float STEIN_A = 0.00436953656192422;
const float STEIN_B = 0.000942199116744498;
const float STEIN_C = 0.000101710499635659;

const float LIN_B = 62.4688362492716;
const float LIN_A = -0.140370799828433;

const int THERMISTOR_DIVIDER_R1 = 10000;//9979; //Ohms 
const float THERMISTOR_DIVIDER_VIN = 3.28;


//Nokia_5110 lcd = Nokia_5110(RST, CE, DC, DIN, CLK);
Nokia_5110 lcd = Nokia_5110(PB3,PB5,PB4,PA7,PA5);


bool lastEncCLK = true;

int encoderPos = 0;

int setpoint = 0;
float currentTemp = 0;
const int tempThreshold = 1;
const int firstTempThreshold = 5;
bool firstHeating = true;

void encoderISR(){
  if(digitalRead(ENCODER_DT))
    encoderPos++;
  else
    encoderPos--;
}

int resistanceFromADC(int adc_val){
  float vout = THERMISTOR_DIVIDER_VIN/4095*adc_val;
  //return vout;
  return THERMISTOR_DIVIDER_R1/(THERMISTOR_DIVIDER_VIN/vout -1.0);
}
float temperatureFromResistance(float R){
  //return R;
  R = R/1000.0;
    if(R<125.0)
      return 1/(STEIN_A+STEIN_B*log(R)+STEIN_C*pow(log(R),3));
    else
      return LIN_A*R + LIN_B;
}

float readTemperature(){
  int mean = 0;
  for(int i=0;i<5;i++)
    mean+=analogRead(THERMISTOR);

  int temp = 10*temperatureFromResistance(resistanceFromADC(mean/5));
  currentTemp = temp/10.0;
  return currentTemp;
}

/*
uint16_t encoderState = 0;
void encoderFilter(){
  //https://www.best-microcontroller-projects.com/rotary-encoder.html#Digital_Debounce_Filter
  // Debounce filter that checks for a FALLING edge
  //Encoder must be 1 then stay 0 for 12 cycles, giving 1111 0000 0000 0000
  //Operation OR with 0xe000 keeps 16-bit variable 1s on leftmost positions: 0xe000 = 1110 0000 0000 0000
  encoderState = (encoderState<<1) | digitalRead(ENCODER_CLK) | 0xe00;
  if(encoderState == 0xf00){
    encoderState = 0x00;
    //Falling edge with DATA high means clockwise motion 
    if(digitalRead(ENCODER_DT))
        encoderPos++;
      else 
        encoderPos--;
  }
}
*/

void readEncoder(){
    bool clk = digitalRead(ENCODER_CLK);
  if(clk and !lastEncCLK){ //rising edge
    if(digitalRead(ENCODER_DT)) //if true, got CCW motion
      encoderPos--;
    else
      encoderPos++;
  }
      lastEncCLK = clk;
      if(encoderPos<0)
        encoderPos=0;
      if(encoderPos>250)
        encoderPos = 250;
}

void splashScreen(){
  lcd.setCursor(0,0);
  lcd.print("LAMINADORA");
  lcd.setCursor(20,2);
  lcd.print("BULLDOZER");
  lcd.setCursor(0,4);
  lcd.print("por Vitor");
  delay(3000);
  lcd.clear();
}

void setup() {
  Serial.begin(115200);
  Serial.print("Sampling every ");
  Serial.print(SAMPLING_PERIOD);
  Serial.println(" ms");
  Serial.println("Number , Temperature");
  
  
  analogReadResolution(ADC_BITS);  

  pinMode(LCD_BACKLIGHT, OUTPUT);

  pinMode(BTN, INPUT);
  pinMode(ENCODER_CLK,INPUT);
  pinMode(ENCODER_DT,INPUT);
  //attachInterrupt(ENCODER_CLK,encoderISR,FALLING);

  pinMode(THERMISTOR,INPUT_ANALOG);
  pinMode(SS_RELAY,OUTPUT);

  digitalWrite(LCD_BACKLIGHT,LOW);
  lcd.setContrast(50);
  splashScreen();
  lcd.setCursor(0,0);
  lcd.println("Atual:");
  lcd.println("Set:");
  lcd.println("Entrada:");
  digitalWrite(SS_RELAY,HIGH);
  //setpoint = readTemperature();
  encoderPos = EEPROM.read(0);
  setpoint = encoderPos;
  if(setpoint-readTemperature()<firstTempThreshold)
    firstHeating = false;
}



void loop() {
  readEncoder();
  if(!digitalRead(BTN)){
    setpoint = encoderPos;
    EEPROM.update(0,encoderPos);
  }

  lcd.setCursor(50,0);
  
  float temp = readTemperature();
  lcd.print(temp);
  if(temp<100) lcd.print("  ");
  lcd.setCursor(50,1);
  lcd.print(setpoint);
  lcd.print("   ");
  lcd.setCursor(50,2);
  lcd.print(encoderPos);
  lcd.print("   ");

if(Serial.available())
  if((char)Serial.read()=='s')
    serialEnabled = !serialEnabled;
    
  if(serialEnabled and millis()-lastSampleTime>=SAMPLING_PERIOD){
    Serial.print(currentSample);
    Serial.print(",");
    Serial.println(temp);
    lastSampleTime=millis();
    if(currentSample==100)
      digitalWrite(SS_RELAY,LOW);
    currentSample++;
    
  }
  
//  int error = setpoint-int(floorf(currentTemp));
//  if(firstHeating){
//  
//    if(error>firstTempThreshold)
//      digitalWrite(SS_RELAY,LOW);
//    else {
//      digitalWrite(SS_RELAY,HIGH);
//      firstHeating = false;
//      delay(5000);
//  
//    }
//  }
//  else if(error>tempThreshold)
//    digitalWrite(SS_RELAY,LOW);
//  else
//    digitalWrite(SS_RELAY,HIGH);    
}