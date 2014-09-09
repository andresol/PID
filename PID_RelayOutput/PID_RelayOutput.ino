/********************************************************
 * 
 * 
 * Temperature PID for controlling Beer Fermenting
 *
 * 
 * 
 ********************************************************/
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <MenuBackend.h>
#include <stdio.h>
#include <string.h>

#define Relay 22
#define RELAY_ON 0
#define RELAY_OFF 1
#define TEMPERATUR_SENORS 1
#define ERRORVALUE -1000;
#define DEFAULT_SAMPLE_TIME 1000
#define INT_TIME 150
#define MAX_OUTPUT 5000
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
#define MENU_MODE 0
#define TEMP_MODE 1

// KEY VARIABLES
int lcd_key = btnNONE;
int lcd_key_prev = btnNONE;
unsigned long lcd_key_prev_pressed = 0;
unsigned long millisPressedKey = 0;
int buttonPressed = 0;
unsigned long debounceTime = 0;

//PROPERTIES
int EEPROM_ADDR = 0;
int doTempSerialLogging = 1;
unsigned long last = 0;
unsigned long now = 0;
float lastOutput[TEMPERATUR_SENORS] = {0};

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

double aggKp=6, aggKi=0.1, aggKd=0.5;
double consKp=1, consKi=0.05, consKd=0.1;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

int temperaturPins[TEMPERATUR_SENORS] = {24};
OneWire oneWires[TEMPERATUR_SENORS] = {OneWire (temperaturPins[0])};
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


unsigned long start;
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  start = 0;
  digitalWrite(Relay, RELAY_OFF);
  pinMode(Relay, OUTPUT); 
  //initialize the variables we're linked to
  Setpoint = 30;
  lastOutput[0] = 0;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, MAX_OUTPUT);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

/**
* Get a butten if it is pressed.
*/
int getButtonPressed() {
  lcd_key = read_LCD_buttons();  // read the buttons
  unsigned long now = millis();
  if (lcd_key == btnNONE) {
    return btnNONE;
  } else if (millisPressedKey == 0 && lcd_key != btnNONE) {
    millisPressedKey = now;
    return btnNONE;
  } else if (((millisPressedKey + 10) < now) && (debounceTime < now)) {
    int lcd_key_confirm = read_LCD_buttons();
    debounceTime = now + 300;
    millisPressedKey = 0;
    if (lcd_key == lcd_key_confirm) {
      //Serial.println(lcd_key);
      buttonPressed = 0;
      return lcd_key;
    } else {
      millisPressedKey = now;
      return btnNONE;
    }
  } else {
    return btnNONE;
  }
}
/**
* Read which button is pressed
*/
int read_LCD_buttons() {
  int adc_key_in = 0;
  adc_key_in = analogRead(0);
  // my buttons when read are centered at these valies: 0, 97, 254, 437, 639
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 150)  return btnUP;
  if (adc_key_in < 350)  return btnDOWN;
  if (adc_key_in < 450)  return btnLEFT;
  if (adc_key_in < 850)  return btnSELECT;
  return btnNONE;  // when all others fail, return this...
}

void doButtonAction() {
  int btn = btnNONE;
  btn = getButtonPressed();
  switch (value) {
    case btnRIGHT: {
        break;
      }
    case btnLEFT: {
       
      }
    case btnUP:
      {
       Setpoint = Setpoint + 0.5;
        break;
      }
    case btnDOWN:
      {
        Setpoint = Setpoint - 0.5;
        break;
      }
    case btnSELECT:
      {
       
        break;
      }
    case btnNONE:
      {
        break;
      }
  }
  lcd_key = btnNONE;
}

void loop() {
  doButtonAction();
  doPid();
  delay(2);
}

void doPID() {
  unsigned long now = millis();
  if ((last + DEFAULT_SAMPLE_TIME) < now) {
    
    String sensor = "Sensor1";
    Input = printTemp(oneWires[0], sensor);
    double gap = abs(Setpoint-Input); //distance away from setpoint
    if(gap < 3) {  //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else {
       //we're far from setpoint, use aggressive tuning parameters
       myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
   
    if (Output > 0.5 && (Output - lastOutput[0] > 0.25 || Output - lastOutput[0] < 0.25) {
       digitalWrite(Relay, HIGH);
    } else {
       digitalWrite(Relay, LOW);
    }
    lastOutput[0] = Output;
    lcd.setCursor(0, 0);
    printValuesOnLCD();
    last = millis();
  }
}

void printValuesOnLCD() {
    String printTxt = String("I:");
    char temp[6];
    if (Input < 100) { // One digit precicion if temp > 100
       dtostrf(Input, 1, 2, temp);
     } else {
       dtostrf(Input, 1, 1, temp);
     }
     printTxt = printTxt + temp;
     lcd.print(printTxt);
     lcd.setCursor(8, 0);
     printTxt = String("O:");
     if (Output < 100) { // One digit precicion if temp > 100
       dtostrf(Output, 1, 2, temp);
     } else {
       dtostrf(Output, 1, 1, temp);
     }
     printTxt = printTxt + temp;
     lcd.print(printTxt);
     lcd.setCursor(0, 1);
     printTxt = String("S:");
     if (Setpoint < 100) { // One digit precicion if temp > 100
       dtostrf(Setpoint, 1, 2, temp);
     } else {
       dtostrf(Setpoint, 1, 1, temp);
     }
     printTxt = printTxt + temp;
     lcd.print(printTxt);
}


float printTemp(OneWire ds, String& name) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  if ( !ds.search(addr)) {
    ds.reset_search();
    //delay(250);
    //Serial.println("ERROR");
    return ERRORVALUE;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    return ERRORVALUE;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
#if defined(DEBUG_TEMP)
      Serial.println("  Chip = DS18S20");  // or old DS1820
#endif
      type_s = 1;
      break;
    case 0x28:
#if defined(DEBUG_TEMP)
      Serial.println("  Chip = DS18B20");
#endif
      type_s = 0;
      break;
    case 0x22:
#if defined(DEBUG_TEMP)
      Serial.println("  Chip = DS1822");
#endif
      type_s = 0;
      break;
    default:
#if defined(DEBUG_TEMP)
      Serial.println("Device is not a DS18x20 family device.");
#endif
      return ERRORVALUE;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  return celsius;
}




