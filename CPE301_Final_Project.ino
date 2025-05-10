// Author: Victor Lopez-Estrada
// Subject: CPE 301
// Section: 1001
// Purpose: This program is designed to simulate a DIY swamp cooler using
// an Arduino ATMega 2560 and components. The goal of this project is to 
// demonstrate a mix of low-level and high-level understanding of embedded 
// system programming. 

#include <LiquidCrystal.h>
#include <dht.h>
#include <RTClib.h>
#include <Stepper.h>
#define RDA 0x80
#define TBE 0x20
#define DHT11_PIN 13
#define WATER_THRESHOLD 75
#define TEMP_THRESHOLD 30
// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;
// ADC Pointers
volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;
// GPIO Pointers
volatile unsigned char *portDDRA = (unsigned char *)0x21;
volatile unsigned char *portA = (unsigned char *)0x22;
volatile unsigned char *pinB = (unsigned char *)0x23;
volatile unsigned char *portDDRB = (unsigned char *)0x24;
volatile unsigned char *portB = (unsigned char *)0x25;
volatile unsigned char *portDDRE = (unsigned char *)0x2D;
volatile unsigned char *portE = (unsigned char *)0x2E;
volatile unsigned char *portDDRG = (unsigned char *)0x33;
volatile unsigned char *portG = (unsigned char *)0x34;
volatile unsigned char *portDDRH = (unsigned char *)0x101;
volatile unsigned char *portH = (unsigned char *)0x102;
volatile unsigned char *portDDRL = (unsigned char *)0x10A;
volatile unsigned char *portL = (unsigned char *)0x10B;

const int RS = 5, EN = 4, D4 = 39, D5 = 37, D6 = 35, D7 = 33;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
volatile unsigned int waterLevel = 0;
volatile unsigned int angleValue = 165;
volatile unsigned long lastInterruptTimeIdle = 0;
volatile unsigned long lastInterruptTimeDisabled = 0;
const int stepsPerRevolution = 4038;
Stepper myStepper = Stepper(stepsPerRevolution, 28, 24, 26, 22);
dht DHT;
RTC_DS1307 rtc;
int Pval = 0;
int potVal = 0;
volatile int ISRDflag = 0;
volatile int ISRIflag = 0;
enum SystemState {DISABLED, IDLE, RUNNING, ERROR};
const char* stateStrings [4] = {"DISABLED", "IDLE", "RUNNING", "ERROR"};
volatile SystemState currentState = DISABLED;

void setup() {
  // Start the UART
  U0Init(9600);
  lcd.begin(16, 2);
  adc_init();
  rtc.begin();

// #ifndef ESP8266
//   while (!Serial); // wait for serial port to connect. Needed for native USB
// #endif

  // if (! rtc.begin()) {
  //   Serial.println("Couldn't find RTC");
  //   Serial.flush();
  //   while (1) delay(10);
  // }
  // if (! rtc.isrunning()) {
  //   Serial.println("RTC is NOT running, let's set the time!");
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // }
  
  attachInterrupt(digitalPinToInterrupt(3), idleISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(2), disabledISR, LOW);

  *portDDRH |= 0x40;  // pin 9 -> OUTPUT
  *portDDRH |= 0x20;  // pin 8 -> OUTPUT
  *portDDRH |= 0x10;  // pin 7 -> OUTPUT
  *portDDRH |= 0x08;  // pin 6 -> OUTPUT

  *portDDRB |= 0x40;  // pin 12 -> OUTPUT

  *portDDRL |= 0x20;  // pin 44 -> OUTPUT
  *portDDRL |= 0x08;  // pin 46 -> OUTPUT
  *portDDRL |= 0x02;  // pin 48 -> OUTPUT

  *portDDRB &= 0xFD;  // pin 52 -> INPUT
  *portB |= 0x02;     // pin 52 PULLUP enable
  *portDDRE &= 0xDF;  // pin 3 -> INPUT
  *portE |= 0x20;     // pin 3 PULLUP enable
  *portDDRE &= 0xEF;  // pin 1 -> INPUT
  *portE |= 0x10;     // pin 1 PULLUP enable

  myStepper.setSpeed(1000);
  *portH |= 0x10; // Writes high to pin 7 (disabled light)
}

void loop() {
  unsigned long currentMillis = millis();

  // RTC for timestamping
  DateTime now = rtc.now();

  // Read DHT11 sensor
  int tempChk = DHT.read11(DHT11_PIN);
  // Read water level from ADC channel 0
  waterLevel = adc_read(0);
  angleValue = adc_read(2);
  if(angleValue < 150) {
    myStepper.step(-stepsPerRevolution);
    angleChangeMsg(now);
  }
  else if(angleValue > 450) {
    myStepper.step(stepsPerRevolution);
    angleChangeMsg(now);
  }

  // currentState check
  switch(currentState) {
    case IDLE:
      if(ISRIflag == 1){
        printMonitor();
        idleState(now);
        ISRIflag = 0;
      }
      if(DHT.temperature >= TEMP_THRESHOLD){
        currentState = RUNNING;
        printMonitor();
        runningState(now);
      }
      if(waterLevel < WATER_THRESHOLD){
        currentState = ERROR;
        errorMsg();
        errorState(now);
      }
      break;

    case RUNNING:
      if(DHT.temperature < TEMP_THRESHOLD) {
        currentState = IDLE;
        printMonitor();
        idleState(now);
      }
      if(waterLevel < WATER_THRESHOLD){
        currentState = ERROR;
        errorMsg();
        errorState(now);
      }
      break;

    case ERROR:
      if(waterLevel >= WATER_THRESHOLD && !(*pinB & 0x02)){
        currentState = IDLE;
        printMonitor();
        idleState(now);
      }
      break;

    case DISABLED:
      if(ISRDflag == 1){
        lcd.clear();
        disabledState(now);
        ISRDflag = 0;
      }
      break;
  }
}

void idleISR() {
  unsigned long currentTime = millis();
  if(currentTime - lastInterruptTimeIdle > 100){  // 100 ms debounce time
    currentState = IDLE;
    ISRIflag = 1;
    lastInterruptTimeIdle = currentTime;
  }
}

void disabledISR() {
  unsigned long currentTime = millis();
  if(currentTime - lastInterruptTimeDisabled > 100){  // 100 ms debounce time
    currentState = DISABLED;
    ISRDflag = 1;
    lastInterruptTimeDisabled = currentTime;
  }
}

void disabledState(DateTime now) {
  stateChange(now);
  // Turn off fan outputs
  *portL |= 0x08;
  *portL &= ~(0x02);
  analogWrite(12, 0);
  *portH |= 0x10; // Write HIGH
  *portH &= ~(0x40); // Write LOW
  *portH &= ~(0x20); // Write LOW
  *portH &= ~(0x08); // Write LOW
}

void idleState(DateTime now) {
  stateChange(now);
  // Turn off fan outputs
  *portL |= 0x08;
  *portL &= ~(0x02);        
  analogWrite(12, 0);       
  currentState = IDLE;
  *portH |= 0x08; // Write HIGH
  *portH &= ~(0x40); // Write LOW
  *portH &= ~(0x20); // Write LOW
  *portH &= ~(0x10); // Write LOW
}

void errorState(DateTime now) {
  stateChange(now);
  // Turn off fan outputs
  *portL &= ~(0x02);
  *portL |= 0x08;        
  analogWrite(12, 0);       
  *portH |= 0x40; // Write HIGH
  *portH &= ~(0x08); // Write LOW
  *portH &= ~(0x20); // Write LOW
  *portH &= ~(0x10); // Write LOW
}

void runningState(DateTime now) {
  stateChange(now);
  *portH |= 0x20; // Write HIGH
  *portH &= ~(0x40); // Write LOW
  *portH &= ~(0x08); // Write LOW
  *portH &= ~(0x10); // Write LOW

  // Run fan motor
  *portL |= 0x02;
  *portL &= ~(0x08);
  analogWrite(12, 255);
}

void printMonitor() {
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
  lcd.print("%");
}

void errorMsg() {
  lcd.clear();
  lcd.setCursor(3, 0); 
  lcd.print("WARNING!");
  lcd.setCursor(1, 1);
  lcd.print("WATER LOW: ");
  lcd.print(waterLevel);
}

void angleChangeMsg(DateTime now) {
  // Print hour
  putChar('0' + now.hour() / 10);    // Tens place
  putChar('0' + now.hour() % 10);    // Ones place
  putChar(':');

  // Print minute
  putChar('0' + now.minute() / 10);
  putChar('0' + now.minute() % 10);
  putChar(':');

  // Print second
  putChar('0' + now.second() / 10);
  putChar('0' + now.second() % 10);

  // Add space and angle message
  putChar(' ');
  printString("Fan angle changed");
  putChar('\n');
}

void stateChange(DateTime now) {
  // Print hour
  putChar('0' + now.hour() / 10);    // Tens place
  putChar('0' + now.hour() % 10);    // Ones place
  putChar(':');

  // Print minute
  putChar('0' + now.minute() / 10);
  putChar('0' + now.minute() % 10);
  putChar(':');

  // Print second
  putChar('0' + now.second() / 10);
  putChar('0' + now.second() % 10);

  // Add space and state message
  putChar(' ');
  printString("System ");
  printString(stateStrings[currentState]);
  putChar('\n');
}

void printString(const char* str) {
  while (*str != '\0') {
    putChar(*str);
    str++;
  }
}

void U0Init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

unsigned char kbhit() {
  return (*myUCSR0A & RDA) ? 1 : 0;
}

unsigned char getChar() {
  while (!kbhit());
  return *myUDR0;
}

void putChar(unsigned char U0pdata) {
  while (!(*myUCSR0A & TBE));
  *myUDR0 = U0pdata;
}

void adc_init() {
  // setup the A register
  // set bit 7 to 1 to enable the ADC
  *my_ADCSRA |= 0b10000000;
  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b10111111;
  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11011111;
  // clear bit 0-2 to 0 to set prescaler selection
  *my_ADCSRA &= 0b11111000;
  // set prescaler to 128
  *my_ADCSRA |= 0b00000111; 
  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111;
  // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0b11111000;
  // setup the MUX Register
  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= 0b01111111;
  // set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0b01000000;
  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;
  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0b11100000;
}

unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= 0b11011111;
  // set the channel selection bits for channel 0
  *my_ADMUX = (*my_ADMUX & 0b11100000) | (adc_channel_num & 0x07);
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  unsigned int val = (*my_ADC_DATA & 0x03FF);
  return val;
}