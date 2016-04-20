
#define TRIGPIN        11
#define ECHOPIN        12
#define DS3231_I2C_ADDRESS 0x68
#include <Servo.h>
#include "Wire.h"
#include <SoftwareSerial.h>

int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3
int pin11 = 11;
//-------------

//variables for storing values
float tempC = 0;
float tempF = 0;

//Create an instance of the SHT1X sensor
//SHT1x sht15(A4, A5);//Data, SCK

//delacre output pins for powering the sensor
int power = A3;
int gnd = A2;

int SHT_clockPin = 5;  // pin used for clock
int SHT_dataPin  = 4;  // pin used for data

//Speaker Variables
unsigned int frequency = 5000;
int speakerPin = 7; // can be whatever we want it to be
int duration = 500;

int x0, x1, y0, y1, z0, z1, x, y, z;
byte second, second2, minute, minute2, hour, dayOfWeek, dayOfMonth, month, year;
int buzzer = 3; //use for speakers?
//int touch = 2;
//Ping ping = Ping(TRIGPIN, ECHOPIN);

bool started = false;
bool finished = false;
boolean turnedAwayBegan = true;
bool toreturn = true;
int analogPin = 6;
int servoPin = 9;
int servoPin2 = 10;

Servo servo;
Servo servo2;

int angle = 0;   // servo position in degrees
int angle2 = 45;

//------------------------

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup()
{
  Serial.begin(9600);  // Begin the serial monitor at 9600bps
  pinMode(6, OUTPUT);
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
  
  initSht(SHT_dataPin, SHT_clockPin);
  
  servo.attach(servoPin);
  servo2.attach(servoPin2);
  pinMode(pin11, OUTPUT);

  // put your setup code here, to run once:
  pinMode(buzzer, HIGH); //use for speakers?
  //  pinMode(touch, LOW);
  Wire.begin();
  Wire.beginTransmission(0x1D);
  Wire.write(0x2D);
  Wire.write(0x08);
  Wire.endTransmission();
  /*  pinMode(power, OUTPUT);
    pinMode(gnd, OUTPUT);
    //  pinMode(4, OUTPUT);
    //  digitalWrite(4,HIGH);*/
}

boolean lightOn = false;



void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}
void startTimer() {
      readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,&year);
      
      readDS3231time(&second2, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,&year);
      Serial.print(second2-second);
      Serial.print(" Seconds");
}

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}


void loop()
{
  
  if (bluetooth.available()) // If the bluetooth sent any characters
  {
  
    int r = bluetooth.read();
    Serial.print((char)r);
    // Send any characters the bluetooth prints to the serial monitor
    if (r == 107) { // k character
      rotateTowardsLight();
      digitalWrite(6, HIGH);
      lightOn = true;
    }
    else if (r == 108) {
      digitalWrite(6, LOW);
      lightOn = false;
    }
    else if (r == 106) {
      ///**/rotateTowardsLight();
      rotateAwayFromLight();
    }

  }
  if (Serial.available())
  {
    bluetooth.print((char)Serial.read());
  }
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  Serial.println(second);
  if (lightOn) {
    rotateMarshmallow();
    bluetooth.println(getShtTemperature()*9/5+32);
    rotateAwayFromLight();
    digitalWrite(6, LOW);
    Serial.print("Temperature: ");
    delay(7000);
    Serial.println(getShtTemperature()*9/5+32);
    //detect temperature
    bluetooth.println(getShtTemperature()*9/5+32);
    delay(7000);
    bluetooth.println(getShtTemperature()*9/5+32);
    lightOn = false;
    toasted();
  }
  //printShtTemperatureCelsius();
}

void rotateAwayFromLight() {
  for (angle2 = 0; angle2 <= 210; angle2++) {
    servo2.write(angle2);
    delay(15);
  }
  delay(50);
//  servo2.write(115);
}

void rotateTowardsLight() {
  for (angle2 = 210; angle2 >= 0; angle2--) {
    servo2.write(angle2);
    delay(15);
  }
  //  servo2.write(115);
}
void rotateMarshmallow() {
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,&year);
  Serial.println(second);
  do {
  for (angle = 0; angle <= 180; angle++)
  {
    servo.write(angle);
    delay(50);
  }
  // now scan back from 180 to 0 degrees
  for (angle = 180; angle > 0; angle--)
  {
    servo.write(angle);
    delay(50);
  }
  readDS3231time(&second2, &minute2, &hour, &dayOfWeek, &dayOfMonth, &month,&year);
//  Serial.println(minute);
//  Serial.println(minute2-minute);
  Serial.println((double(minute2) + (double(second2) / 60.0)) - (double(minute) + (double(second) / 60.0)));
  } while((double(minute2) + (double(second2) / 60.0)) - (double(minute) + (double(second) / 60.0)) < 1.5);
 // displayTime();
  Wire.endTransmission();
//  Serial.print(minute2-minute);
//  Serial.print("minutes and ");
//  Serial.print(second2-second);
//  Serial.print("seconds \n");
}

void displayTime()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,
  &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(" Day of week: ");
  switch(dayOfWeek){
  case 1:
    Serial.println("Sunday");
    break;
  case 2:
    Serial.println("Monday");
    break;
  case 3:
    Serial.println("Tuesday");
    break;
  case 4:
    Serial.println("Wednesday");
    break;
  case 5:
    Serial.println("Thursday");
    break;
  case 6:
    Serial.println("Friday");
    break;
  case 7:
    Serial.println("Saturday");
    break;
  }
}


void initSht(int dataPin, int clockPin){
  SHT_clockPin = clockPin;
  SHT_dataPin = dataPin;
}

/*
 * Print the temperature in Celsius
 */
void printShtTemperatureCelsius(){
  //these can take a bit to get the values (100ms or so)
  float temperature = 0;
  temperature = getShtTemperature();
  
//  Serial.print("Temperature: ");
//  Serial.print(temperature);
//  Serial.println(" C");
}

/*
 * Print the temperature in Fahrenheit
 */
void printShtTemperatureFahrenheit(){
  //these can take a bit to get the values (100ms or so)
  float temperature = 0;
  temperature = getShtTemperature();
  temperature = temperature + 26 * 1.8;

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" F");
}

/*
 * Print the humidity percentage
 */
void printShtHumidity(){
  float humidity = 0;
  getShtHumidity(&humidity);
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
}

/*
 * Retrieve the temperature in Celsius and place in the
 * passed variable
 */
float getShtTemperature(){
  //Return Temperature in Celsius
  SHT_sendCommand(B00000011, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);

  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  
  float temp = (float)val * 0.01 - 40; //convert to celsius
  return temp;
}

/*
 * Retrieve the humidity and place in the
 * passed variable
 */
void getShtHumidity(float* humid){
  //Return  Relative Humidity
  SHT_sendCommand(B00000101, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);
  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  
  *humid = -4.0 + 0.0405 * val + -0.0000028 * val * val; 
}

/*
 * Bitbanging for issuing a command to the SHT15
 */
void SHT_sendCommand(int command, int dataPin, int clockPin){
  // send a command to the SHTx sensor
  // transmission start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);

  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // verify we get the right ACK
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);

  if (digitalRead(dataPin)) Serial.println("ACK error 0");
  digitalWrite(clockPin, LOW);
  if (!digitalRead(dataPin)) Serial.println("ACK error 1");
}


/*
 * Waiting for ACK from the SHT15
 */
void SHT_waitForResult(int dataPin){
  // wait for the SHTx answer
  pinMode(dataPin, INPUT);

  int ack; //acknowledgement

  //need to wait up to 2 seconds for the value
  for (int i = 0; i < 1000; ++i){
    delay(2);
    ack = digitalRead(dataPin);
    if (ack == LOW) break;
  }

  if (ack == HIGH) Serial.println("ACK error 2");
}

/*
 * Bitbanging for reading data from the SHT15
 */
int SHT_getData(int dataPin, int clockPin){
  // get data from the SHTx sensor

  // get the MSB (most significant bits)
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  byte MSB = shiftIn(dataPin, clockPin, MSBFIRST);

  // send the required ACK
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  // get the LSB (less significant bits)
  pinMode(dataPin, INPUT);
  byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
  return ((MSB << 8) | LSB); //combine bits
}

/*
 * CRC check
 */
void SHT_skipCrc(int dataPin, int clockPin){
  // skip CRC data from the SHTx sensor
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}

void toasted() {
  tone(speakerPin, frequency, duration);
}

