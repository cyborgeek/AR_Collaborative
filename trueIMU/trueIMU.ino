/* 
 *  Program created By Jesus Adrian Gutierrez 
 *  along with help from Ryan Philcox
 *  Date July 13th, 2019
 *  
 *  Rehabilitaion Institude, Loma Linda University
 * 
 *  This program uses adalogger feather M0, multiple IMUs(BNo055), and an OLED screen.
 *  The purpose is to record intertial data from IMUs and save the data onto  
 *  SD memory card while simultanio
 *  usly displaying data onto serial monitor
 *   
 *  Motion tracking wearable unit
 *  Unit is to be connected to a computer to communicate data readings back on serial
 *  monitor. Power source is the micro usb cable connected to a computer   
 */

 
// Libraries necessary to gather/interpret data from IMU BNo055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Libraries necessary to Display on OLED 164x32 I2C, setup
#include <Adafruit_SSD1306.h>
#define OLED_RESET    6 // Reset pin # used on OLED
Adafruit_SSD1306 display(OLED_RESET);

// Libraries necessary to write data onto SD memory card
#include <SPI.h>
#include <SD.h>

File logfile;
uint8_t i=0;
const int chipSelect = 4;   // Data pin that will be used to write onto SD card


  // Countdown timer Function to display on OLED display. Counting down from int "start"
void CountDown(int start){
  int starter = start;
  
  for (int i = starter; i > 0; i--){
    display.clearDisplay();
    display.setTextSize(4);
    display.setTextColor(WHITE);
    display.setCursor(55,0);
    display.println(String(i));
    display.display();
    delay(1000);
  }
  display.clearDisplay();
 }

    // The two BNo055 modules, bnoB has the ADR pin wired to 3.3v to change its I2C address
    // Both are wired: SCL to analog 5, SDA to analog 4, GRN to ground
    // IMU A is on Main board (5v Vin pin)
  Adafruit_BNO055 bnoA = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
    // IMU B is on smaller separate unit (3.3v ADR pin)
  Adafruit_BNO055 bnoB = Adafruit_BNO055(-1, BNO055_ADDRESS_B);


void setup() {         



  Serial.begin(9600);
  delay(3000); // give user time to open serial port before displaying data on it
  Serial.print("Initializing...");

    // Begin OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // // Address 0x3C for 128x32 SCREEN
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Initializing...");
  display.display();
  

    // Check components are plugged in (IMUA,IMUB,OLED)
  if(!bnoA.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, BNO055(A) not detected");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Ooops, BNO055(A) not detected");
      display.display();
      // don't do anything more:
      while(1);
  }
   bnoA.setExtCrystalUse(true);
   
  if(!bnoB.begin()) {
       Serial.print("Ooops, BNO055(B) not detected");
       display.clearDisplay();
       display.setTextSize(1);
       display.setTextColor(WHITE);
       display.setCursor(0,0);
       display.println("Ooops, BNO055(B) not detected");
       display.display();
       // don't do anything more:
       while(1);
  }
   bnoB.setExtCrystalUse(true);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Card failed, or not     present");
      display.display();
    // don't do anything more:
    while (1);
  }

    // Open file on SD card to start wrting data on it
  File logfile = SD.open("Data.txt", FILE_WRITE);
  
  Serial.println(" SD card initialized ... ");
  
   // Write header of file 
  Serial.println("        Main IMU A, Rotation Quat | Linear Acceleration (m/s^2) ||   Secondary IMU B, Rotation Quat | Linear Acceleration ");
  Serial.println(" A    qW     qX     qY     qZ     |      X       Y       Z      ||   B    qW     qX     qY     qZ     |      X       Y       Z");
  
  logfile.println("        Main IMU A, Rotation Quat | Linear Acceleration (m/s^2) ||   Secondary IMU B, Rotation Quat | Linear Acceleration ");
  logfile.println(" A    qW     qX     qY     qZ     |      X       Y       Z      ||   B    qW     qX     qY     qZ     |      X       Y       Z");
  logfile.println(" ");
  logfile.flush();
  

  int start = 5;
  CountDown(start);
  
}

  // end condition state variable
int endCondition = 1;
 
void loop() {    
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Recording Data");
  display.println(" ");
  display.println("   ... in progress");
  display.display();

  
  // Get Orientation and Accelerations from IMU's

    // Get a new sensor event for true orientation 
  sensors_event_t eventA; 
  sensors_event_t eventB; 
  bnoA.getEvent(&eventA);
  bnoB.getEvent(&eventB);
    // Retrivieing linear accelerations vectors (m/s^2)
  imu::Vector<3> linaccA = bnoA.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> linaccB = bnoB.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // Retrieving Quaternion rotation for more accurate data manipulation
  imu::Quaternion quatA = bnoA.getQuat();
  imu::Quaternion quatB = bnoB.getQuat();

  
  // Display Orientation data onto serial monitor

      //IMU A orienatation(Euler Angle) & linear acceleration (m/s^2)
  Serial.print("  ");
  Serial.print(eventA.orientation.x, 3);
  Serial.print("  ");
  Serial.print(eventA.orientation.y, 3);
  Serial.print("  ");
  Serial.print(eventA.orientation.z, 3);  
  Serial.print("  |  ");
  Serial.print(linaccA.x(), 3);
  Serial.print("  ");
  Serial.print(linaccA.y(), 3);
  Serial.print("  ");
  Serial.print(linaccA.z(), 3);
      //IMU B orienatation(Euler Angle) & linear acceleration (m/s^2)
  Serial.print("  |  ");
  Serial.print(eventB.orientation.x, 3);
  Serial.print("  ");
  Serial.print(eventB.orientation.y, 3);
  Serial.print("  ");
  Serial.print(eventB.orientation.z, 3);  
  Serial.print("  |  ");
  Serial.print(linaccB.x(), 3);
  Serial.print("  ");
  Serial.print(linaccB.y(), 3);
  Serial.print("  ");
  Serial.print(linaccB.z(), 3);
  
  Serial.print("\n");

  File logfile = SD.open("Data.txt", FILE_WRITE);  

  
    // Write data onto SD card file
  logfile.print(String(quatA.w(),4));  
  logfile.print("  ");
  logfile.print(String(quatA.x(),4));   // Need to convert data to string data type for SD.print() 
  logfile.print("  ");
  logfile.print(String(quatA.y(), 4));
  logfile.print("  ");
  logfile.print(String(quatA.z(), 4));
  logfile.print("  ");
  logfile.print(String(linaccA.x(), 4));
  logfile.print("  ");
  logfile.print(String(linaccA.y(), 4));
  logfile.print("  ");
  logfile.print(String(linaccA.z(), 4));
  logfile.print("  ");
  
  logfile.flush();                      // Data will only be written onto sd card after flush function
  
  logfile.print(String(quatB.w(),4));  
  logfile.print("  ");
  logfile.print(String(quatB.x(),4));
  logfile.print("  ");
  logfile.print(String(quatB.y(), 4));
  logfile.print("  ");
  logfile.print(String(quatB.z(), 4));
  logfile.print("  ");
  logfile.print(String(linaccB.x(), 4));
  logfile.print("  ");
  logfile.print(String(linaccB.y(), 4));
  logfile.print("  ");
  logfile.println(String(linaccB.z(), 4));
  
  logfile.flush();


  // When end condition is met, close SD card stop recording data
  // End condition is met after certain number of of data point are recorded 
   
   if ( endCondition >= 200){
      //Close & store data
     logfile.close();
     
      // Display OLED
     display.clearDisplay();
     display.println(" Task completed ");
     display.println(" ");
     display.println("Data Recording Terminated ");
     display.display();
     
      // DONE Serial communication
     Serial.println("Task Completed");
     while(1){
      logfile.close();
      delay(1000);
     }
   }
  
  endCondition++;
  
  delay(200);    // Change delay time according to how frequent you need to record data points
  
}
