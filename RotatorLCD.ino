//Rotator6.ino - Mini Satellite-Antenna Rotator.
//Copyright (c) 2015-2018 Julie VK3FOWL and Joe VK3YSP
//Released under the GNU General Public License.
//For more information please visit http://www.sarcnet.org
//Submitted for publication in Amateur Radio magazine: December 2015
//First published in Amateur Radio magazine: May 2016
//Upgraded Mk2 version published in Amateur Radio magazine: October 2017
//Release history:
//Release 1: Original release
//Release 2: Added support for hamlib 3.0.1. Added debug mode.
//Release 3: Improved calibration and operation
//  Added a low pass filter to the sensor data to improve calibration.
//  Reinitialised the I2C bus and sensor prior to each read to avoid I2C lockups caused by power glitches
//  Added support for RS-422 operation
//Release 4: Improved calibration and operation
//  Fixed a bug reading the EEPROM calibration data on some versions
//  Made the SerialPort configurable to support both the Mk1 (USB) and Mk2 (RS422) rotators
//  Added a speaker output to help with the calibration process
//  Removed the overshoot inherent in the anti-windup algorithm
//  Initialised the sensor filters at start up
//  Added a pause command, as requested
//  Added a help menu, as requested
//  Clarified sensor axis definitions
//  Please note that Gpredict position feedback does not work with the USB version on Windows due to a handshaking bug
//  You can fix this by using Linux or the Serial1 port with a TTL to USB converter.
//Release 5: Changes to support the half-price Mk1b version
//  Added support for either the original LMD18200T or the cheaper L298N DC Motor H-Bridge Driver Boards
//  Added compiler option to set the driver board type and the pins used
//  Note: Only PWM pins 5, 6, 9 or 10 can be used for PWM motor drive output
//  Added support for either the original LSM303D or the cheaper LSM303DLHC 3D Accelerometer/Magnetometer
//  Replaced the passive piezo speaker with an active piezo buzzer to help with the calibration process (since there were not enough PWM outputs)
//Release 6: Changes to support the AC Motor triac controller
//  Included the non-blocking timer class to replace the delay() function
//  Changed #include <arduino.h> to #include <Arduino.h> for bug in Arduino IDE 1.8.8 in Linux

//Includes
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "timer.h"
#include "lsm.h"
#include "mot.h"
#include <LiquidCrystal_I2C.h>

//Constants
//User configuration section:
//Please uncomment only one of each of the following MotorTypes, SensorTypes and SerialPort types:
//const int MotorType = PWMDIR;     //Please uncomment this line for the LMD18200T DC motor driver.
//const int MotorType = FWDREV;       //Please uncomment this line for the L298N DC motor driver.
const int MotorType = ACMOTR;     //Please uncomment this line for the triac AC motor driver.
//const int SensorType = LSM303D;   //Please uncomment this line to use the LSM303D sensor.
const int SensorType = LSM303DLHC;  //Please uncomment this line to use the LSM303DLHC sensor.
#define SerialPort Serial           //Please uncomment this line to use the USB port.
//#define SerialPort Serial1        //Please uncomment this line to use the TTL port.
#define WINDUP_LIMIT 450            //Sets the total number of degrees azimuth rotation in any direction before resetting to zero
//Motor pins - Don't change
const int azFwdPin = 5;
const int azRevPin = 6;
const int elFwdPin = 9;
const int elRevPin = 10;
//Speaker pins
const int gndPin = 11;    //Makes a convenient ground pin adjacent to the speaker pin
const int spkPin = 12;    //Attach a piezo buzzer to this pin. It beeps when new calibration data arrives.
//Motor drive gains. These set the amount of motor drive close to the set point
const int azGain = 25;   //Azimuth motor gain
const int elGain = 25;   //Elevation motor gain
//Filter constants
const float azAlpha = 0.5; //Alpha value for AZ motor filter: Decrease to slow response time and reduce motor dither.
const float elAlpha = 0.5; //Alpha value for EL motor filter: Decrease to slow response time and reduce motor dither.
const float lsmAlpha = 0.05; //Alpha value for sensor filter: Decrease to slow response time and ease calibration process.
//LCD config
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


//Modes
enum Modes {tracking, monitoring, demonstrating, calibrating, debugging, pausing};    //Rotator controller modes

//Global variables
float az;               //Antenna azimuth
float el;               //Antenna elevation
String line;            //Command line
float azSet;            //Antenna azimuth set point
float elSet;            //Antenna elevation set point
float azLast;           //Last antenna azimuth reading
float elLast;           //Last antenna element reading
float azWindup;         //Antenna windup angle from startup azimuth position
float azOffset;         //Antenna azimuth offset for whole revolutions
bool windup;            //Antenna windup condition
float azSpeed;          //Antenna azimuth motor speed
float elSpeed;          //Antenna elevation motor speed
float azError;          //Antenna azimuth error
float elError;          //Antenna elevation error
float azInc;            //AZ increment for demo mode
float elInc;            //EL increment for demo mode
Modes mode;             //Rotator mode

//Objects

//Motor driver object: Mot xxMot(Driver-Type, Filter-Alpha, Gain, Fwd-Pin, Rev/Dir-Pin)
Mot azMot(MotorType, azAlpha, azGain, azFwdPin, azRevPin); //AZ motor driver object
Mot elMot(MotorType, elAlpha, elGain, elFwdPin, elRevPin); //EL motor driver object

//LSM sensor object: Lsm lsm(Sensor-Type, Filter-Alpha)
Lsm lsm(SensorType,lsmAlpha);

//Non-blocking Timer object
Timer t1(100);

//Functions
void reset(bool getCal) {
  //Reset the rotator, initialize its variables and optionally get the stored calibration
  azSet = 0.0;
  elSet = 0.0;
  line = "";
  azLast = 0.0;
  elLast = 0.0;
  azWindup = 0.0;
  azOffset = 0.0;
  azSpeed = 0.0;
  elSpeed = 0.0;
  mode = tracking;
  windup = false;
  if (getCal) restore();
  azError = 0.0;
  elError = 0.0;
  azInc = 0.05;
  elInc = 0.05;
  t1.reset(100);
  printCal();
  lsm.calStart(); //Reset the axis calibration objects
}

float diffAngle(float a, float b) {
  //Calculate the acute angle between two angles in -180..180 degree format
  float diff = a - b;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return diff;
}

void save() {
  //Save the calibration data to EEPROM
  EEPROM.put(0, lsm.cal);
}

void restore() {
  //Restore the calibration data from EEPROM
  EEPROM.get(0, lsm.cal);
}

void printDebug(void) {
  //Print raw sensor data
  SerialPort.print(lsm.mx); SerialPort.print(",");
  SerialPort.print(lsm.my); SerialPort.print(",");
  SerialPort.print(lsm.mz); SerialPort.print(",");
  SerialPort.print(lsm.gx); SerialPort.print(",");
  SerialPort.print(lsm.gy); SerialPort.print(",");
  SerialPort.println(lsm.gz);
}

void printCal(void) {
  //Print the calibration data
  SerialPort.print(lsm.cal.md, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.gs.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.gs.j, 1); SerialPort.print(",");
  SerialPort.println(lsm.cal.gs.k, 1);
}

void printMon(float az, float el, float azSet, float elSet, float azWindup, float azError, float elError) {
  //Print the monitor data
  SerialPort.print(az, 0); SerialPort.print(",");
  SerialPort.print(el, 0); SerialPort.print(",");
  SerialPort.print(azSet, 0); SerialPort.print(",");
  SerialPort.print(elSet, 0); SerialPort.print(",");
  SerialPort.print(azWindup, 0); SerialPort.print(",");
  SerialPort.print(windup); SerialPort.print(",");
  SerialPort.print(azError, 0); SerialPort.print(",");
  SerialPort.println(elError, 0);
}

void printAzEl() {
  //Print the rotator feedback data in Easycomm II format
  SerialPort.print("AZ");
  SerialPort.print((az < 0) ? (az + 360) : az, 1);
  SerialPort.print(" EL");
  SerialPort.print(el, 1);
  SerialPort.print("\n");
}

void calibrate() {
  //Process raw accelerometer and magnetometer samples
  bool changed = lsm.calibrate();
  //Print any changes and beep the speaker to facilitate manual calibration
  if (changed) {
    digitalWrite(spkPin, HIGH);     //Sound the piezo buzzer
    printCal();                     //Print the calibration data
  } else {
    digitalWrite(spkPin, LOW);      //Silence the piezo buzzer
  }
}

void getWindup(bool *windup,  float *azWindup, float *azOffset, float *azLast, float *elLast, float az, float elSet) {
  //Get the accumulated windup angle from the home position (startup or last reset position) and set the windup state if greater than the limit.
  //Get the raw difference angle between the current and last azimuth reading from the sensor
  float azDiff = az - *azLast;

  //Detect crossing South: azDiff jumps 360 for a clockwise crossing or -360 for an anticlockwise crossing
  //Increment the azimuth offset accordingly
  if (azDiff < -180) *azOffset += 360;
  if (azDiff > 180) *azOffset -= 360;

  //Save the current azimuth reading for the next iteration
  *azLast = az;

  //Compute the azimuth wind-up angle, i.e. the absolute number of degrees from the home position
  *azWindup = az + *azOffset;

  //Detect a windup condition where the antenna has rotated more than 450 degrees from home
  if (abs(*azWindup) > WINDUP_LIMIT) *windup = true;    //Set the windup condition - it is reset later when the antenna nears home

  //Perform the anti-windup procedure at the end of each pass - This is overkill unless you absolutely don't want anti-windup during a pass
  //  if (elSet <= 0)
  //    if (elLast > 0)
  //      if (mode == tracking) {
  //        *windup = true;
  //      }

  //Save the current elevation reading for the next iteration
  *elLast = elSet;
}

void getAzElDemo(float *azSet, float *elSet, float *azInc, float *elInc) {
  //Autoincrement the azimuth and elevation to demo the rotator operation
  if (*azSet > 180.0) *azInc = -*azInc;
  if (*azSet < -180.0) *azInc = -*azInc;
  if (*elSet > 90.0) *elInc = -*elInc;
  if (*elSet < 0.0) *elInc = -*elInc;
  *azSet += *azInc;
  *elSet += *elInc;
  SerialPort.print(*azSet, 0); SerialPort.print(",");
  SerialPort.println(*elSet, 0);
}

void getAzElError(float *azError, float *elError, bool *windup, float *azSet, float elSet, float az, float el) {
  //Compute the azimuth and elevation antenna pointing errors, i.e. angular offsets from set positions
  //Compute the azimuth antenna pointing error: Normally via the shortest path; opposite if windup detected.
  if (*windup) {                               //Check for a windup condition
    //To unwind the antenna set an azError in the appropriate direction to home
    *azError = constrain(azWindup, -180, 180); //Limit the maximum azimuth error to -180..180 degrees
    //Cancel the windup condition when the antenna is within 180 degrees of home (Actually 175 degrees to avoid rotation direction ambiguity)
    //Set a zero home position by default, but return azumith control to the computer if still connected
    if (abs(*azError) < 175) *windup = false; //Cancel windup and permit computer control
  }
  else {
    //Compute the normal azimuth antenna pointing error when there is no windup condition
    *azError = diffAngle(az, *azSet);
  }

  //Compute the elevation antenna pointing error
  *elError = diffAngle(el, elSet);
}

void processPosition() {
  //Perform the main operation of positioning the rotator under different modes
  //Read the accelerometer and magnetometer
  lsm.readGM();
  switch (mode) {
    case debugging:
      printDebug(); //Print the raw sensor data for debug purposes
      break;
    case calibrating:
      calibrate();  //Process calibration data
      break;
    case pausing:
      azMot.halt(); //Stop the AZ motor
      elMot.halt(); //Stop the EL motor
      break;
    default:
      lsm.getAzEl();  //Get the azimuth and elevation of the antenna                                                              //Get the antenna AZ and EL
      az = lsm.az;
      el = lsm.el;
      lcd.setCursor(0,0);
      lcd.print("Azimuth:");
      lcd.setCursor(11,0);
      lcd.print(az);
      lcd.setCursor(0,1);
      lcd.print("Elevation:");
      lcd.setCursor(11,1);
      lcd.print(el);
      getWindup(&windup, &azWindup, &azOffset, &azLast, &elLast, az, elSet);      //Get the AZ windup angle and windup state
      if (mode == demonstrating) getAzElDemo(&azSet, &elSet, &azInc, &elInc);     //Set the AZ and EL automatically if in demo mode
      getAzElError(&azError, &elError, &windup, &azSet, elSet, az, el);           //Get the antenna pointing error
      if (mode == monitoring) printMon(az, el, azSet, elSet, azWindup, azError, elError); //Print the data if in monitor mode
  }
}

void processMotors() {
  //Drive the motors to reduce the azimuth and elevation error to zero
  azMot.drive(azError);
  elMot.drive(elError);
}

void processUserCommands(String line) {
  //Process user commands
  //User command type 1: r, b, m, c, a, d, s, d, h, p or e<decl> followed by a carriage return
  //User command type 2: <az> <el> followed by a carriage return
  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command line
  char command = line.charAt(0);                          //Get the first character
  switch (command) {                                      //Process type 1 user commands
    case 'r':                                             //Reset command
      SerialPort.println("Reset in progress");
      reset(true);  //Reset the rotator and load calibration from EEPROM
      SerialPort.println("Reset complete");
      break;
    case 'b':                                             //Debug command
      SerialPort.println("Debugging in progress: Press 'a' to abort");
      mode = debugging;
      t1.reset(100);
      break;
    case 'm':                                             //Monitor command
      SerialPort.println("Monitoring in progress: Press 'a' to abort");
      mode = monitoring;
      t1.reset(100);
      break;
    case 'c':                                             //Calibrate command
      SerialPort.println("Calibration in progress: Press 'a' to abort or 's' to save");
      reset(false); //Reset the rotator, but don't load calibration from EEPROM
      mode = calibrating;
      t1.reset(50);
      break;
    case 'a':                                             //Abort command
      mode = tracking;
      t1.reset(100);
      reset(true);
      SerialPort.println("Function aborted");
      break;
    case 'e':                                             //Magnetic declination command
      param = line.substring(1);                          //Get the second parameter
      lsm.cal.md = param.toFloat();
      break;
    case 's':                                             //Save command
      save();
      reset(true);
      SerialPort.println("Calibration saved");
      break;
    case 'd':                                             //Demo command
      SerialPort.println("Demo in progress: Press 'a' to abort");
      t1.reset(50);
      mode = demonstrating;
      break;
    case 'h':                                             //Help command
      SerialPort.println("Commands:");
      SerialPort.println("az el -(0..360 0..90)");
      SerialPort.println("r -Reset");
      SerialPort.println("eNN.N -MagDecl");
      SerialPort.println("c -Calibrate");
      SerialPort.println("s -Save");
      SerialPort.println("a -Abort");
      SerialPort.println("d -Demo");
      SerialPort.println("b -Debug");
      SerialPort.println("m -Monitor");
      SerialPort.println("p -Pause");
      break;
    case 'p':                                             //Pause command
      if (mode == pausing) {
        mode = tracking;
      } else {
        mode = pausing;
        SerialPort.println("Paused");
      }
      break;
    default:                                              //Process type 2 user commands
      firstSpace = line.indexOf(' ');                     //Get the index of the first space
      param = line.substring(0, firstSpace);              //Get the first parameter
      azSet = param.toFloat();                            //Get the azSet value
      param = line.substring(firstSpace + 1);             //Get the second parameter
      elSet = param.toFloat();                            //Get the elSet value
  }
}

void processEasycommCommands(String line) {
  //Process Easycomm II rotator commands
  //Easycomm II position command: AZnn.n ELnn.n UP000 XXX DN000 XXX\n
  //Easycomm II query command: AZ EL \n
  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command line
  int secondSpace;                                        //Position of the second space in the command line
  if (line.startsWith("AZ EL")) {                         //Query command received
    printAzEl();                                          //Send the current Azimuth and Elevation
  } else {
    if (line.startsWith("AZ")) {                          //Position command received: Parse the line.
      firstSpace = line.indexOf(' ');                     //Get the position of the first space
      secondSpace = line.indexOf(' ', firstSpace + 1);    //Get the position of the second space
      param = line.substring(2, firstSpace);              //Get the first parameter
      azSet = param.toFloat();                            //Set the azSet value
      if (azSet > 180) azSet = azSet - 360;               //Convert 0..360 to -180..180 degrees format
      param = line.substring(firstSpace + 3, secondSpace);//Get the second parameter
      elSet = param.toFloat();                            //Set the elSet value
    }
  }
}

void processCommands(void) {
  //Process incoming data from the control computer
  //User commands are entered by the user and are terminated with a carriage return
  //Easycomm commands are generated by a tracking program and are terminated with a line feed
  while (SerialPort.available() > 0) {
    char ch = SerialPort.read();                                //Read a single character from the serial buffer
    switch (ch) {
      case 13:                                                  //Carriage return received
        processUserCommands(line);                              //Process user commands
        line = "";                                              //Command processed: Clear the command line
        break;
      case 10:                                                  //Line feed received
        processEasycommCommands(line);                          //Process Easycomm commands
        line = "";                                              //Command processed: Clear the command line
        break;
      default:                                                  //Any other character received
        line += ch;                                             //Add this character to the command line
        break;
    }
  }
}

void setup() {
  //Initialize the system
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  //Set speaker pins to outputs
  pinMode(spkPin, OUTPUT);
  pinMode(gndPin, OUTPUT);
  digitalWrite(gndPin, LOW);
  //Reset the rotator and load configuration from EEPROM
  reset(true);
  //Initialize the serial port
  SerialPort.begin(9600);
  //Initialize the sensor
  lsm.begin();
}

void loop() {
  //Repeat continuously
  processCommands();                                              //Process commands from the control computer
  t1.execute(&processPosition);                                   //Process position only periodically
  processMotors();                                                //Process motor drive
}
