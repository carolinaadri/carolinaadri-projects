#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect VDD to 3-5V DC (depending on your board's logic level)
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address

Adafruit_BNO055 thumb = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_BNO055 indexF = Adafruit_BNO055(-1, BNO055_ADDRESS_B);
Adafruit_BNO055 middle = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &Wire);
Adafruit_BNO055 pinky = Adafruit_BNO055(-1, BNO055_ADDRESS_B, &Wire);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t thumbSensor;
  sensor_t indexSensor;
  sensor_t middleSensor;
  sensor_t pinkySensor;
  thumb.getSensor(&thumbSensor);
  indexF.getSensor(&indexSensor);
  thumb.getSensor(&middleSensor);
  indexF.getSensor(&pinkySensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 1:       "); Serial.println(thumbSensor.name);
  Serial.print  ("Driver Ver 1:   "); Serial.println(thumbSensor.version);
  Serial.print  ("Unique ID 1:    "); Serial.println(thumbSensor.sensor_id);
  Serial.print  ("Max Value 1:    "); Serial.print(thumbSensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 1:    "); Serial.print(thumbSensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 1:   "); Serial.print(thumbSensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 2:       "); Serial.println(indexSensor.name);
  Serial.print  ("Driver Ver 2:   "); Serial.println(indexSensor.version);
  Serial.print  ("Unique ID 2:    "); Serial.println(indexSensor.sensor_id);
  Serial.print  ("Max Value 2:    "); Serial.print(indexSensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 2:    "); Serial.print(indexSensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 2:   "); Serial.print(indexSensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 3:       "); Serial.println(middleSensor.name);
  Serial.print  ("Driver Ver 3:   "); Serial.println(middleSensor.version);
  Serial.print  ("Unique ID 3:    "); Serial.println(middleSensor.sensor_id);
  Serial.print  ("Max Value 3:    "); Serial.print(middleSensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 3:    "); Serial.print(middleSensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 3:   "); Serial.print(middleSensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 4:       "); Serial.println(pinkySensor.name);
  Serial.print  ("Driver Ver 4:   "); Serial.println(pinkySensor.version);
  Serial.print  ("Unique ID 4:    "); Serial.println(pinkySensor.sensor_id);
  Serial.print  ("Max Value 4:    "); Serial.print(pinkySensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 4:    "); Serial.print(pinkySensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 4:   "); Serial.print(pinkySensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t thumb_system_status, thumb_self_test_results, thumb_system_error;
  thumb_system_status = thumb_self_test_results = thumb_system_error = 0;
  thumb.getSystemStatus(&thumb_system_status, &thumb_self_test_results, &thumb_system_error);
  uint8_t index_system_status, index_self_test_results, index_system_error;
  index_system_status = index_self_test_results = index_system_error = 0;
  indexF.getSystemStatus(&index_system_status, &index_self_test_results, &index_system_error);
  uint8_t middle_system_status, middle_self_test_results, middle_system_error;
  middle_system_status = middle_self_test_results = middle_system_error = 0;
  middle.getSystemStatus(&middle_system_status, &middle_self_test_results, &middle_system_error);
  uint8_t pinky_system_status, pinky_self_test_results, pinky_system_error;
  pinky_system_status = pinky_self_test_results = pinky_system_error = 0;
  pinky.getSystemStatus(&pinky_system_status, &pinky_self_test_results, &pinky_system_error);
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status 1: 0x");
  Serial.println(thumb_system_status, HEX);
  Serial.print("Self Test 1:     0x");
  Serial.println(thumb_self_test_results, HEX);
  Serial.print("System Error 1:  0x");
  Serial.println(thumb_system_error, HEX);
  Serial.println("");
  Serial.println("");
  Serial.print("System Status 2: 0x");
  Serial.println(index_system_status, HEX);
  Serial.print("Self Test 2:     0x");
  Serial.println(index_self_test_results, HEX);
  Serial.print("System Error 2:  0x");
  Serial.println(index_system_error, HEX);
  Serial.println("");
  Serial.println("");
  Serial.print("System Status 3: 0x");
  Serial.println(middle_system_status, HEX);
  Serial.print("Self Test 3:     0x");
  Serial.println(middle_self_test_results, HEX);
  Serial.print("System Error 3:  0x");
  Serial.println(middle_system_error, HEX);
  Serial.println("");
  Serial.println("");
  Serial.print("System Status 4: 0x");
  Serial.println(pinky_system_status, HEX);
  Serial.print("Self Test 4:     0x");
  Serial.println(pinky_self_test_results, HEX);
  Serial.print("System Error 4:  0x");
  Serial.println(pinky_system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t thumbSystem, thumbGyro, thumbAccel, thumbMag;
  thumbSystem = thumbGyro = thumbAccel = thumbMag = 0;
  thumb.getCalibration(&thumbSystem, &thumbGyro, &thumbAccel, &thumbMag);
  uint8_t indexSystem, indexGyro, indexAccel, indexMag;
  indexSystem = indexGyro = indexAccel = indexMag = 0;
  indexF.getCalibration(&indexSystem, &indexGyro, &indexAccel, &indexMag);
  uint8_t middleSystem, middleGyro, middleAccel, middleMag;
  middleSystem = middleGyro = middleAccel = middleMag = 0;
  middle.getCalibration(&middleSystem, &middleGyro, &middleAccel, &middleMag);
  uint8_t pinkySystem, pinkyGyro, pinkyAccel, pinkyMag;
  pinkySystem = pinkyGyro = pinkyAccel = pinkyMag = 0;
  pinky.getCalibration(&pinkySystem, &pinkyGyro, &pinkyAccel, &pinkyMag);
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!thumbSystem || !indexSystem || !middleSystem || !pinkySystem)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys 1:");
  Serial.print(thumbSystem, DEC);
  Serial.print(" G 1:");
  Serial.print(thumbGyro, DEC);
  Serial.print(" A 1:");
  Serial.print(thumbAccel, DEC);
  Serial.print(" M 1:");
  Serial.print(thumbMag, DEC);
  
  Serial.print("Sys 2:");
  Serial.print(indexSystem, DEC);
  Serial.print(" G 2:");
  Serial.print(indexGyro, DEC);
  Serial.print(" A 2:");
  Serial.print(indexAccel, DEC);
  Serial.print(" M 2:");
  Serial.print(indexMag, DEC);

  Serial.print("Sys 3:");
  Serial.print(middleSystem, DEC);
  Serial.print(" G 3:");
  Serial.print(middleGyro, DEC);
  Serial.print(" A 3:");
  Serial.print(middleAccel, DEC);
  Serial.print(" M 3:");
  Serial.print(middleMag, DEC);
  
  Serial.print("Sys 4:");
  Serial.print(pinkySystem, DEC);
  Serial.print(" G 4:");
  Serial.print(pinkyGyro, DEC);
  Serial.print(" A 4:");
  Serial.print(pinkyAccel, DEC);
  Serial.print(" M 4:");
  Serial.print(pinkyMag, DEC);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Wire.begin();
  // pinMode(70, LOW);
  // pinMode(71, LOW);
  pinMode(9, OUTPUT); //pump 1
  pinMode(10, OUTPUT); //pump 2
  pinMode(11, OUTPUT); //index valve 1
  pinMode(12, OUTPUT); //index valve 2
  
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  // if(!thumb.begin() || !indexF.begin() || !middle.begin() || !pinky.begin())
  // {
  //   /* There was a problem detecting the BNO055 ... check your connections */
  //   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while(1);
  // }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  thumb.setExtCrystalUse(true);
  indexF.setExtCrystalUse(true);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t thumbEvent;
  thumb.getEvent(&thumbEvent);
  sensors_event_t indexEvent;
  indexF.getEvent(&indexEvent);
  sensors_event_t middleEvent;
  middle.getEvent(&middleEvent);
  sensors_event_t pinkyEvent;
  pinky.getEvent(&pinkyEvent);
  /* Display the floating point data */
  // Serial.print("X 1: ");
  // Serial.print(thumbEvent.orientation.x, 4);
  // Serial.print(" Y 1: ");
  // Serial.print(thumbEvent.orientation.y, 4);
  // Serial.print(" Z 1: ");
  // Serial.print(thumbEvent.orientation.z, 4);
  // Serial.print("\t\tX 2: ");
  // Serial.print(indexEvent.orientation.x, 4);
  // Serial.print(" Y 2: ");
  // Serial.print(indexEvent.orientation.y, 4);
  // Serial.print(" Z 2: ");
  // Serial.print(indexEvent.orientation.z, 4);
  // Serial.print("\t\tX 3: ");
  // Serial.print(middleEvent.orientation.x, 4);
  // Serial.print(" Y 3: ");
  // Serial.print(middleEvent.orientation.y, 4);
  // Serial.print(" Z 3: ");
  // Serial.print(middleEvent.orientation.z, 4);
  // Serial.print("\t\tX 4: ");
  // Serial.print(pinkyEvent.orientation.x, 4);
  // Serial.print(" Y 4: ");
  // Serial.print(pinkyEvent.orientation.y, 4);
  // Serial.print(" Z 4: ");
  // Serial.print(pinkyEvent.orientation.z, 4);
  // Serial.print("\t\t");
  byte thumb;
  byte index;
  byte middle;
  byte pinky;
  int gloveThumb;
  int gloveIndex;
  int gloveMiddle;
  int glovePinky;
  //Serial.println("");
  // delay(1000);
  //Serial.setTimeout(50);
  //while(Serial.read() != "#");
  Serial.println("here");
  thumb = Serial.parseInt();
  index = Serial.parseInt();
  middle = Serial.parseInt();
  pinky = Serial.parseInt();

  Serial.print(thumb);
  Serial.print(index);
  Serial.print(middle);
  Serial.print(pinky);
  // if(Serial.available() > 0) {
  //   delay(50);
  //   thumb1 = Serial.parseInt();
  //   delay(50);
  //   //thumb2 = Serial.read();
  //   index1 = Serial.parseInt();
  //   delay(50);
  //   //index2 = Serial.read();
  //   middle1 = Serial.parseInt();
  //   delay(50);
  //   //middle2 = Serial.read();
  //   pinky1 = Serial.parseInt();
  //   delay(50);
    //pinky2 = Serial.read();
    // gloveThumb = thumb1 - '0'; //+ thumb2;
    // gloveIndex = index1 - '0';// + index2;
    // gloveMiddle = middle1 - '0';// + middle2;
    // glovePinky = pinky1 - '0';// + pinky2;
    // if(gloveThumb > 9) {
    //   gloveThumb = 0;
    // }
    // if(gloveIndex > 9) {
    //   gloveIndex = 0;
    // }
    // if(gloveMiddle > 9) {
    //   gloveMiddle = 0;
    // }
    // if(glovePinky > 9) {
    //   glovePinky = 0;
    // }
  //}
  Serial.print("Index ");
  //Serial.print(gloveIndex);
  int thumbFIST = thumbEvent.orientation.y;
  int indexFIST = indexEvent.orientation.y;
  int middleFIST = middleEvent.orientation.y;
  int pinkyFIST = pinkyEvent.orientation.y;

  if(0 == gloveIndex) {
    digitalWrite(9, LOW); //stop pump
    digitalWrite(10, LOW); //stop pump
    digitalWrite(11, LOW); //close valve 1
    digitalWrite(12, LOW); //close valve 2
  } else if (0 > gloveIndex) {
    digitalWrite(9, LOW); //stop other pump (pressure)
    digitalWrite(12, LOW); //close other valve
    digitalWrite(11, HIGH); //open valve
    digitalWrite(10, HIGH); //start pump (vacuum)
  } else {
    digitalWrite(10, LOW); //stop other pump (vacuum)
    digitalWrite(11, LOW); //close other valve
    digitalWrite(12, HIGH); //open valve
    digitalWrite(9, HIGH); //start pump (pressure)
  }

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
