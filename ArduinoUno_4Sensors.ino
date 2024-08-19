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

Adafruit_BNO055 bno1 = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_BNO055 bno2 = Adafruit_BNO055(-1, BNO055_ADDRESS_B);
Adafruit_BNO055 bno3 = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &Wire);
Adafruit_BNO055 bno4 = Adafruit_BNO055(-1, BNO055_ADDRESS_B, &Wire);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor1;
  sensor_t sensor2;
  sensor_t sensor3;
  sensor_t sensor4;
  bno1.getSensor(&sensor1);
  bno2.getSensor(&sensor2);
  bno1.getSensor(&sensor3);
  bno2.getSensor(&sensor4);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 1:       "); Serial.println(sensor1.name);
  Serial.print  ("Driver Ver 1:   "); Serial.println(sensor1.version);
  Serial.print  ("Unique ID 1:    "); Serial.println(sensor1.sensor_id);
  Serial.print  ("Max Value 1:    "); Serial.print(sensor1.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 1:    "); Serial.print(sensor1.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 1:   "); Serial.print(sensor1.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 2:       "); Serial.println(sensor2.name);
  Serial.print  ("Driver Ver 2:   "); Serial.println(sensor2.version);
  Serial.print  ("Unique ID 2:    "); Serial.println(sensor2.sensor_id);
  Serial.print  ("Max Value 2:    "); Serial.print(sensor2.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 2:    "); Serial.print(sensor2.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 2:   "); Serial.print(sensor2.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
    Serial.println("------------------------------------");
  Serial.print  ("Sensor 3:       "); Serial.println(sensor3.name);
  Serial.print  ("Driver Ver 3:   "); Serial.println(sensor3.version);
  Serial.print  ("Unique ID 3:    "); Serial.println(sensor3.sensor_id);
  Serial.print  ("Max Value 3:    "); Serial.print(sensor3.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 3:    "); Serial.print(sensor3.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 3:   "); Serial.print(sensor3.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 4:       "); Serial.println(sensor4.name);
  Serial.print  ("Driver Ver 4:   "); Serial.println(sensor4.version);
  Serial.print  ("Unique ID 4:    "); Serial.println(sensor4.sensor_id);
  Serial.print  ("Max Value 4:    "); Serial.print(sensor4.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 4:    "); Serial.print(sensor4.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 4:   "); Serial.print(sensor4.resolution); Serial.println(" xxx");
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
  uint8_t system_status1, self_test_results1, system_error1;
  system_status1 = self_test_results1 = system_error1 = 0;
  bno1.getSystemStatus(&system_status1, &self_test_results1, &system_error1);
  uint8_t system_status2, self_test_results2, system_error2;
  system_status2 = self_test_results2 = system_error2 = 0;
  bno2.getSystemStatus(&system_status2, &self_test_results2, &system_error2);
  uint8_t system_status3, self_test_results3, system_error3;
  system_status3 = self_test_results3 = system_error3 = 0;
  bno3.getSystemStatus(&system_status3, &self_test_results3, &system_error3);
  uint8_t system_status4, self_test_results4, system_error4;
  system_status4 = self_test_results4 = system_error4 = 0;
  bno4.getSystemStatus(&system_status4, &self_test_results4, &system_error4);
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status 1: 0x");
  Serial.println(system_status1, HEX);
  Serial.print("Self Test 1:     0x");
  Serial.println(self_test_results1, HEX);
  Serial.print("System Error 1:  0x");
  Serial.println(system_error1, HEX);
  Serial.println("");
  Serial.println("");
  Serial.print("System Status 2: 0x");
  Serial.println(system_status2, HEX);
  Serial.print("Self Test 2:     0x");
  Serial.println(self_test_results2, HEX);
  Serial.print("System Error 2:  0x");
  Serial.println(system_error2, HEX);
  Serial.println("");
  Serial.println("");
  Serial.print("System Status 3: 0x");
  Serial.println(system_status3, HEX);
  Serial.print("Self Test 3:     0x");
  Serial.println(self_test_results3, HEX);
  Serial.print("System Error 3:  0x");
  Serial.println(system_error3, HEX);
  Serial.println("");
  Serial.println("");
  Serial.print("System Status 4: 0x");
  Serial.println(system_status4, HEX);
  Serial.print("Self Test 4:     0x");
  Serial.println(self_test_results4, HEX);
  Serial.print("System Error 4:  0x");
  Serial.println(system_error4, HEX);
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
  uint8_t system1, gyro1, accel1, mag1;
  system1 = gyro1 = accel1 = mag1 = 0;
  bno1.getCalibration(&system1, &gyro1, &accel1, &mag1);
  uint8_t system2, gyro2, accel2, mag2;
  system2 = gyro2 = accel2 = mag2 = 0;
  bno2.getCalibration(&system2, &gyro2, &accel2, &mag2);
  uint8_t system3, gyro3, accel3, mag3;
  system3 = gyro3 = accel3 = mag3 = 0;
  bno3.getCalibration(&system3, &gyro3, &accel3, &mag3);
  uint8_t system4, gyro4, accel4, mag4;
  system4 = gyro4 = accel4 = mag4 = 0;
  bno4.getCalibration(&system4, &gyro4, &accel4, &mag4);
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  // if (!system1 || !system2 || !system3 || !system4)
  // {
  //   Serial.print("! ");
  // }

  // /* Display the individual values */
  // Serial.print("Sys 1:");
  // Serial.print(system1, DEC);
  // Serial.print(" G 1:");
  // Serial.print(gyro1, DEC);
  // Serial.print(" A 1:");
  // Serial.print(accel1, DEC);
  // Serial.print(" M 1:");
  // Serial.print(mag1, DEC);
  
  // Serial.print("Sys 2:");
  // Serial.print(system2, DEC);
  // Serial.print(" G 2:");
  // Serial.print(gyro2, DEC);
  // Serial.print(" A 2:");
  // Serial.print(accel2, DEC);
  // Serial.print(" M 2:");
  // Serial.print(mag2, DEC);

  // Serial.print("Sys 3:");
  // Serial.print(system3, DEC);
  // Serial.print(" G 3:");
  // Serial.print(gyro3, DEC);
  // Serial.print(" A 3:");
  // Serial.print(accel3, DEC);
  // Serial.print(" M 3:");
  // Serial.print(mag3, DEC);
  
  // Serial.print("Sys 4:");
  // Serial.print(system4, DEC);
  // Serial.print(" G 4:");
  // Serial.print(gyro4, DEC);
  // Serial.print(" A 4:");
  // Serial.print(accel4, DEC);
  // Serial.print(" M 4:");
  // Serial.print(mag4, DEC);
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
  pinMode(13, OUTPUT);
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  
  // if( !bno1.begin() || !bno2.begin() || !bno3.begin() || !bno4.begin())
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

  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);

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
  sensors_event_t event1;
  bno1.getEvent(&event1);
  sensors_event_t event2;
  bno2.getEvent(&event2);
  sensors_event_t event3;
  bno3.getEvent(&event3);
  sensors_event_t event4;
  bno4.getEvent(&event4);
  /* Display the floating point data */
  // Serial.print("X 1: ");
  // Serial.print(event1.orientation.x, 4);
  // Serial.print(" Y 1: ");
  // Serial.print(event1.orientation.y, 4);
  // Serial.print(" Z 1: ");
  // Serial.print(event1.orientation.z, 4);
  // Serial.print("\t\tX 2: ");
  // Serial.print(event2.orientation.x, 4);
  // Serial.print(" Y 2: ");
  // Serial.print(event2.orientation.y, 4);
  // Serial.print(" Z 2: ");
  // Serial.print(event2.orientation.z, 4);
  // Serial.print("\t\tX 3: ");
  // Serial.print(event3.orientation.x, 4);
  // Serial.print(" Y 3: ");
  // Serial.print(event3.orientation.y, 4);
  // Serial.print(" Z 3: ");
  // Serial.print(event3.orientation.z, 4);
  // Serial.print("\t\tX 4: ");
  // Serial.print(event4.orientation.x, 4);
  // Serial.print(" Y 4: ");
  // Serial.print(event4.orientation.y, 4);
  // Serial.print(" Z 4: ");
  // Serial.print(event4.orientation.z, 4);
  // Serial.print("\t\t");
  byte thumb1;
  byte thumb2;
  byte index1;
  byte index2;
  byte middle1;
  byte middle2;
  byte pinky1;
  byte pinky2;
  int gloveThumb;
  int gloveIndex;
  int gloveMiddle;
  int glovePinky;
  Serial.println("");
  Serial.println("here");
 // delay(1000);
  if(Serial.read() >= 4) {
    delay(50);
    thumb1 = Serial.read();
    delay(50);
    //thumb2 = Serial.read();
    index1 = Serial.read();
    delay(50);
    //index2 = Serial.read();
    middle1 = Serial.read();
    delay(50);
    //middle2 = Serial.read();
    pinky1 = Serial.read();
    delay(50);
    //pinky2 = Serial.read();
    gloveThumb = thumb1 - '0'; //+ thumb2;
    gloveIndex = index1 - '0';// + index2;
    gloveMiddle = middle1 - '0';// + middle2;
    glovePinky = pinky1 - '0';// + pinky2;
    if(gloveThumb > 9) {
      gloveThumb = 0;
    }
    if(gloveIndex > 9) {
      gloveIndex = 0;
    }
    if(gloveMiddle > 9) {
      gloveMiddle = 0;
    }
    if(glovePinky > 9) {
      glovePinky = 0;
    }
    Serial.println("");
    Serial.print("Glove values: ");
    Serial.print(gloveThumb);
    Serial.print("\t\t");
    Serial.print(gloveIndex);
    Serial.print("\t\t");
    Serial.print(gloveMiddle);
    Serial.print("\t\t");
    Serial.print(glovePinky);
  }
  
  if((gloveThumb >= -3) && (gloveThumb <= 3)) {
    digitalWrite(13, HIGH);
    digitalWrite(11, HIGH); //for demonstration
    //digitalWrite(10, LOW);
  } else {
    digitalWrite(13, LOW);
    digitalWrite(11, HIGH);
    //digitalWrite(10, HIGH);  
  }
  delay(100);
  if(gloveIndex >= -3 && gloveIndex <= 3) {
    digitalWrite(14, 255);
    // digitalWrite(4, LOW);
    // digitalWrite(5, LOW);
  } else {
    digitalWrite(14, 0);
    // digitalWrite(4, HIGH);
    // digitalWrite(5, HIGH);  
  }
  delay(100);
  if(gloveMiddle >= -3 && gloveMiddle <= 3) {
    digitalWrite(15, 255);
    // digitalWrite(6, LOW);
    // digitalWrite(7, LOW);
    // digitalWrite(8, LOW);
    // digitalWrite(9, LOW);
  } else {
    digitalWrite(15, 0);
    // digitalWrite(6, HIGH);
    // digitalWrite(7, HIGH);
    // digitalWrite(8, HIGH);
    // digitalWrite(9, HIGH);  
  }
  delay(100);
    if(glovePinky >= -3 && glovePinky <= 3) {
    digitalWrite(16, 255);
    // digitalWrite(10, LOW);
    // digitalWrite(11, LOW);
  } else {
    digitalWrite(16, 0);
    // digitalWrite(10, HIGH);
    // digitalWrite(11, HIGH);  
  }
  delay(100);
  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
