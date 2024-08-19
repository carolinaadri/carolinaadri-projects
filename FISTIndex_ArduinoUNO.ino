#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define pumpVacuum 2
#define pumpPressure 3
#define pressureValve 4
#define vacuumValve 5

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

Adafruit_BNO055 index = Adafruit_BNO055(-1, BNO055_ADDRESS_A);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor1;
  index.getSensor(&sensor1);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor 1:       "); Serial.println(sensor1.name);
  Serial.print  ("Driver Ver 1:   "); Serial.println(sensor1.version);
  Serial.print  ("Unique ID 1:    "); Serial.println(sensor1.sensor_id);
  Serial.print  ("Max Value 1:    "); Serial.print(sensor1.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value 1:    "); Serial.print(sensor1.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution 1:   "); Serial.print(sensor1.resolution); Serial.println(" xxx");
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
  index.getSystemStatus(&system_status1, &self_test_results1, &system_error1);
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status 1: 0x");
  Serial.println(system_status1, HEX);
  Serial.print("Self Test 1:     0x");
  Serial.println(self_test_results1, HEX);
  Serial.print("System Error 1:  0x");
  Serial.println(system_error1, HEX);
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
  index.getCalibration(&system1, &gyro1, &accel1, &mag1);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system1)
  {
    Serial.print("! ");
  }

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
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  
  if( !index.begin() /*|| !bno2.begin() || !bno3.begin() || !bno4.begin()*/)
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  index.setExtCrystalUse(true);

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
  index.getEvent(&event1);
  // sensors_event_t event2;
  // bno2.getEvent(&event2);
  // sensors_event_t event3;
  // bno3.getEvent(&event3);
  // sensors_event_t event4;
  // bno4.getEvent(&event4);
  /* Display the floating point data */
  Serial.print("X 1: ");
  Serial.print(event1.orientation.x, 4);
  Serial.print(" Y 1: ");
  Serial.print(event1.orientation.y, 4);
  Serial.print(" Z 1: ");
  Serial.print(event1.orientation.z, 4);
  int indexVal = event1.orientation.y;
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
  // long gloveIndex = 0;
  // int gloveVal = 0;
  Serial.println("");
 // delay(1000);
  // delay(10);
  // Serial.println("Before data");
  // if(Serial.available() > 0) {
  //   Serial.println("Has data");
  //   // if(Serial.parseInt() > 0) {
  //   //   gloveIndex = Serial.parseInt();
  //   // } else {
  //     gloveIndex = Serial.read() - '0';
  //  // }
  //   Serial.println("Got data");
  // }
  // if(gloveIndex >= 200) {
  //   gloveVal = gloveIndex - 256;
  // } else {
  //   gloveVal = gloveIndex;
  // }
  // if(gloveIndex == 253) {
  //   gloveIndex = 0;
  // }
  // Serial.print("Glove: ");
  // Serial.println(gloveVal);

  if(indexVal == 0) {
    digitalWrite(3, HIGH);
    digitalWrite(2, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
  } else if (indexVal > 0) {
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(2, LOW);
  } else {
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(2, HIGH);
  }
  delay(100);
  //   if(indexVal == 0) {
  //   digitalWrite(pumpPressure, HIGH);
  //   digitalWrite(pumpVacuum, HIGH);
  // } else if (indexVal < 0) {
  //   digitalWrite(pumpPressure, HIGH);
  //   digitalWrite(pumpVacuum, LOW);
  // } else {
  //   digitalWrite(pumpPressure, LOW);
  //   digitalWrite(pumpVacuum, HIGH);
  // }
  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}