#include "MPU9250.h"

// Create Array for Storing Data
int CalData[100][5];

// Initialize Variables for Storing time data
unsigned long start;
unsigned long now;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  // Calibrate Accelerometer
  float calCont[3];
  for(int i=1;i<4;i++){
      calCont[i] = IMUCal(i);
    }

  // Get the start time
  start = millis();
}

void loop() {
  // read the sensor
  IMU.readSensor();
  now = millis();

  // display the data
  Serial.print(now);
  Serial.print("\t");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  /*
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  */
  Serial.println(IMU.getTemperature_C(),6);
  delay(10);
}

// Create a calibration Function
// Inputs: Item to calibrate
// Output: Constant value of data
float IMUCal(int n){

  // Initiate the data vector
  float calData[100];

  // Fill the calibration Data
  switch(n){

    case 1:
       for(int i=0; i<100; i++){
        calData[i] = IMU.getAccelX_mss();
        }
       break;

   case 2:
       for(int i=0; i<100; i++){
        calData[i] = IMU.getAccelY_mss();
        }
       break;
    
   case 3:
       for(int i=0; i<100; i++){
        calData[i] = IMU.getAccelZ_mss();
        }
       break; 
    }
  
  float average;

  for (int i=0; i<100; i++){
    
    average += calData[i];
    }

 average = average/100;

 return average;
  
  }
