#include "OrientationTracker.h"

//TODO: fill in from hw 4 as necessary

OrientationTracker::OrientationTracker(double imuFilterAlphaIn,  bool simulateImuIn) :

  imu(),
  gyr{0,0,0},
  acc{0,0,0},
  gyrBias{0,0,0},
  gyrVariance{0,0,0},
  accBias{0,0,0},
  accVariance{0,0,0},
  previousTimeImu(0),
  imuFilterAlpha(imuFilterAlphaIn),
  deltaT(0.0),
  simulateImu(simulateImuIn),
  simulateImuCounter(0),
  flatlandRollGyr(0),
  flatlandRollAcc(0),
  flatlandRollComp(0),
  quaternionGyr{1,0,0,0},
  eulerAcc{0,0,0},
  quaternionComp{1,0,0,0}

  {

}

void OrientationTracker::initImu() {
  Wire1.setSCL(ICM_SCL);
  Wire1.setSDA(ICM_SDA);
  Wire1.begin(ICM_ADR);

  // Try to initialize!
  if (!imu.begin_I2C(ICM_ADR, &Wire1)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
}


/**
 * TODO: see documentation in header file
 */
void OrientationTracker::measureImuBiasVariance() {

  // **** These comments are incorrect on interfacing with IMU:
  //check if imu.read() returns true
  //then read imu.gyrX, imu.accX, ...
  // ****

  //compute bias, variance.
  //update:
  //gyrBias[0], gyrBias[1], gyrBias[2]
  //gyrVariance[0], gyrBias[1], gyrBias[2]
  //accBias[0], accBias[1], accBias[2]
  //accVariance[0], accBias[1], accBias[2]


  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  float gyrSum[3] = {0.0, 0.0, 0.0};
  float accSum[3] = {0.0, 0.0, 0.0};
  float gyrSumSq[3] = {0.0, 0.0, 0.0};
  float accSumSq[3] = {0.0, 0.0, 0.0};
  int count = 0;
  while (count < 1000){
    if(imu.getEvent(&accel, &gyro, &temp, &mag)){
      gyr[0] = gyro.gyro.x;
      gyr[1] = gyro.gyro.y;
      gyr[2] = gyro.gyro.z;

      acc[0] = accel.acceleration.x;
      acc[1] = accel.acceleration.y;
      acc[2] = accel.acceleration.z;

      gyrSum[0] += gyr[0];
      gyrSumSq[0] += gyr[0]*gyr[0];
      gyrSum[1] += gyr[1];
      gyrSumSq[1] += gyr[1]*gyr[1];
      gyrSum[2] += gyr[2];
      gyrSumSq[2] += gyr[2]*gyr[2];
      accSum[0] += acc[0];
      accSumSq[0] += acc[0]*acc[0];
      accSum[1] += acc[1];
      accSumSq[1] += acc[1]*acc[1];
      accSum[2] += acc[2];
      accSumSq[2] += acc[2]*acc[2];
      count++;
    }
  }
  gyrBias[0] = gyrSum[0]/count;
  gyrBias[1] = gyrSum[1]/count;
  gyrBias[2] = gyrSum[2]/count;
  accBias[0] = accSum[0]/count;
  accBias[1] = accSum[1]/count;
  accBias[2] = accSum[2]/count;
  gyrVariance[0] = (gyrSumSq[0] - (gyrSum[0]*gyrSum[0]/count))/count;
  gyrVariance[1] = (gyrSumSq[1] - (gyrSum[1]*gyrSum[1]/count))/count;
  gyrVariance[2] = (gyrSumSq[2] - (gyrSum[2]*gyrSum[2]/count))/count;
  accVariance[0] = (accSumSq[0] - (accSum[0]*accSum[0]/count))/count;
  accVariance[1] = (accSumSq[1] - (accSum[1]*accSum[1]/count))/count;
  accVariance[2] = (accSumSq[2] - (accSum[2]*accSum[2]/count))/count;

  /* Results: */
  /* 2.1.1 Bias Estimation */
  /* GYR_BIAS: -0.00068 0.01259 -0.00854 */
  /* ACC_BIAS: -0.01790 -0.11327 10.11502 */

  /* 2.1.2 Variance Estimation */
  /* GYR_VAR: 0.00001 0.00001 0.00001 */
  /* ACC_VAR: 0.00077 0.00080 0.00044 */
}

void OrientationTracker::setImuBias(double bias[3]) {

  for (int i = 0; i < 3; i++) {
    gyrBias[i] = bias[i];
  }

}

void OrientationTracker::resetOrientation() {

  flatlandRollGyr = 0;
  flatlandRollAcc = 0;
  flatlandRollComp = 0;
  quaternionGyr = Quaternion();
  eulerAcc[0] = 0;
  eulerAcc[1] = 0;
  eulerAcc[2] = 0;
  quaternionComp = Quaternion();

}

bool OrientationTracker::processImu() {

  if (simulateImu) {

    //get imu values from simulation
    updateImuVariablesFromSimulation();

  } else {

    //get imu values from actual sensor
    if (!updateImuVariables()) {

      //imu data not available
      return false;

    }

  }

  //run orientation tracking algorithms
  updateOrientation();

  return true;

}

void OrientationTracker::updateImuVariablesFromSimulation() {

    deltaT = 0.002;
    //get simulated imu values from external file
    for (int i = 0; i < 3; i++) {
      gyr[i] = imuData[simulateImuCounter + i];
    }
    simulateImuCounter += 3;
    for (int i = 0; i < 3; i++) {
      acc[i] = imuData[simulateImuCounter + i];
    }
    simulateImuCounter += 3;
    simulateImuCounter = simulateImuCounter % nImuSamples;

    //simulate delay
    delay(1);

}

/**
 * TODO: see documentation in header file
 */
bool OrientationTracker::updateImuVariables() {

  //sample imu values
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);

  //call micros() to get current time in microseconds
  //update:
  //previousTimeImu (in seconds)
  //deltaT (in seconds)

  float currentTimeImu = (micros()/1000000.0); // microseconds to seconds
  deltaT = currentTimeImu - previousTimeImu; 
  previousTimeImu = currentTimeImu;

  //read imu.gyrX, imu.accX ...
  //update:
  //gyr[0], ...
  //acc[0], ...

  // You also need to appropriately modify the update of gyr as instructed in (2.1.3).
  gyr[0] = gyro.gyro.x - gyrBias[0];
  gyr[1] = gyro.gyro.y - gyrBias[1];
  gyr[2] = gyro.gyro.z - gyrBias[2];

  acc[0] = accel.acceleration.x - accBias[0];
  acc[1] = accel.acceleration.y - accBias[1];
  acc[2] = accel.acceleration.z - accBias[2];

  return true;

}


/**
 * TODO: see documentation in header file
 */
void OrientationTracker::updateOrientation() {

  //call functions in OrientationMath.cpp.
  //use only class variables as arguments to functions.

  //update:
  //flatlandRollGyr
  //flatlandRollAcc
  //flatlandRollComp
  //quaternionGyr
  //eulerAcc
  //quaternionComp

  flatlandRollGyr = computeFlatlandRollGyr(flatlandRollGyr, gyr, deltaT);
  flatlandRollAcc = computeFlatlandRollAcc(acc);
  flatlandRollComp = computeFlatlandRollComp(flatlandRollComp, gyr, flatlandRollAcc, deltaT, imuFilterAlpha);
  updateQuaternionGyr(quaternionGyr, gyr, deltaT);
  eulerAcc[0] = computeAccPitch(acc);
  eulerAcc[2] = computeAccRoll(acc);
  updateQuaternionComp(quaternionComp, gyr, acc, deltaT, imuFilterAlpha);

}
