/**
 *  This is the main file for pose tracking with the VRduino.
 */

#include <Wire.h>
#include "OrientationTracker.h"
#include "OrientationMath.h"
#include "TestPose.h"
#include "PoseTracker.h"
#include "PoseMath.h"
#include "PulsePosition.h"

const unsigned int kOutputStringWidth = 40;

//intialize orientation tracker
double alphaImuFilter = 0.9;
bool simulateImu = false;
OrientationTracker tracker(alphaImuFilter, simulateImu);

//runs when the Teensy is powered on

//HW06
//complementary filter value [0,1].
//1: ignore acc tilt, 0: use all acc tilt
//double alphaImuFilter = 0.99;

//get simulated lighthouse timings (to test without physical lighthouse)
bool simulateLighthouse = false;

//if test is true, then run tests in Test.cpp and exit
bool test = false;

//mode of base station
//0:A, 1:B, 2: C
const int A = 0;
const int B = 1;
const int C = 2;
int baseStationMode = A;


//PoseTracker tracker(alphaImuFilter, baseStationMode, simulateLighthouse);

void setup() {

  Serial.begin(115200);

  delay(1000);
//END HW06

  tracker.initImu();

  //measures bias/variance
  testBiasVariance();

  //flatland roll tests
  testComputeFlatlandRoll();

  //quaternion library  tests
  testqSetFromAngleAxis();
  testqLength();
  testqNormalize();
  testqInverse();
  testqMultiply();
  testqRotate();

  //quaternion tracking tests
  testUpdateQuaternionGyr();
  testComputeAccPitch();
  testComputeAccRoll();
  testUpdateQuaternionComp();


  //test functions in PoseMath
  //the ..Simple tests are sanity check tests, 
  //eg rotation of 90 about axis
  //the other tests are checked with numerical results 

  testConvertTicksTo2DPositions();
  testFormASimple();
  testFormA();
  testSolveForHSimple();
  testSolveForH();
  testGetRtFromHSimple();
  testGetRtFromH();
  testGetQuaternionFromRotationMatrixSimple();
  testGetQuaternionFromRotationMatrix();

  // For beginning outputting quaternion to serial port in loop
  delay(5000);
  tracker.resetOrientation();
  tracker.measureImuBiasVariance();
}

void testBiasVariance() {
  tracker.measureImuBiasVariance();

  //print out gyr/acc bias
  Serial.printf("2.1.1 Bias Estimation\n");
  const double* gyrBias = tracker.getGyrBias();
  const double* accBias = tracker.getAccBias();

  Serial.printf("GYR_BIAS: %.5f %.5f %.5f\n", gyrBias[0], gyrBias[1], gyrBias[2]);
  Serial.printf("ACC_BIAS: %.5f %.5f %.5f\n", accBias[0], accBias[1], accBias[2]);
  Serial.println();

  //print out gyr/acc variance
  Serial.printf("2.1.2 Variance Estimation\n");
  const double* gyrVariance = tracker.getGyrVariance();
  const double* accVariance = tracker.getAccVariance();

  Serial.printf("GYR_VAR: %.5f %.5f %.5f\n", gyrVariance[0], gyrVariance[1], gyrVariance[2]);
  Serial.printf("ACC_VAR: %.5f %.5f %.5f\n", accVariance[0], accVariance[1], accVariance[2]);
  Serial.println();


}

void testComputeFlatlandRoll() {


  //test flatland roll from gyro
  double flatlandRollGyrPrev = 45;
  double gyr[3] = {0, 0, 90};
  double deltaT = 0.5;

  double act = computeFlatlandRollGyr(
    flatlandRollGyrPrev, gyr, deltaT);
  double exp = 90;
  bool pass = doubleEq(act, exp);

  Serial.printf("2.2.1 Flatland Gyro-Only: %d\n", pass);
  Serial.printf("exp: %.3f, act: %.3f\n", exp, act);
  Serial.println();


  //test flatland roll from acc
  double acc[3] = {1, 1, 0};
  double rollAcc = computeFlatlandRollAcc(acc);
  act = rollAcc;
  exp = 45;
  pass = doubleEq(act, exp);
  Serial.printf("2.2.2 Flatland Accelerometer-Only: %d\n", pass);
  Serial.printf("exp: %.3f, act: %.3f\n", exp, act);
  Serial.println();


  //test flatland roll from comp filter
  double alpha = 0.25;

  act = computeFlatlandRollComp(flatlandRollGyrPrev, gyr, rollAcc, deltaT, alpha);
  exp = 56.25;
  pass = doubleEq(exp, act);

  Serial.printf("2.2.3 Flatland Complementary Filter: %d\n", pass);
  Serial.printf("exp: %.3f, act: %.3f\n", exp, act);
  Serial.println();

}

void testUpdateQuaternionGyr() {

  //test simple rotation

  Quaternion q = Quaternion().setFromAngleAxis(-90,0,0,1);
  double gyr[3] = {0, 0, 180};
  double dT = 0.5;
  updateQuaternionGyr(q, gyr, dT);

  //expected quat
  Quaternion qExp = Quaternion(1, 0, 0, 0);

  bool pass = quaternionEq(qExp, q);

  Serial.printf("2.4.1 3D Gyro-Only Test#1: %d\n", pass);
  Serial.printf("qExp: %.3f %.3f %.3f %.3f\n",
    qExp.q[0], qExp.q[1], qExp.q[2], qExp.q[3]);
  Serial.printf("qAct: %.3f %.3f %.3f %.3f\n",
    q.q[0], q.q[1], q.q[2], q.q[3]);

  Serial.println();

  //test harder

  q = Quaternion().setFromAngleAxis(-90,0,0,1);
  gyr[0] = 180;
  gyr[1] = 0;
  gyr[2] = 180;
  dT = 0.5;
  updateQuaternionGyr(q, gyr, dT);

  //expected quat
  qExp = Quaternion(0.76198, 0.44801, -0.44801, 0.13404);

  pass = quaternionEq(qExp, q);

  Serial.printf("2.4.1 3D Gyro-Only Test#2: %d\n", pass);
  Serial.printf("qExp: %.3f %.3f %.3f %.3f\n",
    qExp.q[0], qExp.q[1], qExp.q[2], qExp.q[3]);
  Serial.printf("qAct: %.3f %.3f %.3f %.3f\n",
    q.q[0], q.q[1], q.q[2], q.q[3]);
  Serial.println();


}

/* setFromAngleAxis() */
void testqSetFromAngleAxis() {

  Quaternion q0 = Quaternion().setFromAngleAxis(
    2, 1 / sqrt(14), 2 / sqrt(14), 3 / sqrt(14));

  Quaternion qExp = Quaternion(
    0.999848, 0.004664, 0.009329, 0.013993);

  bool pass = quaternionEq(q0, qExp);

  Serial.printf("2.3.i setFromAngleAxis(): %d\n", pass);

  Serial.println("Expected constructed quaternion:");
  qExp.serialPrint();
  Serial.println("Actual: ");
  q0.serialPrint();
  Serial.println();
}


/* length() */
void testqLength() {

  Quaternion q = Quaternion(2.3, 1.2, 2.1, 3.0);
  double l = q.length();
  double exp = 4.487761;
  bool pass = doubleEq(l, exp);
  Serial.printf("2.3.ii length(): %d\n", pass);
  Serial.printf("exp: %.3f act: %.3f\n", exp, l);
  Serial.println();

}


/* normalize() */
void testqNormalize() {
  Quaternion q = Quaternion(2.3, 1.2, 2.1, 3.0);
  Quaternion qExp = Quaternion(
    0.512505, 0.267394, 0.467939, 0.668485);
  q.normalize();

  bool pass = quaternionEq(q, qExp);

  Serial.printf("2.3.iii normalize(): %d\n", pass);
  Serial.println("Expected normalized quaternion:");
  qExp.serialPrint();
  Serial.println("Actual: ");
  q.serialPrint();
  Serial.println();

}

/* inverse() */
void testqInverse() {
  Quaternion p = Quaternion(3.2, 3.3, 5.2, 0.1);
  Quaternion pExp = Quaternion(0.066418, -0.068493, -0.107929, -0.002076);
  p.inverse();
  bool pass = quaternionEq(p, pExp);

  Serial.printf("2.3.iv inverse(): %d\n", pass);

  Serial.println("Expected inverse quaternion:");
  pExp.serialPrint();
  Serial.println("Actual: ");
  p.serialPrint();
  Serial.println();

}

/* multiply() */
void testqMultiply() {

  Quaternion q1   = Quaternion(0.512505, 0.267394, 0.467939, 0.668485);
  Quaternion q2   = Quaternion(0.461017, -0.475423, -0.749152, -0.014407);
  Quaternion q1q2 = Quaternion().multiply(q1, q2);
  Quaternion qExp = Quaternion(
    0.723587, 0.373672, -0.482177, 0.322949);

  bool pass = quaternionEq(qExp, q1q2);
  Serial.printf("2.3.v multiply(): %d\n", pass);
  Serial.println("Expected multiplied quaternion:");
  qExp.serialPrint();
  Serial.println("Actual: ");
  q1q2.serialPrint();
  Serial.println();
}

  /* rotate() */
void testqRotate() {
  Quaternion q3 = Quaternion(0.512505, 0.267394, 0.467939, 0.668485);
  Quaternion q4 = Quaternion(0.461017, -0.475423, -0.749152, -0.014407);
  Quaternion q5 = q3.rotate(q4);
  Quaternion qExp = Quaternion(
    0.512505, -0.145908, 0.750596, -0.390712);

  bool pass = quaternionEq(q5, qExp);

  Serial.printf("2.3.vi rotate(): %d\n", pass);

  Serial.println("Expected multiplied quaternion:");
  qExp.serialPrint();
  Serial.println("Actual: ");
  q5.serialPrint();
  Serial.println();

}

void testComputeAccPitch() {

  double acc[3] = {0, 1, -1};
  double act = computeAccPitch(acc);
  double exp = 45;
  bool pass = doubleEq(act, exp);
  Serial.printf("2.4.2 3D Accelerometer Pitch Test#1: %d\n", pass);
  Serial.printf("exp: %.3f, act: %.3f\n", exp, act);

  Serial.println();

  acc[0] = 0;
  acc[1] = sqrt(3);
  acc[2] = 1;
  act = computeAccPitch(acc);
  exp = -30;
  pass = doubleEq(act, exp);
  Serial.printf("2.4.2 3D Accelerometer Pitch Test#2: %d\n", pass);
  Serial.printf("exp: %.3f, act: %.3f\n", exp, act);

  Serial.println();

}

void testComputeAccRoll() {

  double acc[3] = {1, 1, 0};
  double act = computeAccRoll(acc);
  double exp = 45;
  bool pass = doubleEq(act, exp);
  Serial.printf("2.4.2 3D Accelerometer Roll Test#1: %d\n", pass);
  Serial.printf("exp: %.3f, act: %.3f\n", exp, act);
  Serial.println();

  acc[0] = -1;
  acc[1] = sqrt(3);
  acc[2] = 0;
  act = computeAccRoll(acc);
  exp = -30;
  pass = doubleEq(act, exp);
  Serial.printf("2.4.2 3D Accelerometer Roll Test#2: %d\n", pass);
  Serial.printf("exp: %.3f, act: %.3f\n", exp, act);
  Serial.println();

}

void testUpdateQuaternionComp() {

  //test simple rotation
  Quaternion q = Quaternion().setFromAngleAxis(-180,0,0,1);
  double gyr[3] = {0, 0, 180};
  double dT = 0.5;
  double acc[3] = {0, 1, 0};
  double alpha = 0.75;
  Quaternion qExp = Quaternion().setFromAngleAxis(-0.75 * 90, 0, 0, 1);
  updateQuaternionComp(q, gyr, acc, dT, alpha);
  bool pass = quaternionEq(qExp, q);

  Serial.printf("2.4.3 3D Complementary Filter Test#1: %d\n", pass);
  Serial.printf("qExp: %.3f %.3f %.3f %.3f\n",
    qExp.q[0], qExp.q[1], qExp.q[2], qExp.q[3]);
  Serial.printf("qAct: %.3f %.3f %.3f %.3f\n",
    q.q[0], q.q[1], q.q[2], q.q[3]);
  Serial.println();


  //test harder
  q = Quaternion().setFromAngleAxis(-180,0,0,1);
  gyr[0] = 180;
  gyr[1] = 0;
  gyr[2] = 180;
  dT = 0.5;
  updateQuaternionGyr(q, gyr, dT);

  //expected quat
  qExp = Quaternion(0.634, 0.000, -0.634, -0.444);
  pass = quaternionEq(qExp, q);

  Serial.printf("2.4.3 3D Complementary Filter Test#2: %d\n", pass);
  Serial.printf("qExp: %.3f %.3f %.3f %.3f\n",
    qExp.q[0], qExp.q[1], qExp.q[2], qExp.q[3]);
  Serial.printf("qAct: %.3f %.3f %.3f %.3f\n",
    q.q[0], q.q[1], q.q[2], q.q[3]);
  Serial.println();

}

//HW06

void testConvertTicksTo2DPositions() {

  uint32_t clockTicks[8] = {299756, 172099, 110934, 329408, 
    116350, 248654, 213492, 143459};

  double pos2DAct[8];

  convertTicksTo2DPositions(clockTicks, pos2DAct);

  double pos2DExp[8] = {-0.996, -0.223, 0.841, 1.615, 0.771, 0.402, -0.106, -0.476};

  bool pass = matrixEq((double*) pos2DAct, (double*) pos2DExp, 1, 8);

  Serial.printf("2.1 convertTicksTo2DPositions(): %d\n", pass);
  Serial.printf("exp pos2D: ");
  printMatrix(pos2DExp, 1, 8);
  Serial.printf("act pos2D: ");
  printMatrix(pos2DAct, 1, 8);

  Serial.println();


}

void testFormA() {

  double pos2D[8] = {-0.996, -0.223, 0.841, 1.615, 0.771, 0.402, -0.106, -0.476};
  double posRef[8] = {-42.0, 25.0, 42.0, 25.0, 42.0, -25.0, -42.0, -25.0};
  double Aact[8][8];
  formA(pos2D, posRef, Aact);

  double Aexp[8][8] = {
    {-42.000, 25.000, 1.000, 0.000, 0.000, 0.000, -41.832, 24.900}, 
    {0.000, 0.000, 0.000, -42.000, 25.000, 1.000, -9.366, 5.575}, 
    {42.000, 25.000, 1.000, 0.000, 0.000, 0.000, -35.322, -21.025},
    {0.000, 0.000, 0.000, 42.000, 25.000, 1.000, -67.830, -40.375},
    {42.000, -25.000, 1.000, 0.000, 0.000, 0.000, -32.382, 19.275},
    {0.000, 0.000, 0.000, 42.000, -25.000, 1.000, -16.884, 10.050},
    {-42.000, -25.000, 1.000, 0.000, 0.000, 0.000, -4.452, -2.650},
    {0.000, 0.000, 0.000, -42.000, -25.000, 1.000, -19.992, -11.900}
  };

  bool pass = matrixEq((double*)Aact, (double*)Aexp, 8, 8);

  Serial.printf("2.2 Populate Matrix A: %d\n", pass);

  Serial.printf("exp A:\n");
  printMatrix((double*)Aexp, 8, 8);
  Serial.printf("act A:\n");
  printMatrix((double*)Aact, 8, 8);

  Serial.println();

}

void testFormASimple() {

  double pos2D[8] = {1, 1, 1, 1, 1, 1, 1, 1};
  double posRef[8] = {-42.0, 25.0, 42.0, 25.0, 42.0, -25.0, -42.0, -25.0};
  double Aact[8][8];
  formA(pos2D, posRef, Aact);

  double Aexp[8][8] = {
    {-42.000, 25.000, 1.000, 0.000, 0.000, 0.000, 42, -25}, 
    {0.000, 0.000, 0.000, -42.000, 25.000, 1.000, 42, -25}, 
    {42.000, 25.000, 1.000, 0.000, 0.000, 0.000, -42, -25},
    {0.000, 0.000, 0.000, 42.000, 25.000, 1.000, -42, -25},
    {42.000, -25.000, 1.000, 0.000, 0.000, 0.000, -42, 25},
    {0.000, 0.000, 0.000, 42.000, -25.000, 1.000, -42, 25},
    {-42.000, -25.000, 1.000, 0.000, 0.000, 0.000, 42, 25},
    {0.000, 0.000, 0.000, -42.000, -25.000, 1.000, 42, 25}
  };

  bool pass = matrixEq((double*)Aact, (double*)Aexp, 8, 8);

  Serial.printf("2.2 Populate Matrix A (simple): %d\n", pass);

  Serial.printf("exp A:\n");
  printMatrix((double*)Aexp, 8, 8);
  Serial.printf("act A:\n");
  printMatrix((double*)Aact, 8, 8);

  Serial.println();


}

void testSolveForH() {

  double A[8][8] = {

    {-42.000, 25.000, 1.000, 0.000, 0.000, 0.000, -41.832, 24.900}, 
    {0.000, 0.000, 0.000, -42.000, 25.000, 1.000, -9.366, 5.575}, 
    {42.000, 25.000, 1.000, 0.000, 0.000, 0.000, -35.322, -21.025},
    {0.000, 0.000, 0.000, 42.000, 25.000, 1.000, -67.830, -40.375},
    {42.000, -25.000, 1.000, 0.000, 0.000, 0.000, -32.382, 19.275},
    {0.000, 0.000, 0.000, 42.000, -25.000, 1.000, -16.884, 10.050},
    {-42.000, -25.000, 1.000, 0.000, 0.000, 0.000, -4.452, -2.650},
    {0.000, 0.000, 0.000, -42.000, -25.000, 1.000, -19.992, -11.900}
  };

  double b[8] = {-0.996, -0.223, 0.841, 1.615, 0.771, 0.402, -0.106, -0.476};

  double hAct[8];

  solveForH(A, b, hAct);

  double hExp[8] = {0.014, -0.010, 0.200, 0.014, 0.010, 0.200, -0.000, -0.014};

  bool pass = matrixEq((double*) hExp, (double*) hAct, 1, 8);

  Serial.printf("2.3 Solve for h: %d\n", pass);

  Serial.printf("exp h: \n");
  printMatrix(hExp, 1, 8);

  Serial.printf("act h: \n");
  printMatrix(hAct, 1, 8);

  Serial.println();
   
}

void testSolveForHSimple() {

  double A[8][8];
  //set A = diag(0.5, 0.5, ...)
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {

      if (i == j) {
        A[i][j] = 0.5;
      } else  {
        A[i][j] = 0;
      }

    }
  }

  double b[8] = {1,2,3,4,5,6,7,8};

  double hAct[8];

  solveForH(A, b, hAct);

  double hExp[8] = {2,4,6,8,10,12,14,16};

  bool pass = matrixEq((double*) hExp, (double*) hAct, 1, 8);

  Serial.printf("2.3 Solve for h (simple): %d\n", pass);

  Serial.printf("exp h: \n");
  printMatrix(hExp, 1, 8);

  Serial.printf("act h: \n");
  printMatrix(hAct, 1, 8);

  Serial.println();
   
}

void testGetRtFromH() {

  double h[8] = {0.014, -0.010, 0.200, 0.014, 0.010, 0.200, -0.000, -0.014};

  double RAct[3][3];
  double pos3DAct[3];
  getRtFromH(h, RAct, pos3DAct);

  double RExp[3][3] = {
    {0.707, -0.503, 0.497},
    {0.707, 0.503, -0.497},
    {0.000, 0.704, 0.711}
  };

  double pos3DExp[3] = {10.076, 10.076, -50.379};

  //check translation
  bool passT = matrixEq(pos3DExp, pos3DAct, 1, 3);
  
  Serial.printf("2.4 Get Position from h: %d\n", passT);

  Serial.printf("exp position: ");
  printMatrix((double*)pos3DExp, 1, 3);

  Serial.printf("act position: ");
  printMatrix((double*)pos3DAct, 1, 3);

  Serial.println();
  //check rotation
  bool passR = matrixEq((double*)RExp, (double*)RAct, 3, 3);

  Serial.printf("2.4 Get Rotation from h: %d\n", passR);
  Serial.printf("exp rotation:\n ");
  printMatrix((double*)RExp, 3, 3);

  Serial.printf("act rotation:\n ");
  printMatrix((double*)RAct, 3, 3);

  Serial.println();

}

void testGetRtFromHSimple() {

  double h[8] = {0.5, 0, 10, 0, 0.5, 10, 0, 0};

  double RAct[3][3];
  double pos3DAct[3];
  getRtFromH(h, RAct, pos3DAct);

  double RExp[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };

  double pos3DExp[3] = {20, 20, -2};

  //check translation
  bool passT = matrixEq(pos3DExp, pos3DAct, 1, 3);
  
  Serial.printf("2.4 Get Position from h (simple): %d\n", passT);

  Serial.printf("exp position: ");
  printMatrix((double*)pos3DExp, 1, 3);

  Serial.printf("act position: ");
  printMatrix((double*)pos3DAct, 1, 3);

  Serial.println();
  //check rotation
  bool passR = matrixEq((double*)RExp, (double*)RAct, 3, 3);

  Serial.printf("2.4 Get Rotation from h (simple): %d\n", passR);
  Serial.printf("exp rotation:\n ");
  printMatrix((double*)RExp, 3, 3);

  Serial.printf("act rotation:\n ");
  printMatrix((double*)RAct, 3, 3);

  Serial.println();

}

void testGetQuaternionFromRotationMatrix() {

  double R[3][3] = {
    {0.707, -0.50, 0.500},
    {0.707, 0.500, -0.500},
    {0.000, 0.707, 0.707}
  };

  Quaternion qAct = getQuaternionFromRotationMatrix(R);

  Quaternion qExp = Quaternion(0.854, 0.354, 0.146, 0.354);

  bool pass = quaternionEq(qAct, qExp);

  Serial.printf("2.5 Convert Rotation Matrix to Quaternion: %d\n", pass);

  Serial.printf("exp q: %.3f %.3f %.3f %.3f\n", 
    qExp.q[0], qExp.q[1], qExp.q[2], qExp.q[3]);

  Serial.printf("act q: %.3f %.3f %.3f %.3f\n", 
    qAct.q[0], qAct.q[1], qAct.q[2], qAct.q[3]);

  Serial.println();

}  

void testGetQuaternionFromRotationMatrixSimple() {

  //+90 around z-axis
  double R[3][3] = {
    {0, -1, 0},
    {1, 0, 0},
    {0, 0, 1}
  };

  Quaternion qAct = getQuaternionFromRotationMatrix(R);

  Quaternion qExp = Quaternion(0.7071, 0, 0, 0.7071);

  bool pass = quaternionEq(qAct, qExp);

  Serial.printf("2.5 Convert Rotation Matrix to Quaternion (simple): %d\n", pass);

  Serial.printf("exp q: %.3f %.3f %.3f %.3f\n", 
    qExp.q[0], qExp.q[1], qExp.q[2], qExp.q[3]);

  Serial.printf("act q: %.3f %.3f %.3f %.3f\n", 
    qAct.q[0], qAct.q[1], qAct.q[2], qAct.q[3]);
  
  Serial.println();


}  
//End HW06


//*********************************************  HELPER FUNCTIONS  *********************************************//
/** check if 2 quaternions are equal */
bool quaternionEq(Quaternion& q1, Quaternion& q2) {

  for (int i = 0; i < 4; i++) {
    if (!doubleEq(q1.q[i], q2.q[i])) {
      return false;
    }
  }

  return true;

}

bool doubleEq(double d1, double d2) {

  if (abs(d1 - d2) < 0.001) {
    return true;
  } else {
    return false;
  }

}

/** checks that 2 matrices are equal */
bool matrixEq(double * m1, double * m2, int r, int c) {

  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {

      int index = i * c + j;

      if (!doubleEq(m1[index], m2[index])) {
        return false;
      }

    }
  }

  return true;

}

void printMatrix(double * m, int r, int c) {

  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {
      Serial.printf("%.3f ", m[i * c + j]);
    }
    Serial.println();
  }

}
    

void updatePosePrint() {

  Serial.println();
  uint32_t clockTicks[8] = {299756, 172099, 110934, 329408,
    116350, 248654, 213492, 143459};
  double pos2D[8];

  Serial.printf("CT: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%d ", clockTicks[i]);
  }
  Serial.println();

  convertTicksTo2DPositions(clockTicks, pos2D);

  Serial.printf("P2D: ");
  printMatrix(pos2D, 1, 8);
  Serial.println();
  
  double posRef[8] = {-42.0, 25.0, 42.0, 25.0, 42.0, -25.0, -42.0, -25.0};
  double A[8][8];
  formA(pos2D, posRef, A);
  Serial.printf("A:\n");
  printMatrix((double*) A, 8, 8);
  Serial.println();

  double h[8];
  solveForH(A, pos2D, h);
  Serial.printf("h: \n");
  printMatrix(h, 1, 8);

  Serial.println();
  
  double R[3][3];
  double pos3D[3];
  getRtFromH(h, R, pos3D);
  Serial.printf("pos3D: ");
  printMatrix(pos3D, 1, 3);
  Serial.println();
  
  Serial.println();
  Serial.printf("R:\n");
  printMatrix((double*) R, 3, 3);
  Serial.println();

  Quaternion q = getQuaternionFromRotationMatrix(R); 
  Serial.printf("QH: %.3f %.3f %.3f %.3f\n", 
    q.q[0], q.q[1], q.q[2], q.q[3]);

}
  



void loop() {
  if (!test)
  {
    bool imuTrack = false;

    imuTrack = tracker.processImu();

    const Quaternion& quaternionComp = tracker.getQuaternionComp();
    // const double *gyro = tracker.getGyr();
    // const double *acc = tracker.getAcc();

    if (imuTrack == 1) {
      //print quaternion from imu
      Serial.printf("QC %.3f %.3f %.3f %.3f\n",
      quaternionComp.q[0], quaternionComp.q[1],
      quaternionComp.q[2], quaternionComp.q[3]);

    }

    // Serial.printf("Gyro %.3f %.3f %.3f\n",
    // gyro[0], gyro[1], gyro[2]);
    // Serial.printf("Acc %.3f %.3f %.3f\n",
    // acc[0], acc[1], acc[2]);

    //delay(100);
  }

}
