#include "OrientationMath.h"

//TODO: fill in from hw 4 as necessary

int sign(float num){
  if (num > 0.0){
    return 1;
  }
  else if (num < 0.0)
  {
    return -1;
  }
  else{
    return 0;
  }
}

/** TODO: see documentation in header file */
double computeAccPitch(double acc[3]) {

  return -(180.0/PI) * atan2(acc[2], sign(acc[1])*sqrt(pow(acc[0],2)+pow(acc[1],2)));

}

/** TODO: see documentation in header file */
double computeAccRoll(double acc[3]) {

  return -(180.0/PI) * atan2(-acc[0], acc[1]);

}

/** TODO: see documentation in header file */
double computeFlatlandRollGyr(double flatlandRollGyrPrev, double gyr[3], double deltaT) {

  return flatlandRollGyrPrev + gyr[2]*deltaT;

}

/** TODO: see documentation in header file */
double computeFlatlandRollAcc(double acc[3]) {

  return (180.0/PI) * atan2(acc[0], acc[1]);

}

/** TODO: see documentation in header file */
double computeFlatlandRollComp(double flatlandRollCompPrev, double gyr[3], double flatlandRollAcc, double deltaT, double alpha) {

  return alpha * computeFlatlandRollGyr(flatlandRollCompPrev, gyr, deltaT) + (1.0 - alpha) * flatlandRollAcc;

}


/** TODO: see documentation in header file */
void updateQuaternionGyr(Quaternion& q, double gyr[3], double deltaT) {
  // q is the previous quaternion estimate
  // update it to be the new quaternion estimate

  double gyr_mag = sqrt(pow(gyr[0], 2) + pow(gyr[1], 2) + pow(gyr[2], 2));
  if (gyr_mag < pow(10.0,-8)){
    return; // Ignore measurement if magnitude of gyro measurement is close to 0
  }
  double theta = deltaT*gyr_mag; // deg
  double axis[3] = {gyr[0]/gyr_mag, gyr[1]/gyr_mag, gyr[2]/gyr_mag};

  Quaternion q_delta = Quaternion().setFromAngleAxis(theta, axis[0], axis[1], axis[2]).normalize();

  q = q.multiply(q, q_delta).normalize();

}


/** TODO: see documentation in header file */
void updateQuaternionComp(Quaternion& q, double gyr[3], double acc[3], double deltaT, double alpha) {
  // q is the previous quaternion estimate
  // update it to be the new quaternion estimate

  double gyr_mag = sqrt(pow(gyr[0], 2) + pow(gyr[1], 2) + pow(gyr[2], 2));
  if (gyr_mag < pow(10.0,-8)){
    return; // Ignore measurement if magnitude of gyro measurement is close to 0
  }
  double theta = deltaT*gyr_mag; // deg
  double axis[3] = {gyr[0]/gyr_mag, gyr[1]/gyr_mag, gyr[2]/gyr_mag};
  Quaternion q_delta = Quaternion().setFromAngleAxis(theta, axis[0], axis[1], axis[2]).normalize();

  // Multiply previous complementary filter quaternion by q_delta
  q = q.multiply(q, q_delta).normalize(); // This is q_w_next of next update (t+deltaT)

  Quaternion q_a_body = Quaternion(0.0, acc[0], acc[1], acc[2]);
  Quaternion q_a_world = q_a_body.rotate(q).normalize();

  double q_a_world_len = q_a_world.length();
  double v[3] = {q_a_world.q[1]/q_a_world_len, q_a_world.q[2]/q_a_world_len, q_a_world.q[3]/q_a_world_len};

  double phi = (180.0/PI) * acos(v[1]); // deg; Angle between q_a_world and q_up_world; v[1] is dot product between v and q_up_world a (0,1,0) vector in world space

  double n[3] = {-v[2], 0.0, v[0]}; // Axis between q_a_world and q_up_world; cross product between v and q_up_world a (0,1,0) vector in world space
  double n_len = sqrt(pow(n[0],2) + pow(n[1],2) + pow(n[2],2));
  double n_norm[3] = {n[0]/n_len, n[1]/n_len, n[2]/n_len}; // Normalize n

  Quaternion q_t_alpha = Quaternion().setFromAngleAxis((1-alpha)*phi, n_norm[0], n_norm[1], n_norm[2]).normalize(); // Scaled tilt correction quaternion 
  q = q.multiply(q_t_alpha, q).normalize(); // Apply tilt correction

}
