
#include "navigation.h"

DCM Navigation::quat_to_DCM(float* quat) {
  DCM dcm;
  float q0, q1, q2, q3;
  float q0q0, q1q1, q2q2, q3q3, q0q1, q0q2, q0q3, q1q2, q1q3, q2q3;

  q0 = quat[3]; q1 = quat[0]; q2 = quat[1]; q3 = quat[2];

  // for calculation
  q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;
  q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3; q1q2 = q1*q2; q1q3 = q1*q3; q2q3 = q2*q3;

  // construct elements of DCM
  dcm.a11 = q0q0 + q1q1 - q2q2 - q3q3;
  dcm.a12 = 2*(q1q2 + q0q3);
  dcm.a13 = 2*(q1q3 - q0q2);

  dcm.a21 = 2*(q1q2 - q0q3);
  dcm.a22 = q0q0 - q1q1 + q2q2 - q3q3;
  dcm.a23 = 2*(q2q3 + q0q1);

  dcm.a31 = 2*(q1q3 + q0q2);
  dcm.a32 = 2*(q2q3 - q0q1);
  dcm.a33 = q0q0 - q1q1 - q2q2 + q3q3;

  return dcm;

}


void Navigation::convert_imu_acc_to_body_acc() {
  float cross_w_dot_r_IMU_b[3];
  float cross_w_r_IMU_b[3];
  float cross_w_cross_w_r_IMU_b[3];

  crossProduct(angular_acc, r_IMU_b, cross_w_dot_r_IMU_b);
  crossProduct(angular_rate_filtered, r_IMU_b, cross_w_r_IMU_b);
  crossProduct(angular_rate_filtered, cross_w_r_IMU_b, cross_w_cross_w_r_IMU_b);

  body_acc[0] = linear_acc[0] - cross_w_dot_r_IMU_b[0] - cross_w_cross_w_r_IMU_b[0];
  body_acc[1] = linear_acc[1] - cross_w_dot_r_IMU_b[1] - cross_w_cross_w_r_IMU_b[1];
  body_acc[2] = linear_acc[2] - cross_w_dot_r_IMU_b[2] - cross_w_cross_w_r_IMU_b[2];

  return;
}




void Navigation::crossProduct(float v_A[], float v_B[], float c_P[]) {
   c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
   c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
   c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];

   return;
}

void Navigation::matrixVecMult(DCM T, float b[], float res[]) {
  float t11, t12, t13, t21, t22, t23, t31, t32, t33;
  float b1, b2, b3;

  b1 = b[0];
  b2 = b[1];
  b3 = b[2];

  t11 = T.a11;
  t12 = T.a12;
  t13 = T.a13;
  t21 = T.a21;
  t22 = T.a22;
  t23 = T.a23;
  t31 = T.a31;
  t32 = T.a32;
  t33 = T.a33;

  res[0] = t11 * b1 + t12 * b2 + t13 * b3;
  res[1] = t21 * b1 + t22 * b2 + t23 * b3;
  res[2] = t31 * b1 + t32 * b2 + t33 * b3;

  return;
}

void Navigation::matrixTranspose(DCM A, DCM B) {

  B.a11 = A.a11;
  B.a12 = A.a21;
  B.a13 = A.a31;

  B.a21 = A.a12;
  B.a22 = A.a22;
  B.a23 = A.a32;

  B.a31 = A.a13;
  B.a32 = A.a23;
  B.a33 = A.a33;

  return;
}

void Navigation::acc_body_to_ENU() {
  DCM dcm_from_ENU_to_body, dcm_from_body_to_ENU;
  
  dcm_from_ENU_to_body = quat_to_DCM(quat_ENU_to_b); // convert quaternion to DCM
  matrixTranspose(dcm_from_ENU_to_body, dcm_from_body_to_ENU); // dcm_from_body_to_ENU = dcm_from_ENU_to_body^T
  matrixVecMult(dcm_from_body_to_ENU, body_acc, body_acc_ENU); // body_acc_ENU = dcm_from_body_to_ENU * body_acc
  
  return;
}

float Navigation::lat_geocent_to_geodet(float geocent_lat_rad) {
  float f = fltn;
  float geodet_lat_rad;

  geodet_lat_rad = atan2(tan(geocent_lat_rad), (1-f) * (1-f));

  return geodet_lat_rad; // rad
}

void Navigation::lla_to_ecef() {
  float geodet_lat, lon, alt;
  geodet_lat = pos_gps_lla[0]* PI/180; // rad
  lon = pos_gps_lla[1]* PI/180; // rad
  alt = pos_gps_lla[2]; // m

  float f = fltn;
  float b_e = a_e * (1 - f); // m
  float r = a_e * a_e / sqrt(a_e * a_e + b_e * b_e * tan(geodet_lat) * tan(geodet_lat));

  pos_gps_ECEF[0] = (r + alt * cos(geodet_lat)) * cos(lon);
  pos_gps_ECEF[1] = (r + alt * cos(geodet_lat)) * sin(lon);
  pos_gps_ECEF[2] = (b_e * b_e / a_e / a_e) * tan(geodet_lat) * r + alt * sin(geodet_lat);
  return;
}

void Navigation::ecef_to_enu() {
  float DTR = PI / 180;

  float phi = launch_site_lat_lon[0] * DTR;
  float lam = launch_site_lat_lon[1] * DTR;

  float sLam = sin(lam);
  float cLam = cos(lam);
  float sPhi = sin(phi);
  float cPhi = cos(phi);

  dcm_ecef_to_enu.a11 = -sLam;
  dcm_ecef_to_enu.a12 = cLam;
  dcm_ecef_to_enu.a13 = 0;
  dcm_ecef_to_enu.a21 = -sPhi * cLam;
  dcm_ecef_to_enu.a22 = -sPhi * sLam;
  dcm_ecef_to_enu.a23 = cPhi;
  dcm_ecef_to_enu.a31 = cPhi * cLam;
  dcm_ecef_to_enu.a32 = cPhi * sLam;
  dcm_ecef_to_enu.a33 = sPhi;

  float f = fltn;
  float b_e = a_e * (1 - f); // m
  float r = a_e * a_e / sqrt(a_e * a_e + b_e * b_e * tan(phi) * tan(phi));

  launch_site_ECEF[0] = r * cLam;
  launch_site_ECEF[1] = r * cLam;
  launch_site_ECEF[2] = (b_e * b_e / a_e / a_e) * tan(phi) * r;

  float rel_pos_ECEF[3];
  rel_pos_ECEF[0] = pos_gps_ECEF[0] - launch_site_ECEF[0];
  rel_pos_ECEF[1] = pos_gps_ECEF[1] - launch_site_ECEF[1];
  rel_pos_ECEF[2] = pos_gps_ECEF[2] - launch_site_ECEF[2];

  matrixVecMult(dcm_ecef_to_enu, rel_pos_ECEF, pos_gps_ENU);

}

void Navigation::updateSensorValue(SensorData newSensorData) {
  // current val -> prev val before getting new values
  t_AHRS_prev = t_AHRS;
  angular_rate_filtered_prev[0] = angular_rate_filtered[0];
  angular_rate_filtered_prev[1] = angular_rate_filtered[1];
  angular_rate_filtered_prev[2] = angular_rate_filtered[2];

  r_ENU_prev[0] = r_ENU_curr[0];
  r_ENU_prev[1] = r_ENU_curr[1];
  r_ENU_prev[2] = r_ENU_curr[2];

  v_ENU_prev[0] = v_ENU_curr[0];
  v_ENU_prev[1] = v_ENU_curr[1];
  v_ENU_prev[2] = v_ENU_curr[2];

  // These values should be updated on each navigation update.
  // // bno055
  // t_AHRS 
  // quat_ENU_to_b 
  // linear_acc
  // angular_rate

  // // GPS
  // t_GPS 
  // pos_gps_lla

  // // bmp280
  // t_baro 
  // h_baro

  t_AHRS = newSensorData.t_AHRS;
  quat_ENU_to_b[0] = newSensorData.quat_ENU_to_b[0];
  quat_ENU_to_b[1] = newSensorData.quat_ENU_to_b[1];
  quat_ENU_to_b[2] = newSensorData.quat_ENU_to_b[2];
  linear_acc[0] = newSensorData.linear_acc[0];
  linear_acc[1] = newSensorData.linear_acc[1];
  linear_acc[2] = newSensorData.linear_acc[2];
  angular_rate[0] = newSensorData.angular_rate[0];
  angular_rate[1] = newSensorData.angular_rate[1];
  angular_rate[2] = newSensorData.angular_rate[2];


  t_GPS = newSensorData.t_GPS;
  pos_gps_lla[0] = newSensorData.pos_gps_lla[0];
  pos_gps_lla[1] = newSensorData.pos_gps_lla[1];
  pos_gps_lla[2] = newSensorData.pos_gps_lla[2];

  t_baro = newSensorData.t_baro;
  h_baro = newSensorData.h_baro;

  // filter angular rate
  angular_rate_filtered[0] = alpha * angular_rate[0] + (1 - alpha) * angular_rate_filtered_prev[0];
  angular_rate_filtered[1] = alpha * angular_rate[1] + (1 - alpha) * angular_rate_filtered_prev[1];
  angular_rate_filtered[2] = alpha * angular_rate[2] + (1 - alpha) * angular_rate_filtered_prev[2];

  // calc omega dot
  dt = t_AHRS - t_AHRS_prev;
  float dt_sec = dt * 0.001f;
  angular_acc[0] = (angular_rate_filtered[0] - angular_rate_filtered_prev[0]) / dt_sec; // rad/s^2
  angular_acc[1] = (angular_rate_filtered[1] - angular_rate_filtered_prev[1]) / dt_sec; // rad/s^2
  angular_acc[2] = (angular_rate_filtered[2] - angular_rate_filtered_prev[2]) / dt_sec; // rad/s^2

  convert_imu_acc_to_body_acc();
  acc_body_to_ENU();

  lla_to_ecef();
  ecef_to_enu();

  return;
}

void Navigation::updateNavigation() {
  unsigned long inc_t;
  if (t_GPS > t_AHRS) inc_t = t_GPS - t_AHRS;
  else inc_t = t_AHRS - t_GPS;
  
  if (inc_t < dt) {

    // assume current gps position as a navigation solution
    r_ENU_curr[0] = pos_gps_ENU[0];
    r_ENU_curr[1] = pos_gps_ENU[1];
    r_ENU_curr[2] = pos_gps_ENU[2];

    // assume prev velocity solution is still usable at current time step (because of pos discontinuity)
    v_ENU_curr[0] = v_ENU_prev[0];
    v_ENU_curr[1] = v_ENU_prev[1];
    v_ENU_curr[2] = v_ENU_prev[2];

    return;
  }

  float dt_sec, dt_sec_2;
  dt_sec = dt * 0.001f;
  dt_sec_2 = dt_sec * dt_sec;

  // predict with Euler integration
  v_ENU_curr[0] = v_ENU_prev[0] + body_acc_ENU[0] * dt_sec;
  v_ENU_curr[1] = v_ENU_prev[1] + body_acc_ENU[1] * dt_sec; 
  v_ENU_curr[2] = v_ENU_prev[2] + body_acc_ENU[2] * dt_sec;

  r_ENU_curr[0] = r_ENU_prev[0] + v_ENU_prev[0] * dt_sec + 0.5 * body_acc_ENU[0] * dt_sec_2;
  r_ENU_curr[1] = r_ENU_prev[1] + v_ENU_prev[1] * dt_sec + 0.5 * body_acc_ENU[1] * dt_sec_2;
  r_ENU_curr[2] = r_ENU_prev[2] + v_ENU_prev[2] * dt_sec + 0.5 * body_acc_ENU[2] * dt_sec_2;

  // estimate drift
  float drift;
  drift = r_ENU_curr[2] - h_baro; // m

  // correct navigation solution
  r_ENU_curr[0] = r_ENU_curr[0] - drift;
  r_ENU_curr[1] = r_ENU_curr[1] - drift;
  r_ENU_curr[2] = r_ENU_curr[2] - drift;

  return;
}

ApogeeEstimate Navigation::getChudinovApogeeEst() {
  float theta_0; // rad

  float v1, v2, v3; // m/s

  v1 = v_ENU_curr[0];
  v2 = v_ENU_curr[1];
  v3 = v_ENU_curr[2];

  float dot, tmp;
  dot = v1 * v1 + v2 * v2;
  tmp = dot / sqrt(v1*v1 + v2*v2 + v3*v3) / sqrt(dot);
  theta_0 = acos(tmp); // rad

  float r3; // m
  r3 = r_ENU_curr[2];
  float rho = 1.225 * exp(-r3/10.4/1000); // kg/m3
  float k = rho * C_D0 * S_ref /2 / m_dry / g;

  float vsqured;
  vsqured = v1*v1 + v2*v2 + v3*v3;

  float H_apogee, H_local;
  H_apogee = r3 + vsqured * (sin(theta_0))*(sin(theta_0)) / g / (2 + k * sin(theta_0) * vsqured); // m
  H_local = H_apogee - r3;
  
  float T;
  T = 2 * sqrt(2 * H_local / g);

  float V_apogee;
  
  tmp = 1 + k*vsqured*(sin(theta_0)+cos(theta_0)*cos(theta_0)*log(tan(theta_0/2 + PI/4)));
  V_apogee = sqrt(vsqured) * cos(theta_0) / sqrt(tmp);

  float t_apogee;
  t_apogee = (T - k * H_local * V_apogee) / 2;

  ApogeeEstimate res;
  res.H_apogee = H_apogee;
  res.t_apogee = t_apogee;
  res.V_apogee = V_apogee;

  return res;
}