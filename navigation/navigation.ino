
#include "navigation.h"

void Navigation::quat_to_DCM() {
  // convert attitude quaternion to DCM

  float q1, q2, q3, q4;
  float q1_2, q2_2, q3_2, q4_2;
  float q1q2, q1q3, q1q4, q2q3, q2q4, q3q4;

  q1 = ahrs.quat_ENU_to_B(0); 
  q2 = ahrs.quat_ENU_to_B(1); 
  q3 = ahrs.quat_ENU_to_B(2); 
  q4 = ahrs.quat_ENU_to_B(3);

  // for calculation
  q1_2 = q1*q1; q2_2 = q2*q2; q3_2 = q3*q3; q4_2 = q4*q4;
  q1q2 = q1*q2; q1q3 = q1*q3; q1q4 = q1*q4;
  q2q3 = q2*q3; q2q4 = q2*q4; q3q4 = q3*q4; 

  // construct elements of DCM
  ahrs.DCM_ENU_to_B(0,0) = q1_2 - q2_2 - q3_2 + q4_2;
  ahrs.DCM_ENU_to_B(0,1) = 2*(q1q2 + q3q4);
  ahrs.DCM_ENU_to_B(0,2) = 2*(q1q3 - q2q4);

  ahrs.DCM_ENU_to_B(1,0) = 2*(q1q2 - q3q4);
  ahrs.DCM_ENU_to_B(1,1) = -q1_2 + q2_2 - q3_2 + q4_2;
  ahrs.DCM_ENU_to_B(1,2) = 2*(q2q3 + q1q4);

  ahrs.DCM_ENU_to_B(2,0) = 2*(q1q3 + q2q4);
  ahrs.DCM_ENU_to_B(2,1) = 2*(q2q3 - q1q4);
  ahrs.DCM_ENU_to_B(2,2) = -q1_2 - q2_2 + q3_2 + q4_2;

  return;
}


void Navigation::acc_imu_to_acc_cg() {
  // acc_imu_B : linear acceleration from bno055 represented in body frame
  Vector3f cross_w_dot_r_IMU_B;
  Vector3f cross_w_r_IMU_B;
  Vector3f cross_w_cross_w_r_IMU_B;

  cross_w_dot_r_IMU_B = ahrs.angular_acc_B.cross(TU1f.r_IMU_B);
  cross_w_r_IMU_B = ahrs.angular_rate_filtered_B.cross(TU1f.r_IMU_B);
  cross_w_cross_w_r_IMU_B = ahrs.angular_rate_filtered_B.cross(cross_w_r_IMU_B);

  ahrs.acc_cg_B = ahrs.acc_imu_B - cross_w_dot_r_IMU_B - cross_w_cross_w_r_IMU_B;
  return;
}

void Navigation::acc_B_to_ENU() {
  ahrs.acc_cg_ENU = ahrs.DCM_ENU_to_B.transpose() * ahrs.acc_cg_B;
  return;
}

float Navigation::lat_geocent_to_geodet(float geocent_lat_rad) {
  float f = fltn;
  float geodet_lat_rad;

  geodet_lat_rad = atan2(tan(geocent_lat_rad), (1-f) * (1-f));

  return geodet_lat_rad; // rad
}

void Navigation::geodetic_lla_to_ECEF() {
  float geodet_lat, lon, alt;
  geodet_lat = pos_gps_lla[0]* DEG_TO_RAD; // rad
  lon = pos_gps_lla[1]* DEG_TO_RAD; // rad
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

void Navigation::initializeVehicleConfig(float m_dry, float C_D0, float d_ref, float r_IMU_b[]) {
  // initialize vehicle configuration
  // input
  // m_dry: drymass, in kg
  // C_D0: zero lift drag coefficient
  // d_ref: reference diameter, in m
  // r_IMU_bx, r_IMU_by, r_IMU_bz: location of imu w.r.t. cg, in m
  
  // example values
  // m_dry = 3.7; // kg, dry mass of the vehicle after burnout
  // C_D0 = 0.3; // -, zero lift drag coefficient of the vehicle
  // d_ref = 0.104; // m, reference length (diameter of fuselage)
  // r_IMU_b[3] = {0.25, 0, 0}; // m, cg to imu poistion vector (in body frame)

  TU1f.m_dry = m_dry; // kg
  TU1f.C_D0 = C_D0; // -
  TU1f.d_ref = d_ref; // m
  TU1f.S_ref = PI/4 * d_ref * d_ref; // m^2
  TU1f.r_IMU_b << r_IMU_b[0], r_IMU_b[1], r_IMU_b[2]; // m
  
  return;
}

void Navigation::initializeLPFConfig(float alpha) {

  return;
}