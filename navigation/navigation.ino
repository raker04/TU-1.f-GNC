
#include "navigation.h"

void Navigation::quat_to_DCM(float quat[4], float dcm[3][3]) {
  float q0, q1, q2, q3;
  float q0q0, q1q1, q2q2, q3q3, q0q1, q0q2, q0q3, q1q2, q1q3, q2q3;

  q0 = quat[3]; q1 = quat[0]; q2 = quat[1]; q3 = quat[2];

  // for calculation
  q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;
  q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3; q1q2 = q1*q2; q1q3 = q1*q3; q2q3 = q2*q3;

  // construct elements of DCM
  dcm[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
  dcm[0][1] = 2*(q1q2 + q0q3);
  dcm[0][2] = 2*(q1q3 - q0q2);

  dcm[1][0] = 2*(q1q2 - q0q3);
  dcm[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
  dcm[1][2] = 2*(q2q3 + q0q1);

  dcm[2][0] = 2*(q1q3 + q0q2);
  dcm[2][1] = 2*(q2q3 - q0q1);
  dcm[2][2] = q0q0 - q1q1 - q2q2 + q3q3;

  return;
}

void Navigation::lla_to_ECEF(float lat_deg, float lon_deg, float alt_wgs84, float r_ECEF_m[3]) {
  float phi, lam, h;
  phi = lat_deg * DEG_TO_RAD; // rad, latitude
  lam = lon_deg * DEG_TO_RAD; // rad, longitude
  h = alt_wgs84; // m, ellipsoidal altitude (wgs84)

  float sinLam, cosLam, sinPhi, cosPhi;
  sinLam = sin(lam);
  cosLam = cos(lam);
  sinPhi = sin(phi);
  cosPhi = cos(phi);

  float N;
  N = a_e / sqrt(1 - e_2 * sinLam * sinLam);

  r_ECEF_m[0] = (N + h) * cosPhi * cosLam; // m
  r_ECEF_m[1] = (N + h) * cosPhi * sinLam; // m
  r_ECEF_m[2] = ((1-e_2)*N + h) * sinPhi; // m

  return;
}

void Navigation::get_dcm_ECEF_to_ENU(float lat_deg, float lon_deg, float dcm_ECEF_to_ENU[3][3]) {
  float phi = lat_deg * DEG_TO_RAD;
  float lam = lon_deg * DEG_TO_RAD;

  float sLam = sin(lam);
  float cLam = cos(lam);
  float sPhi = sin(phi);
  float cPhi = cos(phi);

  dcm_ECEF_to_ENU[0][0] = -sLam;
  dcm_ECEF_to_ENU[0][1] = cLam;
  dcm_ECEF_to_ENU[0][2]= 0;
  dcm_ECEF_to_ENU[1][0] = -sPhi * cLam;
  dcm_ECEF_to_ENU[1][1] = -sPhi * sLam;
  dcm_ECEF_to_ENU[1][2] = cPhi;
  dcm_ECEF_to_ENU[2][0] = cPhi * cLam;
  dcm_ECEF_to_ENU[2][1] = cPhi * sLam;
  dcm_ECEF_to_ENU[2][2] = sPhi;

  return;
}

void Navigation::initializeLaunchSiteConfig(float lat_deg, float lon_deg, float alt_orthometric_m, float geoid_separation_m, float atm_pressure_Pa, float atm_temp_K) {
  // update launch site config attributes
  lat_deg_launch_site = lat_deg;
  lon_deg_launch_site = lon_deg;
  alt_orthometric_launch_site = alt_orthometric_m;
  geoid_separation_launch_site = geoid_separation_m;
  atm_pressure_launch_site = atm_pressure_Pa;
  atm_temp_launch_site = atm_temp_K;

  // calculate derived attributes
  alt_wgs84_launch_site = alt_orthometric_launch_site + geoid_separation_launch_site;
  lla_to_ECEF(lat_deg_launch_site, lon_deg_launch_site, alt_wgs84_launch_site, r_launch_site_ECEF);
  get_dcm_ECEF_to_ENU(lat_deg_launch_site, lon_deg_launch_site, dcm_ECEF_to_ENU);
  return;
}

void Navigation::updateAHRSMeasurement(uint32_t t_ms, float* imu_data) {
  t_AHRS_prev_msec = t_AHRS_msec;
  angular_rate_filtered_prev_B[0] = angular_rate_filtered_B[0];
  angular_rate_filtered_prev_B[1] = angular_rate_filtered_B[1];
  angular_rate_filtered_prev_B[2] = angular_rate_filtered_B[2];

  // update AHRS measurement attributes (copy pointer)
  t_AHRS_msec = t_ms;
  quat_ENU_to_B = imu_data + IMUSensor::QUAT_X;
  acc_imu_B = imu_data + IMUSensor::ACC_X;
  angular_rate_B = imu_data + IMUSensor::ANG_VEL_X;

  // calculate derived attributes
  dt_AHRS_msec = t_AHRS_msec - t_AHRS_prev_msec; // ms
  dt_AHRS_sec = dt_AHRS_msec * 0.001f; // sec
  quat_to_DCM(quat_ENU_to_B, dcm_ENU_to_B); // get dcm_ENU_to_B from quat_ENU_to_B
  matrixTranspose(dcm_ENU_to_B, dcm_B_to_ENU); // dcm_B_to_ENU = dcm_ENU_to_B^T
  angular_rate_filtered_B[0] = alpha * angular_rate_B[0] + (1 - alpha) * angular_rate_filtered_prev_B[0];
  angular_rate_filtered_B[1] = alpha * angular_rate_B[1] + (1 - alpha) * angular_rate_filtered_prev_B[1];
  angular_rate_filtered_B[2] = alpha * angular_rate_B[2] + (1 - alpha) * angular_rate_filtered_prev_B[2];
  angular_acc_B[0] = (angular_rate_filtered_B[0] - angular_rate_filtered_prev_B[0]) / dt_AHRS_sec; // rad/s2
  angular_acc_B[1] = (angular_rate_filtered_B[1] - angular_rate_filtered_prev_B[1]) / dt_AHRS_sec; // rad/s2
  angular_acc_B[2] = (angular_rate_filtered_B[2] - angular_rate_filtered_prev_B[2]) / dt_AHRS_sec; // rad/s2
  
  float cross_w_dot_r_imu_B[3];
  float cross_w_r_imu_B[3];
  float cross_w_cross_w_r_imu_B[3];

  crossProduct(angular_acc_B, r_imu_B, cross_w_dot_r_imu_B);
  crossProduct(angular_rate_filtered_B, r_imu_B, cross_w_r_imu_B);
  crossProduct(angular_rate_filtered_B, cross_w_r_imu_B, cross_w_cross_w_r_imu_B);

  acc_body_B[0] = acc_imu_B[0] - cross_w_dot_r_imu_B[0] - cross_w_cross_w_r_imu_B[0];
  acc_body_B[1] = acc_imu_B[1] - cross_w_dot_r_imu_B[1] - cross_w_cross_w_r_imu_B[1];
  acc_body_B[2] = acc_imu_B[2] - cross_w_dot_r_imu_B[2] - cross_w_cross_w_r_imu_B[2];

  matrixVecMult(dcm_B_to_ENU, acc_body_B, acc_body_ENU);

  return;
}

void Navigation::updateBMPMeasurement(float p_baro_hPa) {
  p_baro = p_baro_hPa * 100;
  // calculate altitude increment from pressure measurement
  inc_h_baro = (R * atm_temp_launch_site / g0) * log(atm_pressure_launch_site / p_baro); // m 

  return;
}

void Navigation::updateGPSMeasurement(float* gps_data) {
  float new_wgs84_alt = gps_data[GPSSensor::ALTITUDE] + gps_data[GPSSensor::GEOID_HEIGHT];
  isGPSUpdated = (new_wgs84_alt != alt_wgs84_body); // gps is updated if prev alt is different from the new alt

  if (!isGPSUpdated) return; // if gps is not updated simply just return.

  // calculate derived attributes
  alt_wgs84_body = gps_data[GPSSensor::ALTITUDE] + gps_data[GPSSensor::GEOID_HEIGHT];
  lla_to_ECEF(gps_data[GPSSensor::LATITUDE], gps_data[GPSSensor::LONGITUDE], alt_wgs84_body, r_body_gps_ECEF);
  r_body_rel_gps_ECEF[0] = r_body_gps_ECEF[0] - r_launch_site_ECEF[0];
  r_body_rel_gps_ECEF[1] = r_body_gps_ECEF[1] - r_launch_site_ECEF[1];
  r_body_rel_gps_ECEF[2] = r_body_gps_ECEF[2] - r_launch_site_ECEF[2];
  matrixVecMult(dcm_ECEF_to_ENU, r_body_rel_gps_ECEF, r_body_gps_ENU);

  return;
}

void Navigation::update(SensorDataCollection& newSensorData) {
  updateSensorValues(newSensorData);
  updateNavigation();
  return;
}

void Navigation::updateSensorValues(SensorDataCollection& newSensorData) {
  updateAHRSMeasurement(newSensorData.current_time, newSensorData.imu_data);
  updateBMPMeasurement(newSensorData.barometer_data[BarometerSensor::PRESSURE]);
  updateGPSMeasurement(newSensorData.gps_data);
  return;
}

void Navigation::updateNavigation() {
  // transfer curr value to prev value
  r_body_prev_ENU[0] = r_body_curr_ENU[0];
  r_body_prev_ENU[1] = r_body_curr_ENU[1];
  r_body_prev_ENU[2] = r_body_curr_ENU[2];
  v_body_prev_ENU[0] = v_body_curr_ENU[0];
  v_body_prev_ENU[1] = v_body_curr_ENU[1];
  v_body_prev_ENU[2] = v_body_curr_ENU[2];

  // if gps is renewed then use gps
  if (isGPSUpdated) {
    // assume current gps position as a navigation solution for EN coordinate
    r_body_curr_ENU[0] = r_body_gps_ENU[0];
    r_body_curr_ENU[1] = r_body_gps_ENU[1];
    // U coordinate estimate
    r_body_curr_ENU[2] = inc_h_baro; // always assume that barometer altitude is true


    // assume prev velocity solution is still usable at current time step (because of pos discontinuity)
    // i.e. virtually no velocity update
    v_body_curr_ENU[0] = v_body_prev_ENU[0];
    v_body_curr_ENU[1] = v_body_prev_ENU[1];
    v_body_curr_ENU[2] = v_body_prev_ENU[2];

    return;
  }

  // during no gps message integrate acc.
  float dt_sec, dt_sec_2;
  dt_sec = dt_AHRS_sec; // sec
  dt_sec_2 = dt_sec * dt_sec; // sec2

  // predict with Euler integration
  v_body_curr_ENU[0] = v_body_prev_ENU[0] + acc_body_ENU[0] * dt_sec;
  v_body_curr_ENU[1] = v_body_prev_ENU[1] + acc_body_ENU[1] * dt_sec; 
  v_body_curr_ENU[2] = v_body_prev_ENU[2] + acc_body_ENU[2] * dt_sec;

  r_body_curr_ENU[0] = r_body_prev_ENU[0] + v_body_prev_ENU[0] * dt_sec + 0.5 * acc_body_ENU[0] * dt_sec_2;
  r_body_curr_ENU[1] = r_body_prev_ENU[1] + v_body_prev_ENU[1] * dt_sec + 0.5 * acc_body_ENU[1] * dt_sec_2;
  r_body_curr_ENU[2] = r_body_prev_ENU[2] + v_body_prev_ENU[2] * dt_sec + 0.5 * acc_body_ENU[2] * dt_sec_2;

  // estimate drift
  float drift;
  drift = r_body_curr_ENU[2] - inc_h_baro; // m

  // correct navigation solution
  r_body_curr_ENU[0] = r_body_curr_ENU[0] - drift;
  r_body_curr_ENU[1] = r_body_curr_ENU[1] - drift;
  r_body_curr_ENU[2] = r_body_curr_ENU[2] - drift;

  return;
}

void Navigation::getPosENU_m(float pos_ENU[3]) {
  pos_ENU[0] = r_body_curr_ENU[0];
  pos_ENU[1] = r_body_curr_ENU[1];
  pos_ENU[2] = r_body_curr_ENU[2];
  return;
}

void Navigation::getVelENU_ms(float vel_ENU[3]) {
  vel_ENU[0] = v_body_curr_ENU[0];
  vel_ENU[1] = v_body_curr_ENU[1];
  vel_ENU[2] = v_body_curr_ENU[2];
  return;
}

float Navigation::getAltENU_m() {
  return r_body_curr_ENU[2];
}

ApogeeEstimate Navigation::getChudinovApogeeEst() {
  float theta_0; // rad

  float v1, v2, v3; // m/s

  v1 = v_body_curr_ENU[0];
  v2 = v_body_curr_ENU[1];
  v3 = v_body_curr_ENU[2];

  float dot, tmp;
  dot = v1 * v1 + v2 * v2;
  tmp = dot / sqrt(v1*v1 + v2*v2 + v3*v3) / sqrt(dot);
  theta_0 = acos(tmp); // rad

  float r3; // m
  r3 = r_body_curr_ENU[2];
  float rho = 1.225 * exp(-r3/10.4/1000); // kg/m3
  float k = rho * C_D0 * S_ref /2 / m_dry / g0;

  float vsqured;
  vsqured = v1*v1 + v2*v2 + v3*v3;

  float H_apogee, H_local;
  H_apogee = r3 + vsqured * (sin(theta_0))*(sin(theta_0)) / g0 / (2 + k * sin(theta_0) * vsqured); // m
  H_local = H_apogee - r3;
  
  float T;
  T = 2 * sqrt(2 * H_local / g0);

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