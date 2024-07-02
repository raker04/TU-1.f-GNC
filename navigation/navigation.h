#ifndef NAVIGATION_H
#define NAVIGATION_H


struct ApogeeEstimate {
  float t_apogee; // sec
  float H_apogee; // m
  float V_apogee; // m/s
};
struct SensorData {
  // for bno055
  unsigned long t_AHRS; // ms, last AHRS aquired time
  float quat_ENU_to_b[4]; // [qx, qy, qz, qw]
  float linear_acc[3]; // m/s^2, [ax, ay, az]
  float angular_rate[3]; // rad/s, [p, q, r]

  // for GPS
  unsigned long t_GPS; // ms, last GPS aquired time
  float pos_gps_lla[3]; // (lon_deg, lat_deg, alt_m)

  // for bmp280
  unsigned long t_baro; // ms, last barometer aquried time
  float h_baro; // m, corrected altitude from bmp280 (h_measured - h_offset)

  // 시분할 삼센서 센서 융합을 통한 항법 최적해를 통한 모델로켓 탑재 검증을 통한 알고리즘 검증

};

struct DCM {
  float a11;
  float a12;
  float a13;
  float a21;
  float a22;
  float a23;
  float a31;
  float a32;
  float a33;
};

class Navigation {
  public:
    void updateSensorValue(SensorData newSensorData);
    void updateNavigation(); // update and estimate the best navigation solution
    ApogeeEstimate getChudinovApogeeEst(); // estimate apogee height, time to apogee, and velocity at apogee using Chudinov eqn.

  private:

    DCM quat_to_DCM(float* quat);
    void convert_imu_acc_to_body_acc();
    void crossProduct(float v_A[], float v_B[], float c_P[]); // c_P = v_A x v_B
    void matrixVecMult(DCM T, float b[], float res[]); // res = T * b (3x3 matrix and 3x1 vector multiplication)
    void matrixTranspose(DCM A, DCM B); // B = A^T;
    void acc_body_to_ENU();
    float lat_geocent_to_geodet(float geocent_lat_rad); // convert geocentric latitude to geodetic latitude
    void lla_to_ecef(); // convert gps lla to ecef
    void ecef_to_enu(); // calculate dcm for ecef to enu at launch site

  // def of body frame == bno055 body frame

    // LPF parameters
    const float alpha = 0.5; // y(n) = alpha * x(n) + (1-a) * y(n-1)


    // Universal Constant
    const float g_ENU[3] = {0, 0, -9.80665}; // m/s^2, gravitational acc vec in ENU
    const float g = 9.80665; // m/s^2, gravitational acc
    const float a_e = 6378137; // m, equatorial radius of Earth
    const float fltn = 1 / 298.257223563f; // -, flattening of Earth

    // TU-1.f configuration
    const float m_dry = 3.7; // kg, dry mass of the vehicle after burnout
    const float C_D0 = 0.3; // -, zero lift drag coefficient of the vehicle
    const float d_ref = 104 * 0.001f; // m, reference length (diameter of fuselage)
    const float S_ref = PI/4 * d_ref * d_ref; // m^2, reference area for aerodynamic coeff.
    float r_IMU_b[3] = {0.25, 0, 0}; // m, cg to imu poistion vector (in body frame)

    // Launch Site Configuration
    float launch_site_lat_lon[2]; // (deg, deg), geodetic latitude and longitude of launch site (initialized after boot up)
    DCM dcm_ecef_to_enu; // DCM from ECEF to ENU at launch site
    float launch_site_ECEF[3]; // m, launch site position in ECEF

    // below variables are updated real-time during flight.
    unsigned long t_GPS; // millisec, latest GPS aquired time
    unsigned long t_AHRS; // millisec, latest AHRS aquired time
    unsigned long t_AHRS_prev; // millisec, AHRS aquired time at previous time step
    unsigned long t_baro; // millisec, latest BMP aquired time
    unsigned int dt; // millisec, time step

    float quat_ENU_to_b[4]; // bno055 attitude quaternion from 'ENU' to 'initial body frame', [qx, qy, qz, qw]

    float linear_acc[3]; // m/s^2, linear acceleration felt by imu module (need to subtract centrifugal force) (in body frame)
    float body_acc[3]; // m/s^2, linear acceleration felt by the CG of the vehicle. (in body frame)
    float body_acc_ENU[3]; // m/s^2, accelartion in ENU, for navigation

    float angular_rate[3]; // rad/s, x(n) body angular rate (in body frame)
    float angular_rate_filtered[3]; // rad/s, y(n) body angular rate LPF output (in body frame)
    float angular_rate_filtered_prev[3]; // rad/s, y(n-1) body angular rate in previous time step (in body frame)
    
    float angular_acc[3]; // rad/s^2, body angular acc (in body frame)

    float pos_gps_lla[3]; // position from gps: lon, lat, alt (deg, deg, m)
    float pos_gps_ECEF[3]; // m, position from gps, in ECEF
    float pos_gps_ENU[3]; // m, position from gps, in ENU

    float r_ENU_prev[3]; // m, pos navigation solution in previous time step (in ENU frame)
    float v_ENU_prev[3]; // m/s, vel navigation solution in previous time step (in ENU frame)

    float r_ENU_curr[3]; // m, pos navigation solution in current time step (in ENU frame)
    float v_ENU_curr[3]; // m/s, vel navigation solution in current time step (in ENU frame)

    float h_baro; // m, pressure altitude measured by BMP280.
    
    float chamber_press; // barg, motor chamber pressure measured by pressure transducer.
};

#endif