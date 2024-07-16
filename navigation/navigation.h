#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "vector_util.h"
#include "navigation_config.h"
#include "SensorSet.h"

struct ApogeeEstimate {
  float t_apogee; // sec
  float H_apogee; // m
  float V_apogee; // m/s
};

class Navigation {
  public:
    void initializeLaunchSiteConfig(float lat_deg, float lon_deg, float alt_orthometric_m, float geoid_separation_m, float atm_pressure_Pa, float atm_temp_K);
    void update(SensorDataCollection& newSensorData);
    void getPosENU_m(float pos_ENU[3]);
    void getVelENU_ms(float vel_ENU[3]);
    float getAltENU_m(); // return altitude measured from the launch site (AGL)
    
    ApogeeEstimate getChudinovApogeeEst(); // estimate apogee height, time to apogee, and velocity at apogee using Chudinov eqn.

  private:
    // private methods
    void updateSensorValues(SensorDataCollection& newSensorData);
    void updateNavigation(); // update and estimate the best navigation solution
    
    void updateAHRSMeasurement(uint32_t t_ms, float* imu_data);
    void updateBMPMeasurement(float p_baro_hPa);
    void updateGPSMeasurement(float* gps_data);
    
    // helper functions
    void quat_to_DCM(float quat[4], float dcm[3][3]);
    void lla_to_ECEF(float lat_deg, float lon_deg, float alt_wgs84, float r_ECEF[3]); // convert lla to ecef
    void get_dcm_ECEF_to_ENU(float lat_deg, float lon_deg, float dcm_ECEF_to_ENU[3][3]); // calculate dcm for ecef to enu at launch site


    // private attributes

    /* Below parameters are calculated from the value specified in 'navigation_config.h' file. */
    // LPF parameters
    const float tau = 1 / (2 * PI * F_CUTOFF); // sec, time constant of the transfer function of the lpf
    const float alpha = T_S / (tau + T_S); // y(n) = alpha * x(n) + (1-a) * y(n-1)
    
    // TU-1.f configuration
    float r_imu_B[3] = {IMU_CG_DIST, 0, 0}; // m, cg to imu poistion vector (in body frame)
    /* (You can change above) */

    /* Don't change below */
    // Universal Constant
    const float g0 = 9.80665; // m/s^2, gravitational acc
    const float g_ENU[3] = {0, 0, -g0}; // m/s^2, gravitational acc vec in ENU
    const float a_e = 6378137; // m, equatorial radius of Earth
    const float fltn = 1 / 298.257223563f; // -, flattening of Earth
    const float b_e = a_e * (1 - fltn); // m
    const float e_2 = 1 - b_e*b_e / (a_e*a_e); // eccentricity squared
    const float R = 287.05; // J/kg-K, air gas constant

    // Launch Site Configuration
    // should be measured
    float lat_deg_launch_site; // deg, geodetic latitude of launch site
    float lon_deg_launch_site; // deg, longitude of launch site
    float alt_orthometric_launch_site; // m, orthometric altitude of the launch site
    float geoid_separation_launch_site; // m, geoid separation at the launch site
    float atm_pressure_launch_site; // Pa, atm pressure measured at launch origin
    float atm_temp_launch_site; // K, atm temperature measured at launch origin
    // should be calculated (derived)
    float alt_wgs84_launch_site; // m, wgs84 altitude at the launch site
    float r_launch_site_ECEF[3]; // m, launch site position in ECEF
    float dcm_ECEF_to_ENU[3][3]; // DCM from ECEF to ENU at launch site
    
    /* below variables are updated real-time during flight. */
    // AHRS data
    // should be transfered from the previous time step
    uint32_t t_AHRS_prev_msec = 0; // millisec, AHRS aquired time at previous time step
    float angular_rate_filtered_prev_B[3] = {0, 0, 0}; // rad/s, y(n-1) body angular rate in previous time step (in body frame)
    // should be measured
    uint32_t t_AHRS_msec = 0; // millisec, latest AHRS aquired time
    float* quat_ENU_to_B; // bno055 attitude quaternion from 'ENU' to 'initial body frame', [qx, qy, qz, qw]
    float* acc_imu_B; // m/s^2, linear acceleration felt by imu module (need to subtract centrifugal force) (in body frame)
    float* angular_rate_B; // rad/s, x(n) body angular rate (in body frame)
    // should be calculated (derived)
    uint32_t dt_AHRS_msec; // millisec, time step
    float dt_AHRS_sec; // sec, time step
    float dcm_ENU_to_B[3][3]; // dcm from ENU to body
    float dcm_B_to_ENU[3][3]; // dcm from body to ENU
    float angular_rate_filtered_B[3] = {0, 0, 0}; // rad/s, y(n) body angular rate LPF output (in body frame)
    float angular_acc_B[3]; // rad/s^2, body angular acc (in body frame)
    float acc_body_B[3]; // m/s^2, linear acceleration felt by the CG of the vehicle. (in body frame)
    float acc_body_ENU[3]; // m/s^2, linear accelartion of the CG of the vehicle in ENU, for navigation
    
    // Barometer
    // should be measured
    float p_baro = 101325; // Pa, atm pressure measured at altitude by BMP280
    // should be calculated (derived)
    float inc_h_baro = 0; // m, pressure altitude measured by BMP280.

    // GPS
    bool isGPSUpdated = false; // true if new GPS message has arrived, false otherwise.
    // should be measured
    // should be calculated (derived)
    float alt_wgs84_body = 0; // m, wgs84 altitude of the vehicle from GPS
    float r_body_gps_ECEF[3]; // m, position from gps, in ECEF
    float r_body_rel_gps_ECEF[3]; // m, position from gps, relative to the launch site, in ECEF
    float r_body_gps_ENU[3]; // m, position from gps, in ENU
    
    // Navigation Solution
    float r_body_prev_ENU[3] = {0, 0, 0}; // m, pos navigation solution in previous time step (in ENU frame)
    float v_body_prev_ENU[3] = {0, 0, 0}; // m/s, vel navigation solution in previous time step (in ENU frame)
    float r_body_curr_ENU[3] = {0, 0, 0}; // m, pos navigation solution in current time step (in ENU frame)
    float v_body_curr_ENU[3] = {0, 0, 0}; // m/s, vel navigation solution in current time step (in ENU frame)
};

#endif