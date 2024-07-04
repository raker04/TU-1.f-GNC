#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <ArduinoEigen.h>

using Eigen::Matrix3f; // 3x3 matrix, double type element
using Eigen::Vector3f; // 3x1 vector, double type element
using Eigen::Vector4f; // 4x1 vector, double type element, for quaternion (qx, qy, qz, qw)

struct LaunchSite {
  float geodetic_lat; // deg, geodetic latitude
  float lon; // deg, longitude
  float alt; // m, geodetic altitude
  Matrix3f DCM_ECEF_2_ENU; // DCM from ECEF to ENU at launch site
  Vector3f r_ECEF; // m, launch site position in ECEF
};
struct EarthModel {
  Vector3f g_ENU; // m/s^2, gravitational acc vec in ENU
  const float g0 = 9.80665; // m/s^2, gravitational acc
  const float a_e = 6378137; // m, equatorial radius of Earth
  const float f = 1 / 298.257223563f; // -, flattening of Earth

};
struct LPFConfig {
  float alpha; // -, y(n) = alpha * x(n) + (1-a) * y(n-1)
};

struct VehicleProperty {
  // struct for TU-1.f config
  float m_dry; // kg, dry mass of the vehicle after burnout
  float C_D0; // -, zero lift drag coefficient of the vehicle
  float d_ref; // m, reference length (diameter of fuselage)
  float S_ref; // m^2, reference area for aerodynamic coeff.
  Vector3f r_IMU_B; // m, cg to imu poistion vector (in body frame)
};

struct NavigationSol {
  // the most recent navigation solution
  Vector3f r_ENU; // m, position of the vehicle in ENU frame.
  Vector3f v_ENU; // m/s, velocity of the vehicle in ENU frame.
  Vector3f r_ENU_prev; // m, pos navigation solution in previous time step (in ENU frame)
  Vector3f v_ENU_prev; // m/s, vel navigation solution in previous time step (in ENU frame)

};

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
};

struct AHRSData {
  unsigned long t_AHRS; // millisec, latest AHRS aquired time
  unsigned long t_AHRS_prev; // millisec, AHRS aquired time at previous time step
  unsigned int dt; // millisec, time step

  Vector4f quat_ENU_to_B; // bno055 attitude quaternion from 'ENU' to 'initial body frame', [qx, qy, qz, qw]
  Matrix3f DCM_ENU_to_B; // dcm from enu to B
  
  Vector3f acc_imu_B; // m/s^2, linear acceleration felt by imu module (need to subtract centrifugal force) (in body frame)
  Vector3f acc_cg_B; // m/s^2, linear acceleration felt by the CG of the vehicle. (in body frame)
  Vector3f acc_cg_ENU; // m/s^2, accelartion in ENU, for navigation

  Vector3f angular_rate_B; // rad/s, x(n) body angular rate (in body frame)
  Vector3f angular_rate_filtered_B; // rad/s, y(n) body angular rate LPF output (in body frame)
  Vector3f angular_rate_filtered_prev_B; // rad/s, y(n-1) body angular rate in previous time step (in body frame)

  Vector3f angular_acc_B; // rad/s^2, body angular acc (in body frame)
};

struct GPSData {
  unsigned long t_GPS; // millisec, latest GPS aquired time
  float gps_geodetic_lat; // deg, geodetic latitude from GPS
  float gps_lon; // deg, longitude from GPS
  float gps_geoidSeperation; // m, wgs geodetic altitude
  float gps_alt_msl; // m, mean sea level altitude
  // then, alt_wgs84 = alt_msl + geoidSeperation
  float gps_alt_wgs84; // m, altitude from ellipsoid
  Vector3f pos_gps_ECEF; // m, position from gps, in ECEF
  Vector3f pos_gps_ENU; // m, position from gps, in ENU

};

struct BMPData {
  unsigned long t_baro; // millisec, latest BMP aquired time
  float h_baro; // m, pressure altitude measured by BMP280.
};

class Navigation {
  public:
    // public method
    void initializeLPFConfig(float alpha);
    void initializeVehicleConfig(float m_dry, float C_D0, float d_ref, float r_IMU_b[]);
    void initializeEarthModel(float g, float a_e, float f);
    
    void updateSensorValue(SensorData newSensorData);
    void updateNavigation(); // update and estimate the best navigation solution
    ApogeeEstimate getChudinovApogeeEst(); // estimate apogee height, time to apogee, and velocity at apogee using Chudinov eqn.

  private:
    // attributes

    // Configuration, should be updated by calling initialization method
    LPFConfig lpf; // LPF parameters
    EarthModel earth; // Universal Constant for Earth
    VehicleProperty TU1f; // TU-1.f configuration
    LaunchSite launch_site; // Launch Site Configuration

    // below variables are updated real-time during flight.
    AHRSData ahrs;
    GPSData gps;
    BMPData bmp;
    NavigationSol nav_sol;

    // private methods
    void quat_to_DCM();
    void acc_imu_to_acc_cg();
    void acc_B_to_ENU();

    float lat_geocent_to_geodet(float geocent_lat_rad); // convert geocentric latitude to geodetic latitude
    void lla_to_ecef(); // convert gps lla to ecef
    void ecef_to_enu(); // calculate dcm for ecef to enu at launch site

  // def of body frame == bno055 body frame

    };

#endif