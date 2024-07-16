#ifndef NAVIGATION_CONFIG_H
#define NAVIGATION_CONFIG_H

/*************** Gyro Low Pass Filter Config ****************/
#define T_S 0.02 // Nominal sampling rate of gyroscope, in sec
#define F_CUTOFF 0.5 // Hz, cutoff frequency of 1st order low pass filter

/*************** TU-1.f Vehicle Config ****************/
#define M_DRY 3.7 // kg, dry mass of the vehicle after burnout
#define C_D0 0.3 // -, zero lift drag coefficient of the vehicle
#define S_REF 0.00849486653 // m^2, reference area for aerodynamic coeff.
#define IMU_CG_DIST 0.25 // m, distance from cg to imu

#endif