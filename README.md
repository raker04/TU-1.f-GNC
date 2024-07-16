## Initialization
You need to set correct parameters in `navigation_config.h` file before you use this module. The LPF parameters will be automatically calculated from this information. Also, the distance between CG and IMU should be properly set to ensure an accurate conversion from imu acceleartion to body acceleration.

In the main flight computer code, first, declare a new Navigation object. Then, you should call `initializeLaunchSiteConfig` method to set the launch site information. These launch site information should be measured under stationary condition while the rocket is inserted at the launch pad.

## Usage
Call `update` method with `newSensorData` argument in your main loop. `newSensorData` is a reference to `SensorDataCollection` variable maintained in the main flight computer code. The time step is automatically calculated by using these timestamps, hence, the measurement time could be not-uniform, however, consistent and small time step is recommended for better performance.

## Methods
After you called `update`, the newest navigation solution is calculated and you have access to those by calling `getPosENU_m`, `getVelENU_ms`, and `getAltENU_m` methods. They provide position (in m), velocity (in m/s), and altitude (in m), all in ENU (East-North-Up) frame attached to the launch origin.

Also, the method `getChudinovApogeeEst` provides newest estimation for time-to-apogee, altitude-at-apogee, and velocity-at-apogee, in sec, m, and m/s respectively.
