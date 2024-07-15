## Initialization
First, declare a new Navigation object. Then, you should call 'initializeLaunchSiteConfig' method to set the launch site information. These launch site information should be measured under stationary condition while the rocket is inserted at the launch pad.

## Usage
Call 'update' method with 'newSensorData' argument in your main loop. 'newSensorData' should contain newest bno055, bmp280, and gps measurement with timestamps. The time step is automatically calculated by using these timestamps, hence, the measurement time could be not-uniform, however, consistent and small time step is recommended for better performance.

## Methods
After you called 'update', the newest navigation solution is calculated and you have access to those by calling 'getPosENU_m', 'getVelENU_ms', and 'getAltENU_m' methods. They provide position (in m), velocity (in m/s), and altitude (in m), all in ENU (East-North-Up) frame attached to the launch origin.

Also, the method 'getChudinovApogeeEst' provides newest estimation for time-to-apogee, altitude-at-apogee, and velocity-at-apogee, in sec, m, and m/s respectively.
