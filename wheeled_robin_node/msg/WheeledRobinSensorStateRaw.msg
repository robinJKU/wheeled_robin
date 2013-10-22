uint8 OI_MODE_OFF = 0
uint8 OI_MODE_PASSIVE = 1
uint8 OI_MODE_SAFE = 2
uint8 OI_MODE_FULL = 3

Header header

int16 linear_velocity #mm/s
int16 angular_velocity #mrad/s
int16 pitch #mrad
int16 pich_rate #mrad/s
int16 left_wheel #mrad
int16 right_wheel #mrad
int16 left_wheel_rate #mrad/s
int16 right_wheel_rate #mrad/s
uint8 mode #mode of the robot 0..off, 1..passive, 2..safe, 3..full 
int16 left_current # mA
int16 right_current # mA
int16 voltage #mV
int16 imu_acc_x #mm/s^2
int16 imu_acc_y #mm/s^2
uint8 digital_outputs
uint8 digital_inputs

