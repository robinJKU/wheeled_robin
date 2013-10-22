uint8 OI_MODE_OFF = 0
uint8 OI_MODE_PASSIVE = 1
uint8 OI_MODE_SAFE = 2
uint8 OI_MODE_FULL = 3

Header header

float32 linear_velocity #mm/s
float32 angular_velocity #mrad/s
float32 pitch #mrad
float32 pich_rate #mrad/s
float32 left_wheel #mrad
float32 right_wheel #mrad
float32 left_wheel_rate #mrad/s
float32 right_wheel_rate #mrad/s
uint8 mode #mode of the robot 0..off, 1..passive, 2..safe, 3..full 
float32 left_current # mA
float32 right_current # mA
float32 voltage #mV
float32 imu_acc_x #mm/s^2
float32 imu_acc_y #mm/s^2
uint8 digital_outputs
uint8 digital_inputs

