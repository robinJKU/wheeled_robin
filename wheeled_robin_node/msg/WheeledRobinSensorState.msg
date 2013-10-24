uint8 OI_MODE_OFF = 0
uint8 OI_MODE_PASSIVE = 1
uint8 OI_MODE_SAFE = 2
uint8 OI_MODE_FULL = 3

Header header

float32 linear_velocity #m/s
float32 angular_velocity #rad/s
float32 pitch #rad
float32 pitch_rate #rad/s
float32 left_wheel #rad
float32 right_wheel #rad
float32 left_wheel_rate #rad/s
float32 right_wheel_rate #rad/s
uint8 mode #mode of the robot 0..off, 1..passive, 2..safe, 3..full 
float32 left_current # A
float32 right_current # A
float32 voltage #mV
float32 imu_acc_x #m/s^2
float32 imu_acc_y #m/s^2
uint8 digital_outputs
uint8 digital_inputs

