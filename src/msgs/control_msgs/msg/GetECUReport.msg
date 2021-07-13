Header header

control_msgs/ModeReport mode

control_msgs/SteerReport steer_cur

control_msgs/GearReport shift_cur
control_msgs/GearReport shift1_cur

control_msgs/BrakeReport brake_cur
control_msgs/ThrottleReport throttle_cur # include engine_status

control_msgs/SpeedReport speed

control_msgs/HMIReport manual


float64 motor_I_L
float64 motor_I_R
float64 motor_Torque_L
float64 motor_Torque_R
float64 motor_U_L
float64 motor_U_R
float64 motor_n_now_L
float64 motor_n_now_R
float64 Gear_state_now_L
float64 Gear_state_now_R
float64 steer_degree_now
float64 steer_direction_now
float64 soc_now



uint16 remote_control_start
uint16 remote_control_move
uint16 remote_control_drive
float64 remote_control_brake
uint16 remote_control_switch
uint16 remote_control_steer
float64 remote_expect_rs
float64 remote_expect_steer


float32 APU_engine_temperature
float32 Vehicle_speed_exp_feedback

float32 APU_VCU_engine_n
float32 APU_VCU_engine_Torque
float32 APU_VCU_power

