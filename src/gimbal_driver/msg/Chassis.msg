# TypeID=6 chassis feedback decoded from lower machine packed fields
# reserve_32_1: low16=steer_angle, high16=angular_velocity
# reserve_32_2: low16=velocity_x, high16=velocity_y
# Each value is int16 raw from lower machine:
# steer_angle = low16(reserve_32_1) / 10
# angular_velocity = high16(reserve_32_1) / 100
# velocity_x = low16(reserve_32_2) / 100
# velocity_y = high16(reserve_32_2) / 100

std_msgs/Header header
float32 steer_angle
float32 angular_velocity
float32 velocity_x
float32 velocity_y
