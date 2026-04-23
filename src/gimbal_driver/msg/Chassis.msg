# TypeID=6 chassis feedback decoded from lower machine packed fields
# reserve_32_1: low16=steer_angle, high16=angular_velocity
# reserve_32_2: low16=velocity_x, high16=velocity_y
# Each 16-bit value uses: 8-bit integer + 8-bit decimal (2 digits)

std_msgs/Header header
float32 steer_angle
float32 angular_velocity
float32 velocity_x
float32 velocity_y
