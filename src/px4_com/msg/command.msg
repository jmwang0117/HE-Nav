
std_msgs/Header header

#enum Command
#{
#    Move_ENU,
#    Move_Body,
#    Hold,
#    Land,
#    Disarm,
#    Failsafe_land,
#    Custom
#};

# sub_mode 2-bit value:
# 0 for position, 1 for vel, 1st for xy, 2nd for z.
#                   xy position     xy velocity
# z position       	0b00(0)       0b10(2)
# z velocity		0b01(1)       0b11(3)
#

uint32 comid
uint8 command
uint8 sub_mode
float32[3] pos_sp      ## [m]
float32[3] vel_sp      ## [m/s]
float32 yaw_sp          ## [du]
float32 yaw_rate_sp


