int32 left_paw_distance_control  #0x00 无动作 0x01 轴距增大  0x02 轴距减小
int32 right_paw_distance_control
int32 left_paw_speed   #分辨率1，单位mm（范围暂定0-1000）（轴距相对初始位置）
int32 right_paw_speed
int32 paw_lift_control  #0x00 无动作 0x01 夹爪上升  0x02 夹爪降低 （其他数值无动作上升下降档位）
int32 vehicle_put_control   #0x00无动作 0xAA自动取车 0xBB自动放车（其他数值无动作）



