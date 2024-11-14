# neo_rox_moveit2
Moveit packages for all the rox series having manipulators

### Launch Command

The following command uses the rox robot urdf available in the [`rox_description`](https://github.com/neobotix/rox/tree/rolling/rox_description) package. The `ur_type` launch argument supports `ur10` and `ur5e` arms

`ros2 launch neo_rox_moveit2 neo_ur_moveit.launch.py ur_type:=ur10 use_sim_time:=True use_fake_hardware:=true prefix:=ur10`

- Make sure to use the above command after launching the simulation using the following command.
 
`ros2 launch rox_bringup bringup_sim_launch.py arm_type:=ur10`