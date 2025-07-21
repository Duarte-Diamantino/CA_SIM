#  1. Launch do comando e da camera

```bash
cd test_ws
source install/setup.bash
source /home/jetson/f1tenth/install/setup.bash
ros2 launch point_cloud_processor sensors_init.launch.py launch_ekf:=true launch_camera:=true use_sim_time:=false
```

# 2. Launch do mapa e da localização global

```bash
cd test_ws
source install/setup.bash
source /home/jetson/f1tenth/install/setup.bash
ros2 launch point_cloud_processor global_localization.launch.py use_sim_time:=false
```
