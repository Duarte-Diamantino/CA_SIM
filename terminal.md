
## 🖥️ RustDesk

```bash
wget https://github.com/rustdesk/rustdesk/releases/latest/download/rustdesk-1.2.3-armv7.deb
sudo dpkg -i rustdesk-1.2.3-armv7.deb
```


# 💡 LED Control (ROS2)


## Criar o Pacote
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake led_controller --dependencies rclcpp std_msgs
```

## Criar Arquivo Fonte
```bash
cd ~/ros2_ws/src/led_controller/src
touch led_controller.cpp
code led_controller.cpp
code CMakeLists.txt
```

## Compilar o Pacote
```bash
cd ~/ros2_ws
colcon build --packages-select led_controller
source /opt/ros/humble/setup.bash
. install/setup.bash
```

## Executar o Nó

**Terminal 1:**
```bash
ros2 run led_controller led_controller
```

**Terminal 2 (Testes):**
```bash
ros2 topic pub /signal std_msgs/msg/String "data: 'animais_via'" --once
ros2 topic pub /signal std_msgs/msg/String "data: 'parque'" --once
```

```markdown
# 🦾 Gazebo
```
```bash
cd carro_tutorial_yotube
ros2 launch urdf_tutorial display.launch.py model:=/home/duarte_diamantino/carro_tutorial_yotube/my_robot_com_mesh_1.urdf
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ros_gz_sim gz_sim.launch.py
ros2 run ros_gz_sim create -file /home/duarte_diamantino/carro_tutorial_yotube/tentativa_gazebo_2.urdf -name meu_carro -x 0 -y 0 -z 0.4
ros2 topic pub /base_front_right_wheel_joint/command std_msgs/msg/Float64 '{data: 3.57}'
ros2 launch carro_tutorial_yotube_copy spawn_car.launch.py
```

```markdown
# 🛠️ VESC Tool
```
```bash
cd Downloads/VESC/
./vesc_tool_6.05
```

```markdown
# 🏎️ F1_ws (Carro RC)
```
```bash
cd f1tenth/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

```markdown
# 📡 Tópicos ROS2
```
```bash
ros2 topic list
ros2 topic echo /nome_do_topic
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 4600.0"
source /opt/ros/humble/setup.bash
```

```markdown
# 📈 Spline Visualizer
```
```bash
cd ~/f1tenth
source /opt/ros/humble/setup.bash
colcon build --packages-select spline_visualizer --packages-ignore vesc_ackermann 
source install/setup.bash
ros2 run spline_visualizer spline_visualizer \
  --ros-args \
  -p a:=0.0 -p b:=4.0 -p c:=1.0 \
  -p x_min:=0.0 -p x_max:=5.0
```

```markdown
# 🔧 MPC Controller
```
```bash
source /opt/ros/humble/setup.bash
cd ~/f1tenth
colcon build --packages-select mpc_controller
source install/setup.bash
ros2 run mpc_controller mpc_follower
```


```markdown
# 🔧 DWA Controller
```

```bash
source /opt/ros/humble/setup.bash
cd ~/f1tenth
colcon build --packages-select dwa_follower
source install/setup.bash
ros2 run dwa_follower dwa_follower
```



```markdown
# 🧭 RViz e Rosbag
```

## Rosbag Play
```bash
ros2 bag play odom_bag
ros2 bag play odom_bag --loop
```

## Rosbag Record
```bash
ros2 bag record /odom
ros2 bag record -o nova_bag_odom /odom
python3 analisar_odom.py
```

## Tópico IMU VESC
```bash
ros2 topic echo /sensors/imu --field imu.orientation
```
