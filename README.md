# 🚗 Como configurar o projeto ROS 2 `carro_tutorial_yotube_copy` no WSL2

## 🧰 1. Preparar o ambiente WSL2

### 1.1 Atualizar e instalar os pacotes essenciais do ROS 2 Humble

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-core \
  ros-humble-launch-ros \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-keyboard \
  build-essential \
  python3-colcon-common-extensions \
  git
```

### 1.2 Ativar fallback de GPU para o WSLg

```bash
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
source ~/.bashrc
```

---

## 🏗️ 2. Criar o workspace do ROS 2

```bash
mkdir -p ~/carro_ws/src
cd ~/carro_ws
rm -rf build install log  # (opcional)
```

---

## 📦 3. Criar o pacote Python

```bash
cd src
ros2 pkg create carro_tutorial_yotube_copy \
  --build-type ament_python \
  --dependencies launch_ros ros_gz_sim controller_manager robot_state_publisher
cd ..
```

---

## 🦴 4. Adicionar o ficheiro URDF

```bash
mkdir -p src/carro_tutorial_yotube_copy/urdf
cp /home/duarte_diamantino/carro_tutorial_yotube/tentativa_gazebo_2.urdf \
   src/carro_tutorial_yotube_copy/urdf/
```

---

## ⚙️ 5. Criar o ficheiro de configuração `ros2_control`

```bash
cat <<EOF > src/carro_tutorial_yotube_copy/ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
      left_wheel_names:  [ base_back_left_wheel_joint, base_front_left_wheel_joint ]
      right_wheel_names: [ base_back_right_wheel_joint, base_front_right_wheel_joint ]
      wheel_separation: 0.51
      wheel_radius:     0.10
      cmd_vel_timeout:  0.25
EOF
```

---

## 🌍 6. Criar mundo customizado em SDF

```bash
mkdir -p src/carro_tutorial_yotube_copy/worlds
cat <<EOF > src/carro_tutorial_yotube_copy/worlds/empty_with_commands.sdf
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty">
    <!-- directional light -->
    <light name="sun" type="directional">
      <direction>-0.5 -0.5 -1</direction>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
    </light>
    <!-- ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    <!-- plugins -->
    <plugin filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
  </world>
</sdf>
EOF
```

---

## 🚀 7. Criar ficheiro de lançamento

```bash
mkdir -p src/carro_tutorial_yotube_copy/launch
cat <<'EOF' > src/carro_tutorial_yotube_copy/launch/spawn_car.launch.py
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('carro_tutorial_yotube_copy')

    gz = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'),
                     'launch', 'gz_sim.launch.py')
      ),
      launch_arguments={'gz_args': os.path.join(pkg, 'worlds', 'empty_with_commands.sdf')}.items()
    )

    urdf = os.path.join(pkg, 'urdf', 'tentativa_gazebo_2.urdf')
    with open(urdf, 'r') as f:
        desc = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': desc}]
    )

    ctrl = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg, 'ros2_controllers.yaml')],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_car',
        output='screen',
        arguments=[
          '--world', 'empty',
          '--file', urdf,
          '--name', 'meu_carro',
          '--x', '0.0', '--y', '0.0', '--z', '0.4'
        ]
    )

    delay = TimerAction(period=5.0, actions=[spawn])

    return LaunchDescription([gz, rsp, ctrl, delay])
EOF

chmod +x src/carro_tutorial_yotube_copy/launch/spawn_car.launch.py
```

---

## 🛠️ 8. Atualizar `setup.py` e `package.xml`

### No `setup.py`, adiciona:

```python
('share/carro_tutorial_yotube_copy/launch', glob('launch/*.py')),
('share/carro_tutorial_yotube_copy/worlds', glob('worlds/*.sdf')),
('share/carro_tutorial_yotube_copy/urdf', glob('urdf/*.urdf')),
('share/carro_tutorial_yotube_copy', ['ros2_controllers.yaml']),
```

### No final do `package.xml`, adiciona:

```xml
<export>
  <build_type>ament_python</build_type>
</export>
```

---

## 🧪 9. Compilar e carregar ambiente

```bash
cd ~/carro_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 🧨 10. Lançar tudo com um comando

```bash
ros2 launch carro_tutorial_yotube_copy spawn_car.launch.py
```

---

## 🎛️ 11. Noutro terminal: ativar os controladores

```bash
source /opt/ros/humble/setup.bash
source ~/carro_ws/install/setup.bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller
```

---

## 🕹️ 12. Controlar o carro

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x:0.0,y:0.0,z:0.0}}"
```
