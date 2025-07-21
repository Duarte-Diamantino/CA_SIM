# Pickup Description: Map & Robot Setup

Este guia mostra como compilar o pacote e lançar, separadamente, o servidor de mapa e o modelo do robô (URDF) no ROS 2 Humble. Podes colar este conteúdo num `README.md`.

---

```bash
rm -rf build/MPC_python
colcon build --symlink-install --packages-select MPC_python
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run MPC_python mpc_controller
```

# 1. Preparar e compilar o workspace

```bash
# Vai para a raiz do teu workspace
cd ~/pickup_ws

# Apaga diretórios de build, install e log (build “limpo”)
rm -rf build/ install/ log/

# Compila **todos** os pacotes (para instalar share/… corretamente)
source /opt/ros/humble/setup.bash
colcon build --packages-select pickup_description
# Carrega a instalação no ambiente
source install/setup.bash
rviz2 -d ~/f1tenth/src/pickup_description/config/display.rviz
```


# 2. Lançar o mapa no RViz

```bash
# (Re)compila se tiveres alterações no launch de mapas
cd ~/pickup_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pickup_description
source install/setup.bash

# Lança o servidor de mapa
ros2 launch pickup_description map_only.launch.py
```

# 3. Publicar o modelo URDF (sem abrir novo RViz)

```bash
# Dá permissão de execução ao launch
chmod +x src/pickup_description/launch/robot.launch.py

# Compila e fonteia
cd ~/pickup_ws

colcon build --packages-select pickup_description

source /opt/ros/humble/setup.bash
source install/setup.bash

# Lança o publish do URDF + TF
ros2 launch pickup_description robot.launch.py
```

No **mesmo RViz** onde tens o mapa:
- Em **Displays** → **Add** → **RobotModel**  
  - **Description Source** → **File**  
  - **Description File** → `/home/duarte_diamantino/pickup_ws/src/pickup_description/urdf/pickup.urdf`  
- (Opcional) Em **Displays** → **Add** → **TF** para ver `map` → `base_link`

# 4. Dar reset ao pose do carro
```bash
source /opt/ros/humble/setup.bash
ros2 launch pickup_description set_pose.launch.py   x:=2.0 y:=-2.0 z:=0.05 roll:=-0.2 pitch:=0.0 yaw:=0.0
```

# 5. desenhar a spline
```bash

cd ~/f1tenth
colcon build --packages-select pickup_description
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pickup_description spline.launch.py   a:=-0.7 b:=0.0 c:=4.0 x_min:=-2.0 x_max:=1.0
```


# 6. Desenho da trajetoria quadrada
```bash
# Vai para o teu workspace
cd ~/f1tenth
source /opt/ros/humble/setup.bash

# Compila apenas o pacote pickup_description
colcon build --packages-select pickup_description

# Fonteia o ambiente (bash)
source install/setup.bash

# (Opcional) certifica-te de que o teu launch está executável:
chmod +x src/pickup_description/launch/launch_rounded_rectangle.launch.py

ros2 launch pickup_description launch_rounded_rectangle.launch.py   width:=7.0 height:=5.0 radius:=1.0 center_x:=3.0 center_y:=2.5 frame_id:=map
```
