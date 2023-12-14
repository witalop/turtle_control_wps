# turtle_control_wps
## Instalação do pacote

Abra o diretório src dentro do seu workspace ROS2 e faça o clone do repositório:
```
cd ~/ros2_ws/src && git clone https://github.com/witalop/turtle_control_wps.git
```

No workspace faça o build e o setup:
```
cd ~/ros2_ws && colcon build && source install/setup.bash
```

## Laboratório 5:

Para realizar os experimentos do Laboratório 5 siga os passos:

1 - Utilize um terminal para executar o turtlesim_node:
```
ros2 run turtlesim turtlesim_node
```

2 - Em um segundo terminal, execute o nó responsável por fazer o controle da tartaruga.
```
ros2 run turtle_control_wps turtle_control
```

3 - Abra um terceiro terminal e utilize para enviar os comandos de posição desejada no seguinte modelo:
```
ros2 topic pub /goal geometry_msgs/msg/Pose2D "{x: 7.0, y: 7.0, theta: 0.0}"
```

## Laboratório 6:

Para realizar os experimentos do Laboratório 5 siga os passos:

1 - Realize as configurações necessárias no ambiente e abra a simulação do mundo:
```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2 - Em um segundo terminal, execute o nó responsável por fazer o controle baixo nível da tartaruga.
```
source ~/ros2_ws/install/setup.bash
ros2 run turtle_control_wps actuator
```

3 - Em um terceiro terminal, execute o nó responsável por enviar as posições desejadas em alto nível.
```
source ~/ros2_ws/install/setup.bash
ros2 run turtle_control_wps conductor
```
