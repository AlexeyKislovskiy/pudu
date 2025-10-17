# Robot Control

## Оглавление
- [Создание проекта](#создание-проекта)
- [Настройка конфигурации](#настройка-конфигурации)
- [Настройка файла запуска контроллера](#настройка-файла-запуска-контроллера)
- [Настройка плагина Gazebo](#настройка-плагина-gazebo)
- [Настройка CMakeLists.txt](#настройка-cmakeliststxt)
- [Запуск!](#запуск)

## Создание проекта

Создадим пакет для работы с контроллерами
```
cd ~/nav_ws/src
ros2 pkg create --build-type ament_cmake bmx_control
```

Удалим директории для хранения исходного кода и добавим папки **config** для хранения конфигураций и **launch** для хранения файлов запуска: 

```
bmx_gazebo/
├── config/
├── launch/
├── CMakeLists.txt
└── package.xml
```

## Настройка конфигурации

Прежде всего установим (если еще не установили) необходимые зависимости: контроллер bicycle_steering_controller, входящий в состав метапакета [ros2_controllers](https://github.com/ros-controls/ros2_controllers)

```bash
sudo apt install ros-jazzy-ros2-controllers
```

Перенесем файл **controllers.yaml** из проекта *bmx_gazebo*. Добавим в файл описание контроллера [bicycle_steering_controller](https://control.ros.org/rolling/doc/ros2_controllers/bicycle_steering_controller/doc/userdoc.html).

```yaml
/bmx/controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
  
    bicycle_steering_controller:
      type: bicycle_steering_controller/BicycleSteeringController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster



/bmx/joint_state_broadcaster:
  ros__parameters:
    use_local_topics: false
    extra_joints: ["front_wheel_joint"]
    frame_id: base_footprint

/bmx/bicycle_steering_controller:
  ros__parameters:
    reference_timeout: 2.0
    open_loop: false
    velocity_rolling_window_size: 10
    position_feedback: false
    traction_joints_names: [rear_wheel_joint]
    steering_joints_names: [front_steering_wheel_joint]
    base_frame_id: base_footprint
    wheelbase: 0.32
    traction_wheel_radius: 0.1
```

## Настройка файла запуска контроллера

Создадим отдельный файл запуска. Для этого в папке */launch* создадим файл **control.launch.py** с заготовкой кода:

```python

from launch import LaunchDescription

def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    return ld
```

Объявим и определеим аргументы файла запуска, необходимые для спавна робота: пространство имен (namespace) робота.

```python

...
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='bmx',
        description='Top-level namespace'
    )
    ...
```

Перенесем ноду запуска контроллера joint_state_broadcaster и добавим контроллер bicycle_steering_controller.

```python

...
from launch_ros.actions import Node

def generate_launch_description():
    ...
    
	joint_state_broadcaster_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["--controller-manager-timeout", "100", 
			 "-c", [namespace, '/controller_manager'], 
			 'joint_state_broadcaster',
			 ],
		output='screen',
	)

	bicycle_steering_controller_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["--controller-manager-timeout", "100", 
			 "-c", [namespace, '/controller_manager'], 
			 'bicycle_steering_controller',
			 ],
		output='screen',
	)
    ...
```

После создания необходимых объектов добавим их выполнение в объект LaunchDescription.

```python

...

def generate_launch_description():
    ...

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(bicycle_steering_controller_spawner)

    return ld
    ...
```
После настройки необходимо убрать запуск контроллера из [bmx_gazebo](robot_gazebo.md#настройка-контроллера-для-публикации-положений-суставов-в-ros), изменить путь до контроллеров в [URDF-файле плагина](robot_gazebo.md#настройка-моторов-и-контроллеров-робота).

## Настройка плагина Gazebo

По умолчанию контроллер будет публиковать одометрию и трансформацию в топики bicycle_steering_controller/odometry. Для того, чтобы выполнить ремаппинг в топики /bmx/tf и /bmx/odom, добавим соответствующие тэги в файл **gz_control.urdf.xacro** по пути *bmx_description/urdf/components/plugins*. Кроме того, не забудем поменять путь до файла контроллеров, с пакета *bmx_gazebo* на *bmx_control*.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

	<xacro:macro name="gz_control" params="namespace">

		<gazebo>
			<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
				<parameters>$(find bmx_control)/config/controllers.yaml</parameters>

				<ros>
					<namespace>${namespace}</namespace>
					<remapping>tf:=${namespace}/tf</remapping>
					<remapping>tf_static:=${namespace}/tf_static</remapping>
					<remapping>bicycle_steering_controller/odometry:=odom</remapping>
					<remapping>bicycle_steering_controller/tf_odometry:=tf</remapping>
				</ros>
			</plugin>
		</gazebo>
		
	</xacro:macro>

</robot>
```

## Настройка CMakeLists.txt

Настроим CMakeLists.txt, чтобы после сборки проекта были доступны launch-скрипты и config-файлы. Для этого перейдем в CMakeLists.txt и добавим в конец файла следующие строки:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bmx_control)

...

install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

## Запуск!

Файл запуска готов. Выполняем сборку проекта
```bash
colcon build
```

Предварительно запускаем робота в Gazebo
```bash
ros2 launch bmx_gazebo bmx_gazebo.launch.py
```

Активируем контроллеры
```bash
source ~/nav_ws/install/setup.bash
ros2 launch bmx_control control.launch.py
```

Убедимся, что робот начал считать одометрию. Для этого в очередной раз воспользуемся пакетом для визуализации трансформаций между фреймами робота.

```bash
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=/bmx/tf -r /tf_static:=/bmx/tf_static
```

<img src="content/bmx_control_frames.svg" alt="drawing" width="1000"/>

Поскольку мы установили контроллер на робота, теперь мы им можем управлять. Для этого предварительно установим пакет, который позволит слать на робота команды управления

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

Запустим узел, сделав ремап из топика */cmd_vel* (узел по умолчанию читает из этого топика) в топик */bmx/bicycle_steering_controller/reference*. Топик */bmx/bicycle_steering_controller/reference* мы берем не случайно, именно из этого топика читает скорости контроллер; в этом можно убедиться, получив информацию об узле

```bash
$ ros2 node info /bmx/bicycle_steering_controller
/bmx/bicycle_steering_controller
  Subscribers:
    /bmx/bicycle_steering_controller/reference: geometry_msgs/msg/TwistStamped
    /clock: rosgraph_msgs/msg/Clock
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /bmx/bicycle_steering_controller/controller_state: control_msgs/msg/SteeringControllerStatus
    /bmx/bicycle_steering_controller/transition_event: lifecycle_msgs/msg/TransitionEvent
    /bmx/odom: nav_msgs/msg/Odometry
    /bmx/tf: tf2_msgs/msg/TFMessage
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /bmx/bicycle_steering_controller/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /bmx/bicycle_steering_controller/get_logger_levels: rcl_interfaces/srv/GetLoggerLevels
    /bmx/bicycle_steering_controller/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /bmx/bicycle_steering_controller/get_parameters: rcl_interfaces/srv/GetParameters
    /bmx/bicycle_steering_controller/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /bmx/bicycle_steering_controller/list_parameters: rcl_interfaces/srv/ListParameters
    /bmx/bicycle_steering_controller/set_logger_levels: rcl_interfaces/srv/SetLoggerLevels
    /bmx/bicycle_steering_controller/set_parameters: rcl_interfaces/srv/SetParameters
    /bmx/bicycle_steering_controller/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

```

Выполним запуск узла телеопа. Кроме ремапа укажем также флаг stamped=true. Этот флаг говорит о том, чтобы мы публиковали скорости с указанием времени публикации. По умолчанию контроллер ждет сообщения с указанием времени публикации.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bmx/bicycle_steering_controller/reference -p stamped:=true
```

Найдем на английской раскладке клавиши *"i"* и *"<"*, робот должен выполнить движение вперед и назад соответственно. При нажатии на *"j"* и *"l"* робот будет поворачивать рулевое колесо налево и направо соответственно. При нажатии на кнопки *"u"*, *"o"*, *"m"*, *">"* робот будет ехать вперед с поворотом налево, ехать вперед с повотором направо, ехать назад с повотором налево и ехать назад с поворотом направо соответственно.