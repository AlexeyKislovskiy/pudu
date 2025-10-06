# Robot Control

## Оглавление
- [Создание проекта](#создание-проекта)
- [Настройка конфигурации](#настройка-конфигурации)
- [Настройка файла запуска контроллера](#настройка-файла-запуска-контроллера)
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
from launch.actions import IncludeLaunchDescription

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

Файл запуска готов. Выполняем сборку проекта: ```colcon build```
Запускаем launch-файл, указав пространство имен для терминала.
```bash
source ~/nav_ws/install/setup.bash
ros2 launch bmx_control control.launch.py
```