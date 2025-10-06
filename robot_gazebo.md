# Robot in Gazebo

## Оглавление
- [Создание проекта](#создание-проекта)
- [Настройка виртуального мира](#настройка-виртуального-мира)
- [Настройка моторов и контроллеров робота](#настройка-моторов-и-контроллеров-робота)
- [Настройка проекта](#настройка-проекта)
  - [Настройка файла запуска Gazebo](#настройка-файла-запуска-gazebo)
  - [Настройка файла спавна робота в Gazebo](#настройка-файла-спавна-робота-в-gazebo)
  - [Настройка файла одновременного запуска Gazebo и спавна робота](#настройка-файла-одновременного-запуска-gazebo-и-спавна-робота)
  - [Настройка контроллера для публикации положений суставов в ROS](#настройка-контроллера-для-публикации-положений-суставов-в-ros)
  - [Настройка CMakeLists.txt](#настройка-cmakeliststxt)
- [Запуск!](#запуск)

## Создание проекта

Создадим пакет для работы робота в Gazebo
```
cd ~/nav_ws/src
ros2 pkg create --build-type ament_cmake bmx_gazebo
```

Удалим директории для хранения исходного кода и добавим папки **worlds** для хранения файлов виртуальных миров Gazebo, **config** для хранения конфигураций и **launch** для хранения файлов запуска: 

```
bmx_gazebo/
├── config/
├── launch/
├── worlds/
├── CMakeLists.txt
└── package.xml
```

## Настройка виртуального мира

Создадим пустой мир Gazebo с подстилающей плоскостью. Для этого будем придерживаться стандарта [SDF](http://sdformat.org/). Создадим пустой мир с помощью тега \<world\>.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

  </world>
</sdf>

```

Опишем физику виртуального мира в теге \<physics\>:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

  </world>
</sdf>

```

Настроим сцену, для этого укажем тег \<scene\> и отключим тени в Gazebo.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    ...
    <scene>
      <shadows>false</shadows>
    </scene>

  </world>
</sdf>

```

Настроим свет в Gazebo, для этого укажем тег \<light\> и зададим позицию источника света, параметры света, яркость, направление.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    ...
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

  </world>
</sdf>

```

Наконец зазадим модель подстилающей поверхности, представляющей из себя плоскость с нормалью к оси Z и размером 100х100. Флаг \<static\> поднимем, чтобы плоскость быза зафиксирована в своей позиции. Поскольку позиции не задана явно, она равняется (0, 0, 0).

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    ...
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>

```

## Настройка моторов и контроллеров робота

> Эта часть выполняется только после запуска робота в Gazebo без моторов

> Предварительно необходимо ознакомиться с работой [ROS2 Control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)

Чтобы робот начал функционировать в Gazebo, необходимо подключить моторы и контроллеры.

Колеса приводятся в действие моторами. Таким образом, чтобы привести в движение колесо, необходимо послать команду на мотор. В Gazebo моторы, как и другие элементы, задаются в URDF-файле. Добавим файл **control.urdf.xacro** в папку *bmx_description/urdf/components/plugins*. Опишем моторы внутри специального тега \<ros2_control\>. Напишем отдельный макрос для подключения моторов. Укажем интерфейс подключения к моторам - gz_ros2_control/GazeboSimSystem. Моторы укажем в отдельном макросе и будем передавать аргументом.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">


    <xacro:macro name="control" params="**transmissions">
        <ros2_control name="GazeboSimSystem" type="system">

            <hardware>
                <plugin>gz_ros2_control/GazeboSimSystem</plugin>
            </hardware>

            <xacro:insert_block name="transmissions"/>

        </ros2_control>
    </xacro:macro>
</robot>
```

В той же папке создадим файл **transmission.urdf.xacro**. Макрос будет создавать мотор для сустава *joint*. Внутри тега укажем параметры интерфейса мотора - какие команды он может принимать и какое состояние может публиковать. В нашем случае мотор может получать команды изменения скорости и/или позиции. Значения команд ограничены параметрами *min* и *max*. Обратная связь может включать позицию и/или скорость.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">


    <xacro:macro name="transmission" params="joint">

        <joint name="${joint}">
            <command_interface name="velocity">
                <param name="min">-1</param>
                <param name="max">1</param>
            </command_interface>
            <command_interface name="position">
                <param name="min">-10</param>
                <param name="max">10</param>
            </command_interface>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
            <state_interface name="velocity"/>
        </joint>
    </xacro:macro>
</robot>

```

Подключим моторы к нашей модели робота. Для этого в файле **bmx_description/urdf/robots/bmx.urdf.xacro** подключим созданный макрос и передадим необходимые аргументы. 

```xml
<?xml version="1.0"?>
<robot name="bmx" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:arg name="robot_name" default="bmx"/>
    <xacro:arg name="robot_namespace" default="$(arg robot_name)"/>

    ...
    <xacro:include filename="$(find bmx_description)/urdf/components/plugins/control.urdf.xacro" />
    <xacro:include filename="$(find bmx_description)/urdf/components/plugins/transmission.urdf.xacro" />

    <xacro:control>
        <transmissions>
            <xacro:transmission joint="front_wheel_joint"/>
            <xacro:transmission joint="rear_wheel_joint"/>
        </transmissions>
    </xacro:control>

    ...
</robot>
```

ROS не знает о существовании моторов и их показаниях. Чтобы осуществить передачу информации о текущем состоянии моторов и создать интерфейс для отправки команд на эти моторы необходимо создать менеджер ресурсов (управляет аппаратными компонентами) и менеджер контроллеров (управляет контроллерами). Для этого в самом URDF необходимо указать плагин **gz_ros2_control-system**, который создаст менеджер ресурсов и менеджер контроллеров. У каждого робота свои менеджеры, поэтому мы также задаем пространство имен (namespace) для конкретного робота и перенаправляем топики в соответствии с пространством имен.

Плагин ожидает информацию о контроллерах, которые будут использоваться. По умолчанию Gazebo не публикует показание суставов, это настраивается подключением отдельного контроллера. Создадим YAML-файл, в котором будет описан контроллер для публикации положений суставов.

```yaml
/bmx/controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

/bmx/joint_state_broadcaster:
  ros__parameters:
    use_local_topics: false
    extra_joints: ["front_wheel_joint"]
    frame_id: base_footprint
```

Сохраним файл в **controllers.yaml** по пути *bmx_gazebo/config*.


Теперь подключим плагин, который создаст необходимые менеджеры.Создадим файл **gz_control.urdf.xacro** по пути *bmx_description/urdf/components/plugins*. Опишем плагин внутри специального тега \<gazebo\>. Будем передавать макросу пространство имен нашего робота.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

	<xacro:macro name="gz_control" params="namespace">

		<gazebo>
			<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
				<parameters>$(find bmx_gazebo)/config/controllers.yaml</parameters>

				<ros>
					<namespace>${namespace}</namespace>
					<remapping>tf:=${namespace}/tf</remapping>
					<remapping>tf_static:=${namespace}/tf_static</remapping>
				</ros>
			</plugin>
		</gazebo>
		
	</xacro:macro>

</robot>
```

Добавим макрос в xacro-файле робота.

```xml
<?xml version="1.0"?>
<robot name="bmx" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:arg name="robot_name" default="bmx"/>
    <xacro:arg name="robot_namespace" default="$(arg robot_name)"/>

    ...
    <xacro:include filename="$(find bmx_description)/urdf/components/plugins/gz_control.urdf.xacro" />

    <xacro:gz_control namespace="$(arg robot_namespace)"/>

	...
</robot>
```


## Настройка проекта

### Настройка файла запуска Gazebo

Для запуска проекта необходимо настроить launch-файл. Для этого в папке */launch* создадим файл **gazebo.launch.py**. Зададим функцию generate_launch_description(), возвращающую объекта класса LaunchDescription следующим образом:

```python

from launch import LaunchDescription

def generate_launch_description():

	# Create the launch description and populate
	ld = LaunchDescription()

	return ld
```

Создадим переменную world_path, в которой будем хранить путь до world-файла Gazebo. В нашем случае это путь до файла *worlds/empty_world.world*.

```python

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_description():
    world_path = LaunchConfiguration('world_path')

	declare_world_path_cmd = DeclareLaunchArgument(
		name='world_path',
		default_value=os.path.join(get_package_share_directory('bmx_gazebo'), 'worlds', 'empty_world.world'),
		description='Specify world file'
	)
	...
```

Следующими укажем файлы запуска самого Gazebo. Для этого мы можем использовать ноды (nodes), но удобнее воспользоваться готовыми launch-файлами для запуска клиентской и серверной частей Gazebo. Launch-файлы будем запускать с помощью объекта IncludeLaunchDescription, указав путь до файла запуска.

```python

...
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ...
    
    gzserver_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
		launch_arguments={'gz_args': ['-r -s -v4 ', world_path],
						  'on_exit_shutdown': 'true'}.items()
	)

	gzclient_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
		launch_arguments={'gz_args': '-g -v4 '}.items()
	)
	...
```


После создания необходимых объектов добавим их выполнение в объект LaunchDescription.

```python

...

def generate_launch_description():
	...

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_world_path_cmd)

	# Add the actions to launch all of the files and nodes
	ld.add_action(gzserver_cmd)
	ld.add_action(gzclient_cmd)

	return ld
    ...
```

### Настройка файла спавна робота в Gazebo

Для спавна робота в Gazebo создадим отдельный файл запуска. Для этого в папке */launch* создадим файл **spawn.launch.py** с заготовкой кода:

```python

from launch import LaunchDescription

def generate_launch_description():

	# Create the launch description and populate
	ld = LaunchDescription()

	return ld
```

Объявим и определеим аргументы файла запуска, необходимые для спавна робота: пространство имен (namespace) робота и его позицию (x, y, z_rot).

```python

...
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_description():

	# Create the launch configuration variables
	namespace = LaunchConfiguration('namespace')
  	robot_name = LaunchConfiguration('robot_name')
	x = LaunchConfiguration('x')
	y = LaunchConfiguration('y')
	z_rot = LaunchConfiguration('z_rot')

	# Declare the launch arguments
	declare_namespace_cmd = DeclareLaunchArgument(
		name='namespace',
		default_value='bmx',
		description='Top-level namespace'
	)

	declare_x_cmd = DeclareLaunchArgument(
		name='x',
		default_value='0.0',
		description='Robot position (x)'
	)

	declare_y_cmd = DeclareLaunchArgument(
		name='y',
		default_value='0.0',
		description='Robot position (y)'
	)

	declare_z_rot_cmd = DeclareLaunchArgument(
		name='z_rot',
		default_value='0.0',
		description='Robot orientation (yaw)'
	)

	declare_robot_name_cmd = DeclareLaunchArgument(
		name='robot_name',
		default_value=namespace,
		description='Robot name in Gazebo'
	)
	...
```

Укажем файл запуска описания робота description.launch.py и передадим туда аргумент namespace.

```python

...
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ...
    
	robot_description_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory('bmx_description'), 'launch', 'description.launch.py'),
		launch_arguments={'namespace': namespace,
						  'use_sim_time': 'true'}.items()
	)
	...
```

Укажем узел для запуска робота в Gazebo. В этом узле укажем топик, в который мы публикуем описание робота в URDF - в нашем случае это топик **/{namespace}/robot_description**. 

```python

...

def generate_launch_description():
    ...
    
	spawn_entity_cmd = Node(
		package='ros_gz_sim',
		executable='create',
		name='urdf_spawner',
		output='screen',
		arguments=['-topic', [namespace, '/robot_description'],
				   '-name', robot_name,
				   '-x', x,
				   '-y', y,
				   '-Y', z_rot]
	)

```


После создания необходимых объектов добавим их выполнение в объект LaunchDescription.

```python

...

def generate_launch_description():
	...

	ld = LaunchDescription()

	ld.add_action(declare_namespace_cmd)
	ld.add_action(declare_robot_name_cmd)
	ld.add_action(declare_x_cmd)
	ld.add_action(declare_y_cmd)
	ld.add_action(declare_z_rot_cmd)

	ld.add_action(robot_description_cmd)
	ld.add_action(spawn_entity_cmd)

	return ld
    ...
```

### Настройка файла одновременного запуска Gazebo и спавна робота 

Объединим файлы запуска, указав все необходимые аргументы и задав оба файла запуска: **gazebo.launch.py** и **spawn.launch.py**. Создадим файл **bmx_gazebo.launch.py**.

```python

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
	world_path = LaunchConfiguration("world_path")
	verbose = LaunchConfiguration("verbose")

	namespace = LaunchConfiguration("namespace")
	robot_name = LaunchConfiguration("robot_name")
	x = LaunchConfiguration("x")
	y = LaunchConfiguration("y")
	z_rot = LaunchConfiguration("z_rot")

	declare_world_path_cmd = DeclareLaunchArgument(
		name="world_path",
		default_value=os.path.join(get_package_share_directory("bmx_gazebo"), "worlds", "empty_world.world"),
		description="Gazebo world"
	)

	declare_verbose_cmd = DeclareLaunchArgument(
		name="verbose",
		default_value="-v4",
		description="Gazebo verbose"
	)


	declare_namespace_cmd = DeclareLaunchArgument(
		name="namespace",
		default_value="bmx",
		description="Robot namespace"
	)

	declare_robot_name_cmd = DeclareLaunchArgument(
		name="robot_name",
		default_value=namespace,
		description="Robot name"
	)

	declare_x_cmd = DeclareLaunchArgument(
		name="x",
		default_value="0.0",
		description="Robot X axis"
	)

	declare_y_cmd = DeclareLaunchArgument(
		name="y",
		default_value="0.0",
		description="Robot Y axis"
	)

	declare_z_rot_cmd = DeclareLaunchArgument(
		name="z_rot",
		default_value="0.0",
		description="Robot Z ROT axis"
	)

	gazebo_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory("bmx_gazebo"), "launch", "gazebo.launch.py"),
		launch_arguments={
			'world': world_path,
			'verbose': verbose
		}.items()
	)

	spawn_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory("bmx_gazebo"), "launch", "spawn.launch.py"),
		launch_arguments={
			'namespace': namespace,
			'robot_name': robot_name,
			'x': x,
			'y': y,
			'z_rot': z_rot
		}.items()
	)

	ld = LaunchDescription()

	ld.add_action(declare_world_path_cmd)
	ld.add_action(declare_verbose_cmd)
	ld.add_action(declare_namespace_cmd)
	ld.add_action(declare_robot_name_cmd)
	ld.add_action(declare_x_cmd)
	ld.add_action(declare_y_cmd)
	ld.add_action(declare_z_rot_cmd)

	ld.add_action(gazebo_cmd)
	ld.add_action(spawn_cmd)


	return ld
```

### Настройка контроллера для публикации положений суставов в ROS

> Выполняется после настройки менеджера ресурсов и менеджера контроллеров

Контроллер необходимо подключать отдельно. Для этого создаем ноду spawner из пакета controller_manager и указываем название ноды менеджера контроллеров и название нашего контроллера.

```python

...
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

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

	...
	ld = LaunchDescription()

	...
	ld.add_action(joint_state_broadcaster_spawner)
	return ld
```

### Настройка CMakeLists.txt

Запустим пустой мир в Gazebo Harmonic. Прежде всего настроим CMakeLists.txt, чтобы после сборки проекта были доступны launch-скрипты и файлы мира. Для этого перейдем в CMakeLists.txt и добавим в конец файла следующие строки:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bmx_gazebo)

...

install(
  DIRECTORY launch worlds config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

## Запуск!

Файл запуска готов. Выполняем сборку проекта: ```colcon build```
Запускаем launch-файл, указав пространство имен для терминала.
```bash
source ~/nav_ws/install/setup.bash
ros2 launch bmx_gazebo gazebo.launch.py
```