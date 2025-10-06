
# Robot description

## Оглавление
- [Создание проекта](#создание-проекта)
- [Создание базы робота](#создание-базы-робота)
- [Добавление колес](#добавление-колес)
  - [Линейное движение](#линейное-движение)
  - [Вращательное движение (рулевое колесо)](#вращательное-движение-рулевое-колесо)
- [Настройка проекта](#настройка-проекта)
  - [Подготовка CMakeLists.txt](#подготовка-cmakeliststxt)
  - [Настройка файла запуска (launch)](#настройка-файла-запуска-launch)
- [Запуск!](#запуск)

## Создание проекта

Создаем в домашней директории (~) рабочую среду (workspace), в которой будем работать
```
mkdir nav_ws
cd nav_ws
colcon build
```

Поскольку у нас пустая рабочая среда, то список пакетов будет пустым
```
colcon info
```

Создадим пакет описания робота
```
mkdir src
cd src
ros2 pkg create --build-type ament_cmake bmx_description
```

Получим следующую структуру проекта:

```
bmx_description/
├── include/
│   └── bmx_description
├── src
├── CMakeLists.txt
└── package.xml
```

Удалим директории для хранения исходного кода и добавим папки **urdf** для хранения urdf/xacro файлов, **meshes** для хранения моделей и **launch** для хранения файлов запуска. В каждой папке создадим директории для хранения отдельных компонент робота. Получим следующую структуру: 

```
bmx_description/
├── launch/
├── meshes/
│   ├── base
│   ├── sensors
│   └── wheels
├── urdf/
│   ├── robots
│   └── components/
│       ├── base
│       ├── plugins
│       ├── sensors
│       └── wheels
├── CMakeLists.txt
└── package.xml
```

## Создание базы робота

Перейдем к созданию базы робота или шасси, на которую будем крепить остальные элементы. Создадим файл **base.urdf.xacro** в директории *urdf/components/base*. Оформим XML как параметризированный компонент, или макрос.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="base">

    </xacro:macro>
</robot>
```

Добавим корневое соединение base_footprint.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="base">
        <link name="base_footprint"/>
    </xacro:macro>
</robot>
```

Добавим шасси в виде соединения base_link, указав визуальную и коллизионную часть в виде правильного паралеллограмма 0.5x0.1x0.3.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.5"/>
    <xacro:property name="base_thickness" value="0.1"/>
    <xacro:property name="base_height" value="0.3"/>

    <xacro:macro name="base">
        
        ...

        <link name="base_link">
            <visual>
                <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
                <geometry>
                    <box size="${base_length} ${base_thickness} ${base_height}"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
                <geometry>
                    <box size="${base_length} ${base_thickness} ${base_height}"/>
                </geometry>
            </collision>
            <inertial>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <mass value="5.0"/>
                <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
            </inertial>
        </link>
    </xacro:macro>
</robot>
```

Свяжем соединения, добавив сустав base_link_joint.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    ...
    <xacro:property name="clearance" value="0.05"/>

    <xacro:macro name="base">
        ...
        <joint name="base_link_joint" type="fixed">
            <origin xyz="0.0 0.0 ${base_height/2 + clearance}" rpy="0.0 0.0 0.0"/>
            <parent link="base_footprint"/>
            <child link="base_link"/>
        </joint>
    </xacro:macro>
</robot>
```

Создадим модель робота. Для этого в папке *urdf/robots* создадим файл **bmx.urdf.xacro**. В файле укажем в виде аргументов название робота, пространство имен, а также подключим макрос базы, который мы написали.

```xml
<?xml version="1.0"?>
<robot name="bmx" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:arg name="robot_name" default="bmx"/>
    <xacro:arg name="robot_namespace" default="$(arg robot_name)"/>

    <xacro:include filename="$(find bmx_description)/urdf/components/base/base.urdf.xacro" />

    <xacro:base />
</robot>
```

## Добавление колес

> К этому пункту можно вернуться позднее, убедившись, что робот успешно запускается в Gazebo

На нашем велосипеде 2 колеса: переднее (с двумя осями вращения) и заднее (с одной осью вращения). Зададим отдельно два типа сустава:
- сустав с осью вращения Y (который будет создавать линейную скорость)
- сустав с осью вращения Z (будет создавать угловую скорость), который будет являться рулевым колесом.

Оба колеса запрограммируем как макросы. Создадим файлы **wheel.urdf.xacro** и **steering_wheel.urdf.xacro** по пути *bmx_description/urdf/components/wheels*. 

### Линейное движение

Начнем с колеса с осью вращения Y (файл **wheel.urdf.xacro**). В качестве аргументов будем передавать префикс имени сустава/соединения и имя родительского соединения, с которым будем соединять колесо. Назовем наш макрос "wheel". 

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="wheel" params="prefix parent">

    </xacro:macro>
</robot>
```

Добавим соединение в наш макрос. Наше соединение будет представлять из себя цилиндр, повернутый на pi/2 по оси Х. Таким образом мы моделируем наше колесо. Сделаем пока одинаковыми визуальную и коллизионную модели. Цилиндр зададим радиусом 0.1 м и высотой также 0.1 м.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="wheel" params="prefix parent">

        <link name="${prefix}_wheel_link">
            <visual>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
                <geometry>
                    <cylinder radius="0.1" length="0.1"/>
                </geometry>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
                <geometry>
                    <cylinder radius="0.1" length="0.1"/>
                </geometry>
            </collision>
        </link>

    </xacro:macro>
</robot>
```

Заметим, что любой цилиндр мы можем задать радиусом круга и высотой (что и характеризует наше колесо), поэтому сразу вынесем эти параметры в аргументы.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">


    <xacro:property name="wheel_radius" value="0.1"/>
    <xacro:property name="wheel_length" value="0.1"/>

    <xacro:macro name="wheel" params="prefix parent">

        <link name="${prefix}_wheel_link">
            <visual>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
                </geometry>
            </collision>
        </link>

    </xacro:macro>
</robot>
```

Зададим момент инерции цилиндра, в виде которого представлено наше колесо. Для этого воспользуемся [формулой расчета момента инерции для сплошного цилиндра](https://ru.wikipedia.org/wiki/%D0%A1%D0%BF%D0%B8%D1%81%D0%BE%D0%BA_%D0%BC%D0%BE%D0%BC%D0%B5%D0%BD%D1%82%D0%BE%D0%B2_%D0%B8%D0%BD%D0%B5%D1%80%D1%86%D0%B8%D0%B8), предварительно создав также переменную, хранящую массу колеса:
- I_x = I_y = 1/12 * M * (3 * R^2 + L^2)
- I_z = 1/2 * M * R^2
где M - масса цилиндра, R - радиус цилиндра, L - высота цилиндра.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">


    <xacro:property name="wheel_radius" value="0.1"/>
    <xacro:property name="wheel_length" value="0.1"/>
    <xacro:property name="wheel_mass" value="3.0"/>

    <xacro:macro name="wheel" params="prefix parent">

        <link name="${prefix}_wheel_link">
            ...
            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="${wheel_mass}"/>
                <inertia ixx="${(3*wheel_radius**2+wheel_length**2)*wheel_mass*1/12}" ixy="0.0" ixz="0.0"
                        iyy="${(3*wheel_radius**2+wheel_length**2)*wheel_mass*1/12}" iyz="0.0"
                        izz="${(wheel_mass*wheel_radius**2)*1/2}"/>
            </inertial> 
        </link>

    </xacro:macro>
</robot>
```

Наш цилиндр необходимо соединить с базой робота. Для этого создадим сустав, указав в качестве родительского соединения аргумент макроса "parent" и название созданного колеса в качестве дочернего соединения. Укажем также ось вращения с помощью тега \<axis\>. Поскольку наше колесо будет вращаться относительно оси Y, зададим единичный вектор вращения следующим образом: ```<axis xyz="0 1 0" />```. Зададим также параметры динамики - укажем коэффициент вязкого демпфирования для предотвращения колебаний объекта: ```<dynamics damping="0.2"/>```.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    ...

    <xacro:macro name="wheel" params="prefix parent">

        <joint name="${prefix}_wheel_joint" type="continuous">
            <parent link="${parent}"/>
            <child link="${prefix}_wheel_link"/>
            <axis xyz="0 1 0"/>
            <dynamics damping="0.2"/>
        </joint>

        ...

    </xacro:macro>
</robot>
```

Наконец, необходимо указать, где будет находиться сам сустав. По умолчанию сустав соединяет центр родительского и дочернего объекта. Эта позиция будет разной для переднего и заднего колеса. Отсюда вытекает необходимость передачи этой позиции в виде еще одного аргумента для нашего макроса. Добавим аргумент "*origin", который будет означать, что мы передаем тег origin. Для вставки в код этого тега воспользуемся функцией **xacro:insert_block**.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    ...

    <xacro:macro name="wheel" params="prefix parent *origin">

        <joint name="${prefix}_wheel_joint" type="continuous">
            <parent link="${parent}"/>
            <child link="${prefix}_wheel_link"/>
            <axis xyz="0 1 0"/>
            <dynamics damping="0.2"/>

            <xacro:insert_block name="origin"/>
        </joint>

        ...

    </xacro:macro>
</robot>
```

Теперь присоединим колеса к нашей модели робота. Для этого в файле **urdf/robots/bmx.urdf.xacro** подключим созданный макрос и передадим необходимые аргументы. Рассчитаем положение каждого из колес следующим образом:
1. Необходимо, чтобы колеса располагались с передней и задней частей робота. Сделаем это таким образом, чтобы крайние точки цилиндров не выходили на рамки базы. Длина базы - 0.5 м, радиус колеса - 0.1 м. Таким образом по оси Х нам нужно сдвинуть сустав от центра базы на 0.15 и на -0.15 м.
2. Необходимо, чтобы колеса располагались по центру, таким образом по оси Y не сдвигаем сустав.
3. Небходимо, чтобы по оси Z колеса были на уровне соединения base_footprint. Центр base_link находится на уровне 0.2 м от поверхности пола, радиус колеса - 0.1 м. Таким образом необходимо еще отступить 0.1 м вниз, чтобы колеса касались пола. 

Получаем:

```xml
<?xml version="1.0"?>
<robot name="bmx" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:arg name="robot_name" default="bmx"/>
    <xacro:arg name="robot_namespace" default="$(arg robot_name)"/>

    <xacro:include filename="$(find bmx_description)/urdf/components/base/base.urdf.xacro" />
    <xacro:include filename="$(find bmx_description)/urdf/components/wheels/wheel.urdf.xacro" />

    <xacro:base />

    <xacro:wheel prefix="front" parent="base_link">
        <origin xyz="0.15 0 -0.1" rpy="0 0 0"/>
    </xacro:wheel>

    <xacro:wheel prefix="rear" parent="base_link">
        <origin xyz="-0.15 0 -0.1" rpy="0 0 0"/>
    </xacro:wheel>
</robot>
```

### Вращательное движение (рулевое колесо)

Переднее колесо робота должно иметь также ось вращения Z. Для этого создадим виртуальное колесо, которое:
1. Будет дочерним к базе робота
2. Иметь одну степень свободы по оси вращения Z
3. Быть родительским по отношению к соединению переднего колеса

Создадим макрос "steering_wheel", будем как и для основного колеса передавать название, имя родителя и положение относительно родителя. Укажем виртуальное соединение, которое имеет только коллизионную часть.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="steering_wheel" params="prefix parent *origin">

        <link name="${prefix}_steering_wheel_link">
            <collision>
                <geometry>
                    <cylinder length="0.001" radius="0.01"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="0.01"/>
                <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
            </inertial>
        </link>
    </xacro:macro>
</robot>
```

Добавим сустав, который будет соединять базу и колесо.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:macro name="steering_wheel" params="prefix parent *origin">

        <joint name="${prefix}_steering_wheel_joint" type="revolute">
            <parent link="${parent}"/>
            <child link="${prefix}_steering_wheel_link"/>
            <axis xyz="0 0 1"/>
            <limit effort="100.0" lower="-0.5" upper="0.5" velocity="10" />
            <xacro:insert_block name="origin"/>

        </joint>

        ...
    </xacro:macro>
</robot>
```

Присоединим виртуальное колесо к базе.

```xml
<?xml version="1.0"?>
<robot name="bmx" xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:arg name="robot_name" default="bmx"/>
    <xacro:arg name="robot_namespace" default="$(arg robot_name)"/>

    <xacro:include filename="$(find bmx_description)/urdf/components/base/base.urdf.xacro" />
    <xacro:include filename="$(find bmx_description)/urdf/components/wheels/wheel.urdf.xacro" />
    <xacro:include filename="$(find bmx_description)/urdf/components/wheels/steering_wheel.urdf.xacro" />

    <xacro:base />

    <xacro:steering_wheel prefix="front" parent="base_link">
        <origin xyz="0.15 0 -0.1" rpy="0 0 0"/>
    </xacro:steering_wheel>

    <xacro:wheel prefix="front" parent="front_steering_wheel_link">
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:wheel>

    <xacro:wheel prefix="rear" parent="base_link">
        <origin xyz="-0.15 0 -0.1" rpy="0 0 0"/>
    </xacro:wheel>
</robot>
```

## Настройка проекта

### Подготовка CMakeLists.txt

Наша база робота готова, теперь необходимо запустить нашу модель. Прежде всего настроим CMakeLists.txt, чтобы после сборки проекта файлы были доступны при запуске launch-скриптов. Для этого перейдем в CMakeLists.txt и добавим в конец файла следующие строки:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bmx_description)

...

install(
  DIRECTORY launch meshes urdf 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

### Настройка файла запуска (launch)

Для запуска проекта необходимо настроить launch-файл. Для этого в папке */launch* создадим файл **description.launch.py**. Этот файл является скриптом на Python. Содержание этого файла регламентировано. При запуске файла ROS ищет функцию generate_launch_description(), возвращающую объекта класса LaunchDescription. Таким образом изначально файл выглядит следующим образом:

```python

from launch import LaunchDescription

def generate_launch_description():

	# Create the launch description and populate
	ld = LaunchDescription()

	return ld
```

Создадим переменную robot_xacro_path, в которой будем хранить путь до xacro-файла описания робота. Укажем путь до файла, используя встроенную функцию os.path.join языка и функцию get_package_share_directory из ROS2. Предварительно подключим библиотеки для использования этих функций. Получим

```python

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    robot_xacro_path = os.path.join(get_package_share_directory('bmx_description'), 'urdf/robots', 'bmx.urdf.xacro')

	...
```

Добавим аргумент в launch-файл, в нашем случае это флаг **use_sim_time** и пространство имен **namespace**, которое мы передадим в модель робота. Для объявления аргументов используем объект класса LaunchConfiguration, предварительно его импортировав.

```python

...
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	...
    # Create the launch configuration variables
	namespace = LaunchConfiguration('namespace')
	use_sim_time = LaunchConfiguration('use_sim_time')
    ...
```

Для добавления значений аргументов по умолчанию и их описания создадим объекты класса DeclareLaunchArgument, также его предварительно подключив в файле.

```python

...
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
	...
	# Declare the launch arguments
	declare_namespace_cmd = DeclareLaunchArgument(
		name='namespace',
		default_value='bmx',
		description='Top-level namespace'
	)

	declare_use_sim_time_cmd = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='false',
		description='Use simulation (Gazebo) clock if true'
	)
    ...
```

Опишем узел (node), который будет публиковать описание робота и трансформации между соединениями робота. Для этого создадим объект класса Node (подключив класс в шапке файла) и зададим его параметры.

> robot_state_publisher преобразует показания суставов робота в трансформации (статические и динамические). Поэтому этот узел подписывается на топик **/clock** для указания времени трансформации

```python

...
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
	...
	# Declare the launch nodes
	robot_state_publisher_cmd = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		namespace=namespace,
		parameters=[{'robot_description': ParameterValue(Command(['xacro ', robot_xacro_path, ' robot_namespace:=', namespace]), value_type=str)},
					{'use_sim_time': use_sim_time}],
		remappings=[('/tf','tf'),
					('/tf_static','tf_static')]
	)
    ...
```

После создания необходимых объектов их необходимо добавить в возвращаемый объект LaunchDescription. Для этого используется метод add_action().

```python

...

def generate_launch_description():
	...

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_namespace_cmd)

	# Add the actions to launch all of the nodes
	ld.add_action(robot_state_publisher_cmd)

	return ld
    ...
```

## Запуск!

Файл запуска готов. Выполняем сборку проекта: ```colcon build```
Запускаем launch-файл, указав пространство имен для терминала.
```bash
source ~/nav_ws/install/setup.bash
ros2 launch bmx_description description.launch.py
```

После запуска launch-файла робот не запустился в Gazebo, как и не запустился сам Gazebo. Все потому, что мы не указали файл запуска для Gazebo и файл запуска для спавна нашего робота в Gazebo.

Мы можем проверить, правильно ли мы создали структуру робота и заполнили файлы. Для этого в новом терминале пропишем команду ```ros2 topic list```. Мы должны увидеть топики, публикующие URDF-описание робота (/bmx/robot_description), топики, публикующие трансформации между соединениями робота (/bmx/tf и /bmx/tf_static). Мы можем проверить наличие данных с помощью соответствующих команд.

```bash
# Построение дерева соединений (links) робота
sudo apt install ros-jazzy-rqt-tf-tree
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=/bmx/tf -r /tf_static:=/bmx/tf_static

# Получение URDF-файла робота
ros2 topic echo /bmx/robot_description -f
```