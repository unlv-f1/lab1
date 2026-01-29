# Lab 1: Intro to ROS 2

## Learning Goals

- Getting familiar with ROS 2 workflow
- Understanding how to create nodes with publishers, subscribers
- Understanding ROS 2 package structure, files, dependencies
- Creating launch files

## Before you start

It's highly recommended to install Ubuntu natively on your machine for
development in simulation. As an alternative, you may use the
simulator within a Docker container. While the F1TENTH group supports
native and containered implementations, at UNLV we are unable to
support both. Our instructions are limited to Docker containers to
support the greatest number of students, but it is highly recommended
that students pursue a native installation from the instructions found
[here](https://github.com/f1tenth/f1tenth_gym_ros). 

These instructions will install the F1TENTH Gym and create a workspace
for this lab in separate locations. Below is the local file structure the
instructions assume **outside of the container**.

	${HOME}/
	  |
	  +-- sim_ws/
	  |     |
	  |     +-- src/
	  |          |
	  |          `-- f1tenth_gym_ros/        -- The gym
	  |
	  `-- lab1_ws/                           -- This lab

Below is the file expected file structure inside the container. Note that this
is in `/`, the filesystem root, so you'll see other system files within this
folder.

```
/ (Filesystem root)

    ... (Other system files) ...
    
    lab1_ws/
        ...
    
    ... (Other system files) ...

    sim_ws/
        src/
            f1tenth_gym_ros/

    ... (Other system files) ...
```
	  
Preparing the gym environment ensures the Docker resources and other
tools are properly installed to prepare the first lab. Keep this
layout in mind while working on the assignment.

## 1. Overview

The goal of this lab is to get you familiar with the ROS 2
workflow. You'll have the option to complete the coding segment of
this assignment in either Python or C++. However, we highly recommend
trying out both since this will be the easiest assignment to get
started on a new language, and the workflow in these two languages are
slightly different in ROS2 and it's beneficial to understand
both. Future labs will provide templates in both.

In this lab, it'll be helpful to read these tutorials if you're stuck:

[https://docs.ros.org/en/foxy/Tutorials.html](https://docs.ros.org/en/foxy/Tutorials.html)

[https://roboticsbackend.com/category/ros2/](https://roboticsbackend.com/category/ros2/)

## 2. Getting ready **(Docker)**

Install Docker on your system following the instructions and
references:

- Docker installation instructions [here](https://docs.docker.com/get-docker/).
- Docker reference documentation [here](https://docs.docker.com/reference/). 

These instructions support all students. Those with an NVIDIA GPU may
wish to follow the installation instructions for the 3D accelerated
simulation enviornment
[here](https://github.com/f1tenth/f1tenth_gym_ros?tab=readme-ov-file#with-an-nvidia-gpu). 

**Install the following dependencies:**

noVNC will be used to forward the display

- Install **Docker** from [here](https://docs.docker.com/get-docker/).
- Install **docker-compose** from
  [here](https://docs.docker.com/compose/install/).

1. Clone the `f1tenth_gym_ros` repository
   [https://github.com/f1tenth/f1tenth_gym_ros] 
   
    ```bash
	cd ~/sim_ws/src/
	git clone https://github.com/f1tenth/f1tenth_gym_ros
	```

2. Start the docker composition

    ```bash
	cd ~/sim_ws/src/f1tenth_gym_ros
	docker build -t f1tenth_gym_ros -f Dockerfile .
	docker compose up
	```

3. In your browser, navigate to
[http://localhost:8080/vnc.html](http://localhost:8080/vnc.html),
you should see the noVNC logo with the connect button. Click the
connect button to connect to the session. If successful, your browser
tab should look similar to to:

![noVNC success](img/novnc-success.png)


## 3. Launching the Simulator
 This step verifies the F1TENTH gym simulator has been installed
correctly, for this lab it will not be used but it serves as
verification point.

1. With the docker composition "up" in one terminal, open a second
   terminal and enter the docker container for the simulator.

    ```bash
	docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
	# source /opt/ros/foxy/setup.bash
	# source install/local_setup.bash
	# export PS1="(sim-1) $PS1"
	(sim-1) # ros2 launch f1tenth_gym_ros gym_bridge_launch.py
	```

In your web browser tab for
[http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) an
rviz window should pop up showing the simulation. If successful, your
browser tab should look similar to: 

![f1tenth gym success](img/gym-success.png)

2. Test out driving in the simulator by running the following command 
	in a new terminal:

	```bash
	root@16efddc49186:/sim_ws# ros2 run teleop_twist_keyboard teleop_twist_keyboard
	``` 
	The keys corresponding to vehicle controls are displayed in the terminal. To drive, leave this terminal
	open.

3. After verifying the simulator is running and the car can drive, stop the container by
   pressing CTRL-C (hold the control key and hit C) in the terminal
   you started it in.
   
    ```bash
	// (sim-1) running the gym_bridge_launch.py ...
	CTRL-C
	(sim-1) # 
	```

## 4. Creating a workspace and bind mounting it into the Docker container

1. Begin by cloning this repository into `~/lab1_ws`

    ```bash
	cd ~
	git clone https://github.com/unlv-f1/lab1 lab1_ws
	# To get the absolute path do:
	realpath lab1_ws
	```

2. Collect your user id and group id with the `id` command by:

    ```bash
	id -u 
	3000 # UID
	id -g 
	4000 # GID
	```

3. With the docker composition 'up', enter the container bind-mounting
   the source repository into the container:

    ```bash
	mkdir ~/lab1_ws/src
	docker run -it -v <abspath>/lab1_ws/src:/lab1_ws/src \
	    --name f1tenth_lab1 ros:foxy
	```

	This will bind mount a directory from the host at
	`<abspath>/lab1_ws/src` to the path `/lab1_ws/src` while starting
	the container. The container will be named `f1tenth_lab1`. You'll 
	then have access to a terminal inside the container. If you wish
	to edit files outside of the container, you will need to change
	the permissions of the files from *inside* the container.

    ```bash
	cd /lab1_ws/src
	chown -R 3000:4000 *        # UID:GID from Step 4.2
	```

    *Note*, you will need to do this for every file created from
	within the container, every time.

`tmux` is recommended when you're working inside a container. It could
be installed in the container via: `apt update && apt install
tmux`. `tmux` allows you to have multiple `bash` session in the same
terminal window. This will be very convenient working inside
containers. A quick reference on how to use tmux can be found
[here](https://www.redhat.com/sysadmin/introduction-tmux-linux). You
can start a session with `tmux`. Then you can call different `tmux`
commands by pressing `ctrl+B` first and then the corresponding
key. For example, to add a new window, press `ctrl+B` first and
release and press `c` to create a new window. You can also move around
with `ctrl+B` then `n` or `p`. 

A cheatsheet for the original tmux shortcut keys can be found
[here](https://tmuxcheatsheet.com/). To know about how to change the
configuration of tmux to make it more useable (for example, if you
want to toggle the mouse mode on when you start a tmux bash session or
change the shortcut keys), you can find a tutorial
[here](https://www.hamvocke.com/blog/a-guide-to-customizing-your-tmux-conf/). 

## 5: ROS 2 Basics

Now that we have the access to a ROS 2 environment, let's test out the
basic ROS 2 commands. In f1tenth_lab1 container terminal, run: 

```bash
# Within the container
source /opt/ros/foxy/setup.bash
ros2 topic list
```
You should see two topics listed:
```bash
/parameter_events
/rosout
```

If you need multiple terminals and you're inside a Docker container,
use `tmux`. 


## 6. Creating a Package

First, start a new terminal. Remember that whenever you start a new terminal,
you must always source your underlay:

```bash
source /opt/ros/foxy/setup.bash
```

Next, navigate to `/lab1_ws/src`:

```bash
cd /lab1_ws/src
```

Create a ROS 2 Python package named `lab1_pkg` using:

```bash
ros2 pkg create --build-type ament_python lab1_pkg
```

In your directory, you should see that it has created a new folder called
`lab1_pkg`, which you can check that it exists using `ls`.

Next, we will practice building our package.

Change your current directory back to your workspace folder (`/lab1_ws`). **The rest of the**
**commands in this section will be in this folder.**

```bash
cd /lab1_ws
```

Then build all packages in your workspace using:

```bash
colcon build
```

Then, we must set up our environment so that it also aware of the packages
inside this workspace. To do this, source the workspace's **overlay** using:

```bash
source install/local_setup.bash
```

It's a good habit to source the workspace's overlay **every time** that you
rebuild the workspace!

Now, let's check the package list using:

```bash
ros2 pkg list
```

This command prints out the names of every single ROS 2 package that the
current terminal recognizes. Here, we'll check if the `lab1_pkg` exists. To do
this, we'll feed the list of packages to a command-line utility called `grep`,
which filters lines containing a specific substring. Here is the command:

```bash
ros2 pkg list | grep lab1_pkg
```

On success, it will give a single line of output:

```
lab1_pkg
```

On failure, it will give an empty output, indicating that the terminal does not
recognize your `lab1_pkg` ROS 2 package. If this happens, try rebuilding the
package and source your overlay again.

Finally, we'll add some dependencies to the `package.xml` file:

```xml
<license>TODO: License declaration</license>

<!-- Begin new tags -->
<depend>rclpy</depend>
<depend>ackermann_msgs</depend>
<exec_depend>ros2launch</exec_depend>
<!-- End new tags -->

<test_depend>ament_copyright</test_depend>
```

The purpose of these is to know what packages to install when installing this
package. Next, we'll install the dependencies. First, make sure all packages
are updated:

```bash
apt-get update
```

Then, ensure that all of the package dependencies are installed using:

```bash
rosdep install --from-paths src --ignore-src -y
```

Upon success, you should get the following message at the end of the output:

```
All required rosdeps installed successfully
```

## Deliverables

This section details the deliverables for this project. (Don't feel
overwhelmed, the rest of the sections should guide you; this is an high-level
overview.)

For this project, you will have **three** deliverables.

Your **first deliverable** is a ROS 2 package which involve publishers,
subscriptions, and a launch file. It will contain two nodes:

* `talker_node`
  * Takes two float parameters `v` and `d`.
  * Every 2 seconds, publishes to `/drive` an [`AckermannDriveStamped`][ackermanndrivestamped] message, setting its `drive.speed` to `v` and `drive.steering_angle` to `d`.

* `relay_node`
  * Subscribes to the `/drive` topic.
  * Whenever it receives a message from `/drive`, it publishes to `/drive_relay` a modified [`AckermannDriveStamped`][ackermanndrivestamped] message with `drive.speed` and `drive.steering_angle` multiplied by 3.

Then, it will also contain a launch file which:

* Accepts two launch arguments `v` and `d`
* Launches `talker_node` and `relay_node`, passing `v` and `d` to `talker_node`.

The **second deliverable** is a filled-out `SUBMISSION.md` file, which is
available in this repository. 

## 7. Creating nodes with publishers and subscribers

For this section, we will create two nodes `talker_node` and `relay_node`.
Change your current directory to `/lab1_ws/src/lab1_pkg`. You should see the following package
structure:

```
/lab1_ws/src/lab1_pkg/
    lab1_pkg/
    __init__.py
    test/
    ...
    resource/
    ...
    package.xml
    setup.cfg
    setup.py
```

For this section, we will create two nodes `talker_node` and `relay_node`.

If you are stuck, here are some resources to guide your implementation:

* [Understanding topics - Background](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#background)
* [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

### Talker Node

Let's start with `talker_node`. First, in the *inner* `lab1_pkg/` directory,
create a new file named `talker.py`. (If you are doing this via command line,
you can use `nano talker.py` to do so. You can also choose to enter
`touch talker.py` to create the file then edit the file with a text editor
of your choice **outside** the container.) The inner `lab1_pkg/` directory
should look like this:

```
/lab1_ws/src/lab1_pkg/lab1_pkg/
  __init__.py
  talker.py
```

Next, copy and paste the following starter code:

```python
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class TalkerNode(Node):
    def __init__(self) -> None:
        super().__init__("talker_node")

        self.declare_parameter("my_parameter")
        self.my_parameter_value = self.get_parameter("my_parameter").value

        self.create_timer(2.0, self.timer_callback)

        self.my_topic_pub = self.create_publisher(
            AckermannDriveStamped, "/my_topic", 10
        )

        self.get_logger().info(
            f"talker_node initialized with {self.my_parameter_value}"
        )

    def timer_callback(self) -> None:
        self.get_logger().info("Timer callback called.")

        msg = AckermannDriveStamped()
        msg.drive.speed = self.my_parameter_value

        self.my_topic_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TalkerNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

To summarize the important bits of this code:

* `__init__`
  * Declares a parameter `my_parameter` and stores it as an attribute.
  * Creates a timer which will call `timer_callback` every 2 seconds.
  * Creates a publisher to `/my_topic` which will send messages of type
    `AckermannDriveStamped`.
* `timer_callback`
  * Creates a message object of type `AckermannDriveStamped`.
  * Sets its `drive.speed` attribute to the value of its parameter.
  * Publishes the message using `self.my_topic_pub`.

Now, let's test our code. Before we do that, we need to make an proper entry
point for our Python package. Navigate to the `setup.py` file. You should
something like:

```python
from setuptools import setup

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='edericoytas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

Navigate to `entry_point={`. Add the string `"talker = lab1_pkg.talker:main"`
to the list for `console_scripts` It should look like:

```python
entry_points={
    "console_scripts": [
        "talker = lab1_pkg.talker:main",
    ],
},
```

Once you are finished, since you changed your package code, you must rebuild
your package. Run:

```bash
cd /lab1_ws
colcon build
```

> You should always build in your workspace folder, not anywhere else!

Then, as always after a build, resource your overlay:

```bash
source install/local_setup.bash
```

Now, our talker entry point should now be findable. To test it now, use:

```bash
ros2 run lab1_pkg talker --ros-args -p my_parameter:=15.0
```

This runs a new node called `talker_node`.

Once you do this, it will run a new node called `talker_node` and output the
following to the console.

```
[INFO] [1769673012.426746172] [talker_node]: talker_node initialized with 15.0
[INFO] [1769673014.392853447] [talker_node]: Timer callback called.
[INFO] [1769673016.392688042] [talker_node]: Timer callback called.
[INFO] [1769673018.392708953] [talker_node]: Timer callback called.
[INFO] [1769673020.392705936] [talker_node]: Timer callback called.
```

Here, it should print the "initialized" message, and print the timer callback
continuously every two seconds.

Now, **while** the node is running, it is using the `/my_topic` topic. In another terminal, check the topic list using:

```bash
source /opt/ros/foxy/setup.bash
ros2 topic list
```

You should see the following:

```
/my_topic
/parameter_events
/rosout
```

Notice that the topic list contains `/my_topic` since our node is publishing to it
now. We can examine what messages are being passed into `/my_topic` using:

```bash
ros2 topic echo /my_topic
```

You should see the following output published **repeatedly**:

```
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
drive:
  steering_angle: 0.0
  steering_angle_velocity: 0.0
  speed: 15.0
  acceleration: 0.0
  jerk: 0.0
---
```

Try experimenting with changing the value of `my_parameter` in your `ros2 run` 
commmand and see what value is given in `/my_topic`.

### Your Talker Node Tasks

Your job will be to adapt the talker node so that it:

* Takes two float parameters `v` and `d`.
* Every 2 seconds, publishes to `/drive` an [`AckermannDriveStamped`][ackermanndrivestamped] message, setting its `drive.speed` to `v` and `drive.steering_angle` to `d`.

Here are some questions to consider:

* How do you change the topic of `self.my_topic_pub` so it publishes to `/drive` instead?
* How do you change it so that it reads two parameters `v` and `d` instead?
* How do you set `drive.speed` and `drive.steering_angle` to `v` and `d`?

Use `ros2 run` and `ros2 topic echo` to ensure that your ROS 2 node runs correctly.

> Make sure to rebuild every time you change your files and want to run your nodes!

> Note: The syntax for multiple parameters is `ros2 run lab1_pkg --ros-args -p v:=2.0 -p d:=3.0`. You need to use a `-p` flag for each parameter.

Once you have finished these tasks, you may move to the next part.

### Relay Node

Let's move onto `relay_node`. In the inner lab1_pkg/ directory, create a new file named *relay.py*. The inner `lab1_pkg/` directory should now look like this:

```
/lab1_ws/src/lab1_pkg/lab1_pkg/
  __init__.py
	relay.py
  talker.py
```

Next, copy and paste the following starter code:

```python
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class RelayNode(Node):
    def __init__(self) -> None:
        super().__init__("relay_node")

        self.create_subscription(
            AckermannDriveStamped,
            "/drive",
            self.drive_callback,
            10,
        )

        self.get_logger().info("relay_node initialized")

    def drive_callback(self, drive_msg: AckermannDriveStamped) -> None:
        self.get_logger().info("my_topic_callback called")
        drive_msg.drive.speed = drive_msg.drive.speed * 3.0
        drive_msg.drive.steering_angle = drive_msg.drive.steering_angle * 3.0
        self.get_logger().info(f"  new speed: {drive_msg.drive.speed}")
        self.get_logger().info(
            f"  new steering_angle: {drive_msg.drive.steering_angle}"
        )


def main() -> None:
    rclpy.init()
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

To summarize the important bits of this code:

* `__init__`
  * Creates a subscription to `/drive` which calls `drive_callback` every time
    `/drive` receives a new message.
* `drive_callback`
  * Modifies the `drive.speed` and `drive.steering_angle` values.
  * Prints the new values to the console.

Next, we need to add an entry point, so we need to add another line to
`setup.py` as we did in the last step. After adding it, `entry_points` should
look like this:

```python
entry_points={
    "console_scripts": [
        "talker = lab1_pkg.talker:main",
        "relay = lab1_pkg.relay:main",
    ],
},
```

Finally, rebuild and re-source:

```bash
colcon build
source install/local_setup.bash
```

Then, to test it, we need both nodes running at the same time. In one terminal,
run the following (in `/lab1_ws`):

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 run lab1_pkg talker --ros-args -p v:=2.0 -p d:=3.0
```

Then, in another terminal, run (in `/lab1_ws`):

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 run lab1_pkg relay
```

For your second termanal (relay), you should see the following output print
every two seconds:

```
[INFO] [1769676352.619522949] [relay_node]: my_topic_callback called
[INFO] [1769676352.620098725] [relay_node]:   new speed: 6.0
[INFO] [1769676352.620635937] [relay_node]:   new steering_angle: 9.0
```

### Your Relay Node Tasks

Your job will be to adapt the relay node so that:

* Whenever it receives a message from `/drive`, it publishes to `/drive_relay` a modified [`AckermannDriveStamped`][ackermanndrivestamped] message with `drive.speed` and `drive.steering_angle` multiplied by 3.

Here are some questions to consider:

* How do you create a publisher to `/drive_relay`?
  * Hint: See your talker node.
* How do you send your modified message?
  * Hint: See `drive_callback` and your publisher object which publishes to
    `/drive_relay`.

To test your implementation, ensure that both nodes are running and use
`ros2 topic echo /drive_relay` to monitor the messages flowing through that
topic.

## 8. Creating a launch file

> References to launch files may be found here:
> [https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html] 

Finally, we will create a launch file named `lab1_launch.py`. Navigate to your
package root `/lab1_ws/src/lab1_pkg/`. Create a new directory called
`launch/` and create a new file in it called `lab1_launch.py`. Your directory
should look something like this:

```
/lab1_ws/src/lab1_pkg/
	lab1_pkg/
    __init__.py
		relay.py
		talker.py
	launch/
		lab1_launch.py
	test/
    ...
	resource/
    ...
	package.xml
	setup.cfg
	setup.py
```

Then, copy and paste the following code in it:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    my_argument_arg = DeclareLaunchArgument("my_argument")

    talker_node = Node(
        package="lab1_pkg",
        executable="talker",
        name="talker_node",
        parameters=[
            {
                "v": LaunchConfiguration("my_argument"),
                "d": LaunchConfiguration("my_argument"),
            }
        ],
    )
    relay_node = Node(
        package="lab1_pkg",
        executable="relay",
        name="relay_node",
    )

    return LaunchDescription([my_argument_arg, talker_node, relay_node])
```

To summarize the important bits of this code:

* `generate_launch_description` generates a `LaunchDescription` object which
  when used for launch:
  * Declares a launch argument named `my_argument`
  * Runs `talker_node` with its two parameters `v` and `d` set to
    `my_argument`.
  * Runs `listener_node`.

Let's test our launch file. To do this, we need to add our launch file as a
data file so it is available within our package. In *setup.py*, navigate to
the `data_files` and modify it so that it has a new line:

```python
data_files=[
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ("share/" + package_name + "/launch", ["launch/lab1_launch.py"]),
],
```

Then, rebuild and re-source. Afterward, launch the file using:

```bash
ros2 launch lab1_pkg lab1_launch.py my_argument:=5.0
```

You should see the following being repeated every two seconds:

```
[talker-1] [INFO] [1769678953.887773254] [talker_node]: Timer callback called.
[relay-2] [INFO] [1769678953.889046169] [relay_node]: my_topic_callback called
[relay-2] [INFO] [1769678953.889475521] [relay_node]:   new speed: 15.0
[relay-2] [INFO] [1769678953.889809949] [relay_node]:   new steering_angle: 15.0
```

### Your Tasks

Adapt the source code so that it:

* Accepts two launch arguments `v` and `d`
* Still launches `talker_node` and `relay_node`, passing `v` and `d` to `talker_node`.

Some questions to consider:

* How can I change the launch arguments to `v` and `d`?
* How can I pass these arguments to `talker_node`?
  * Hint: Here is a simple example Python dictionary with two elements:
    `{'a': 1, 'b': some_value, 'c': 3.0}`

## 9. ROS 2 commands

After you've finished all the deliverables, launch a new terminal and source
your underlay:

```bash
source /opt/ros/foxy/setup.bash
```

Then, test out these ROS 2 commands (you may use these anywhere in your
filesystem):

```bash
ros2 topic list
ros2 topic info /drive
ros2 topic echo /drive
ros2 topic info /drive_relay
ros2 topic echo /drive_relay
ros2 node list
ros2 node info talker
ros2 node info relay
```

## 10. Deliverables and Submission

Once you are finished coding, fill in the answers to the questions listed
in `SUBMISSION.md` file. Then, provide a screenshot of the output 
of the launch file running your nodes. Label the screenshot **`Output`**.

Once you are finished, upload all of your source code (including your
completed `SUBMISSION.md` file) to a **private** Github repository.

Add the TA as a collaborator to your repository.

If you need help, reference the following:
[https://docs.github.com/en/account-and-profile/setting-up-and-managing-your-personal-account-on-github/managing-access-to-your-personal-repositories/inviting-collaborators-to-a-personal-repository] 

## 11. Grading Rubric

- Correctly creating the package: **25** Points
- Correctly creating the nodes: **25** Points
- Correctly creating the launch file: **25** Points
- Written questions: **25** Points

## Troubleshooting 

### Cannot use the container

If there are problems entering or exiting the docker container, remove
it: 

```bash
docker container rm f1tenth_lab1
```

### Re-entering the container after exiting

```bash
docker start f1tenth_lab1
docker attach f1tenth_lab1
```

### Exiting the container

To exit the container hold down the control key and press the d key.
<!---From Breanna: Must mention tmux as students within a docker container may 
not be able to access the session without- need to utilize multiple 
"terminals" in a session.--->
### Utilizing Multiple Terminals in the Container
`tmux` is included in the container, so you can create multiple
 bash sessions in the same terminal. This allows students the ability 
 to run multiple commands in the docker container- addressing the issue 
 of not being able to run multiple commands within the docker container.

- A quick reference on how to use tmux can be found 
 	[here](https://www.redhat.com/sysadmin/introduction-tmux-linux).   

- A cheatsheet for the original tmux shortcut keys can be found 
       [here](https://tmuxcheatsheet.com/). 

- To know about how to change 
       the configuration of tmux to make it more useable (for example, 
       if you want to toggle the mouse mode on when you start a tmux 
       bash session or change the shortcut keys), you can find a 
       tutorial 
       [here](https://www.hamvocke.com/blog/a-guide-to-customizing-your-tmux-conf/).  


[ackermanndrivestamped]: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
