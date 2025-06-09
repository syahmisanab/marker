
![Ainex Training Page](https://github.com/syahmisanab/marker/raw/main/script/addtional_read/Ainex%20Competition%20Training_page-0004.jpg)


## ROS Overview

ROS, which stands for **Robot Operating System**, is not actually an operating system in the traditional sense. Instead, it’s a powerful framework that provides all the essential tools and infrastructure needed to build complex and flexible robotic systems.

At its core, ROS helps a robot to:

- **Think**: It allows the robot to process data from sensors, make decisions based on that data, and take appropriate actions.
- **Communicate**: It enables different components of the robot—such as sensors, motors, and vision systems—to exchange information reliably and asynchronously.
- **Run code in modules**: ROS encourages you to split your robot’s software into small, reusable programs called **nodes**, which interact through well-defined communication channels.

This modular architecture makes development more organized and scalable, especially when working with a team or a large system.

---

### 🔍 Why ROS is Used for AiNex

In the context of AiNex, ROS is particularly useful because it comes with many built-in features that reduce the need to develop everything from scratch.

For example:

- It includes **motion control interfaces**, which let you control motors and actuators using a standard API.
- It supports **camera streaming**, which is essential for tasks like visual line following or object detection.
- It handles **sensor data processing**, allowing you to interpret input from IMUs, cameras, and other sensors in real time.

By using ROS, the developers of AiNex can focus on implementing specific behaviors—like walking, detecting arrows, or following a line—instead of building low-level communication or control systems.

---

### 🧠 ROS for Managing Robot Behavior

One of the biggest advantages of using ROS is that it simplifies the creation of complex, multi-step robot behaviors.

Take **line following**, for example. A traditional implementation might involve tightly coupled code handling the camera input, image processing, and motor control in one big program. In ROS, each part of this process is handled by a separate node:

- One node reads the camera feed.
- Another processes the image to detect the black line.
- A third decides how to steer based on the line's position.
- Finally, a motor control node actually moves the robot.

These nodes communicate through **topics**, which act like named message channels. This separation makes the code cleaner, easier to debug, and easier to scale.

The same concept applies to **arrow detection**. Using ROS, the AiNex robot can detect left, right, or forward arrows using computer vision and publish the direction to a topic. A different node subscribes to this topic and triggers a movement action accordingly.

---

### 🚀 Real-World Applications of ROS

ROS isn’t just an academic or hobbyist tool—it’s widely used in real-world robotics. Many companies and research labs use ROS for:

- Autonomous drones and ground robots
- Self-driving vehicle prototypes
- Industrial robot arms in manufacturing
- Robotics research in universities and labs

By learning ROS through AiNex, you're gaining experience with the same tools and principles used in professional robotics development.

---

### 🧾 Summary

To summarize:

- ROS helps you **organize** robot code into manageable pieces called nodes.
- It handles **communication** between those nodes, even across different machines.
- It provides **tools** for working with sensors, motors, and cameras.
- With ROS, it's easier to implement, test, and debug complex robotic behaviors like line following or decision-making based on visual cues.
- Learning ROS through the AiNex robot is a great way to build real-world, transferable robotics skills.

---

![Ainex Training Page](https://github.com/syahmisanab/marker/raw/main/script/addtional_read/Ainex%20Competition%20Training_page-0005.jpg)


---
## ROS Core Concepts

Understanding the core concepts of ROS is essential for building reliable and modular robotic systems. Below are the foundational elements that make up a typical ROS-based application.

---

 ### Nodes

A **node** is a basic unit of execution in ROS. Each node is a separate program that performs one task, such as reading sensor data, controlling motors, or processing images.

- You typically run multiple nodes at once, each doing a specific job.
- Nodes are usually written in Python or C++.
- ROS encourages you to keep nodes small and focused for modularity.

> For example, in a robot like AiNex, one node might handle walking, another might read the camera feed, and a third might detect arrows.

---

 ### Messages and Topics

Nodes in ROS **communicate** with each other using messages. These messages are passed through **topics**, which act like communication channels.

- A **message** is just a piece of structured data, like sensor readings or motor commands.
- A **topic** is a named pipe over which nodes exchange messages.
- One node can **publish** data to a topic, and one or more nodes can **subscribe** to it.

> Example: The camera node publishes images on `/camera/image_raw`, and the vision node subscribes to this topic to process them.

---

 ### Services

While topics are used for continuous data streams, **services** are used for request-response communication.

- A **service** consists of a pair of messages: a request and a response.
- It’s useful when one node needs to ask another to do something specific and wait for the result.

> Example: A node may request the robot to go into a "ready" pose, and the motion control node replies when it’s done.

---

 ### ROS Master

The **ROS Master** is the name server and central coordinator in any ROS system.

- It keeps track of all active nodes, topics, and services.
- It helps nodes find each other so they can communicate.
- You must start the ROS master with `roscore` before running any other ROS processes.

> Think of it like the “control tower” that lets all the other parts of the robot know who’s online and how to talk to each other.

---

### Parameters

ROS provides a **parameter server** where nodes can store and retrieve values at runtime.

- Parameters are key-value pairs, often used to configure node behavior.
- You can access them using command-line tools (`rosparam`) or in code.

> For example, you might store the speed of the robot or the HSV values for line detection here.

---

### Stacks and Packages

In ROS, code is organized into **packages**, and related packages are often grouped into **stacks**.

- A **package** contains source code, configuration files, and dependencies.
- A **stack** is just a collection of related packages that solve a higher-level task.

> For AiNex, there might be a package for line following, another for gait control, and a third for arrow detection—all organized under one stack for mobility.

---

## 🧠 Summary

Each of these concepts—nodes, topics, services, parameters, and more—contributes to the flexibility and power of the ROS framework. Understanding how they work together helps you design, debug, and scale your robotic system more effectively.

---

![Ainex Training Page](https://github.com/syahmisanab/marker/raw/main/script/addtional_read/Ainex%20Competition%20Training_page-0006.jpg)


---

## Basic ROS Commands

When working with ROS (especially in ROS 1), there are several essential command-line tools that you'll use constantly. These tools help you run nodes, inspect the system, and visualize what's happening in real-time.

Below is a breakdown of the most important basic commands every ROS developer should know.

---

### `roscore`

`roscore` is the starting point of every ROS system. It launches the ROS Master along with other essential services like the parameter server.

- You must run `roscore` **before starting any nodes or tools**.
- Without it, nodes won’t be able to find each other or communicate.

**Usage:**
```bash
roscore
```

> Tip: Run this in a dedicated terminal and leave it open while working with ROS.

---

### `rosnode`

The `rosnode` command lets you inspect and manage the nodes currently running in your ROS system.

You can:
- List active nodes
- Get detailed info about a specific node
- Ping a node to see if it’s responding

**Common commands:**
```bash
rosnode list            # See all active nodes
rosnode info /node_name # View info about a specific node
rosnode ping /node_name # Check if a node is alive
```

> This is helpful for debugging when nodes don’t seem to be talking to each other.

---

### `rosrun` and `roslaunch`

These commands are used to start nodes.

#### `rosrun`

Used to run a single node from a specific package.
```bash
rosrun <package_name> <node_name>
```

> Example: `rosrun turtlesim turtlesim_node`

#### `roslaunch`

Used to start multiple nodes and load parameters from a `.launch` file.
```bash
roslaunch <package_name> <file.launch>
```

> This is powerful because you can configure and bring up an entire robot system with one command.

---

### `rostopic`

The `rostopic` command is used to monitor and interact with topics in ROS.

You can:
- See what topics are currently active
- Echo the live data on a topic
- Publish your own data manually (for testing)

**Examples:**
```bash
rostopic list
rostopic echo /topic_name
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}' -1
```

> Use `rostopic echo` a lot—it’s one of the best ways to debug data flow between nodes.

---

### `rqt`

`rqt` is a graphical tool that provides visual introspection of ROS components.

- It can show node graphs, topic connections, logs, and more.
- It’s modular, with different plugins like `rqt_graph`, `rqt_plot`, and `rqt_console`.

**To launch the main GUI:**
```bash
rqt
```

> One of the most helpful visual tools is `rqt_graph`, which shows how nodes are connected via topics.

---

## 🧠 Summary

| Command      | Purpose                                  |
|--------------|-------------------------------------------|
| `roscore`    | Starts the ROS system                     |
| `rosnode`    | Manage and inspect active nodes           |
| `rosrun`     | Run a specific node                       |
| `roslaunch`  | Launch multiple nodes and load settings   |
| `rostopic`   | View and publish messages on topics       |
| `rqt`        | Visual GUI tools for ROS system insight   |

Mastering these commands will give you strong control over any ROS-based system and help you debug, test, and launch robot behaviors efficiently.

---

![Ainex Training Page](https://github.com/syahmisanab/marker/raw/main/script/addtional_read/Ainex%20Competition%20Training_page-0007.jpg)


---

## Python Basics for Robotics

Python is one of the most popular programming languages in robotics due to its simplicity and power. This guide introduces the core concepts you'll need to understand and work with Python code on the AiNex robot.

---

### 🐍 Running Python Scripts

Python scripts typically have the `.py` file extension. To run a Python script, you simply use the `python` or `python3` command followed by the filename.

**Example:**
```bash
python3 my_script.py
```

In ROS projects, scripts are often run with `rosrun` or through a `.launch` file.

---

### 🧾 Reading & Modifying Code

Reading code is just as important as writing it. Understanding the structure, flow, and purpose of a script will help you debug and modify it effectively.

When modifying:
- Identify what the script is doing (inputs, outputs, flow)
- Make small, incremental changes
- Test frequently to catch issues early

---

### 🔁 Control Logic

Control logic allows your program to make decisions or repeat actions based on conditions.

**Examples:**

**If-else:**
```python
if sensor_value > threshold:
    move_forward()
else:
    stop()
```

**Loops:**
```python
for i in range(5):
    print("Step", i)
```

These structures are fundamental for writing responsive robot behavior.

---

### 🧮 Variables & Data

Variables store information. You can use them to hold sensor values, flags, or configuration settings.

**Examples:**
```python
speed = 0.5
direction = "left"
detected = True
```

Python supports many data types:
- Numbers: `int`, `float`
- Text: `str`
- Boolean: `True`, `False`
- Lists, dictionaries, tuples, etc.

---

### 🧩 Functions

Functions let you wrap code in reusable blocks. They're great for organization and modularity.

**Example:**
```python
def move_forward(speed):
    print(f"Moving at speed {speed}")
```

Call functions wherever needed in your script:
```python
move_forward(0.5)
```

In robotics, functions are often used to encapsulate commands like starting motors or processing input.

---

### 📦 Imports & Libraries

Python has a rich ecosystem of built-in and third-party libraries. You import only what you need to keep your code clean and readable.

**Examples:**
```python
import math
import rospy
from geometry_msgs.msg import Twist
```

In AiNex, you'll often import ROS packages and message types to control the robot or handle sensor data.

---

## 🧠 Summary

Understanding Python basics will give you confidence to read, modify, and write scripts that control robot behavior. The more you experiment, the more fluent you'll become—just like learning a new spoken language.


---

![Ainex Training Page](https://github.com/syahmisanab/marker/raw/main/script/addtional_read/Ainex%20Competition%20Training_page-0008.jpg)


---

## Linux System Basics for Robotics

Most robotics systems—including ROS—run on Linux. Knowing how to navigate the Linux terminal, manage files, and use key system commands is essential for working effectively on a robot like AiNex.

---

### 📁 File Navigation

Linux uses a hierarchical file structure starting from the root directory `/`.

**Common commands:**
```bash
pwd             # Show current directory
ls              # List files in current directory
cd foldername   # Change directory
cd ..           # Go up one level
```

---

### 📂 File Management

You can create, delete, move, and copy files using simple commands.

```bash
touch file.txt              # Create a new file
mkdir new_folder            # Create a new folder
rm file.txt                 # Delete a file
cp source.txt dest.txt      # Copy a file
mv file.txt new_folder/     # Move a file
```

> Be careful with `rm`, especially with `-r` (recursive), as it deletes folders and contents.

---

### 📜 Running Scripts

Scripts in Linux (like `.py`, `.sh`) can be executed in several ways.

**Python script:**
```bash
python3 myscript.py
```

**Shell script:**
```bash
chmod +x script.sh   # Make it executable
./script.sh          # Run it
```

> ROS launch files (`.launch`) are run using `roslaunch`, not directly.

---

### 🔒 Permissions

Every file has permission settings that control who can read, write, or execute it.

```bash
ls -l           # View permissions
chmod +x file   # Add execute permission
chmod 644 file  # Set specific permission bits
```

> Understanding permissions is useful when your script “won’t run” or says “permission denied”.

---

### 📝 Editing & Viewing

To quickly edit or read files from the terminal, use tools like `nano`, `vim`, or `cat`.

```bash
nano file.txt       # Open a basic editor
vim file.txt        # Open advanced editor
cat file.txt        # View content in terminal
```

> `nano` is easier for beginners, while `vim` is more powerful once you get used to it.

---

### ⌨️ Terminal Shortcuts

- `Tab` – Autocomplete file and folder names
- `Ctrl + C` – Stop running program
- `Ctrl + L` – Clear screen
- `Arrow Up` – Recall previous commands
- `Ctrl + A / E` – Jump to beginning/end of line

These shortcuts can speed up your workflow drastically.

---

### 🧰 System Commands

Check system health and environment with these commands:

```bash
top            # See running processes and CPU usage
htop           # Interactive version of top
df -h          # Show disk space usage
free -h        # Show memory usage
uname -a       # System info
```

> These are handy when working on robots that might slow down or have limited resources.

---

## 🧠 Summary

Being comfortable in a Linux environment means you can:
- Navigate the file system confidently
- Create, edit, and manage scripts
- Debug permission issues
- Use terminal tools to check system status

This knowledge is essential when working on robotic platforms like AiNex, where you often need to deploy or troubleshoot code directly on the robot.

---

![Ainex Training Page](https://github.com/syahmisanab/marker/raw/main/script/addtional_read/Ainex%20Competition%20Training_page-0009.jpg)


---
## Docker Basics (for AiNex)

Docker is a tool used in many software and robotics projects—including AiNex—to simplify setup and ensure consistency across systems.

---

### 🐳 What is Docker?

Docker is a platform for running applications inside **lightweight virtual containers**. These containers bundle everything the software needs—code, dependencies, environment variables—so that it runs the same way on any machine.

In the case of AiNex, Docker is used to run the robot software in a **controlled and isolated environment**. 

You can think of it like this:
> Docker is a pre-packaged robot brain that you can turn on or off as needed.

It contains the ROS environment, motion scripts, configuration files, and all system dependencies—so you don’t need to install and configure everything manually.

---

### 💡 Why Use Docker for AiNex?

There are several reasons Docker is the perfect fit for running the AiNex software stack:

- **No Need to Install Ubuntu on Raspberry Pi**  
  Instead of replacing the Pi’s operating system, Docker runs ROS on top of the existing OS without conflict.

- **Self-Contained ROS Environment**  
  Everything AiNex needs to function—ROS packages, motion control tools, config files—lives inside the container.

- **Isolation from Host OS**  
  The ROS environment stays separate from the host’s core operating system. This makes the system more stable and easier to reset if something breaks.

---

### 🔧 Everyday Benefits

- You can easily start or stop the robot environment without rebooting.
- It makes it safer to experiment or change things—because you’re not touching the host OS directly.
- If you mess something up, you can just recreate the container from a clean image.

---

## 🧠 Summary

Docker allows AiNex to:
- Run its complete software stack inside an isolated container
- Avoid complex system-level installations on the Pi
- Make ROS easier to manage, test, and update

Whether you're debugging, deploying, or experimenting, Docker gives you a clean and repeatable environment for working with your robot.

---

![Ainex Training Page](https://github.com/syahmisanab/marker/raw/main/script/addtional_read/Ainex%20Competition%20Training_page-00010.jpg)


---

## Ainex Controller

The **AiNex Controller** is a PC-based graphical software tool used to configure, control, and program the AiNex robot's servo-based motion system. It allows users to visually create and test action sequences in a user-friendly interface.

---

### 🧠 What It Does

AiNex Controller is designed for:

- **Real-time servo control and calibration**: Adjust servo positions on the fly using sliders.
- **Designing and editing action groups**: Easily build sequences like walking, waving, or standing.
- **Testing motion sequences**: Run motions within the software before deploying them into scripts.
- **Manual adjustment**: Use sliders or numeric input to position servos precisely.
- **Loading and saving poses**: Store reusable poses like `walk_ready`, `stand`, etc., for use in ROS scripts.

---

### 🖥️ Software Interface Overview

The interface is divided into 4 key areas:

---

#### ① Servo Control Area

- Each servo has a slider for adjusting position (range: 0–1000).
- You can also set servo **deviations** (offsets) between -125 and +125.
- A **lock** button disables movement for specific servos to avoid unwanted interference during programming.

---

#### ② Action List Area

- Displays the sequence of servo positions and durations in a motion.
- You can **edit** values by double-clicking or **mirror** actions for symmetry (e.g., left ↔ right).
- Tools include: add, delete, mirror, and preview actions.

---

#### ③ Action Group Settings

- Set time duration for each action (20–30000 ms).
- Use buttons to:
  - Loosen all joints for manual posing
  - Read current angles and add them to the list
  - Insert, replace, move, or delete actions
  - Run or loop action groups
- You can **merge**, **load**, or **save** action groups from:  
  `/software/ainex_controller/ActionGroups`

---

#### ④ Servo Deviation Settings

- Adjust fine-tuned offsets for each servo to correct misalignment.
- Reset all servos to a centered position (`500`) to re-balance the robot.
- Deviation values can be read or written to the robot.

---

### 🧪 Practical Workflow

1. **Pose manually** by loosening joints and positioning limbs.
2. **Read angles** and add them into the action list.
3. **Fine-tune timing** and test motion using the preview.
4. **Save the action group** and assign it a meaningful name like `walk_ready`.
5. **Use in code**: Reference saved groups from your Python or ROS scripts.

---

## 🧾 Summary

The AiNex Controller is an essential tool for:

- Creating and perfecting robot motions
- Calibrating servo accuracy
- Saving time by testing before deploying code
- Organizing all robot poses into reusable, readable action files

Mastering this tool will make motion programming faster, smoother, and more predictable.
