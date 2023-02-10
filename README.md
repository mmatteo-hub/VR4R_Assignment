# <img src="https://user-images.githubusercontent.com/62358773/217943596-e9a45efb-da5a-40ae-919a-16197d4aaf51.png" width="7%" height="7%"> VR4R_Assignment

## <img src="https://user-images.githubusercontent.com/62358773/217944729-f7aee557-c380-4701-9fdb-ba87cd48a881.png" width="5%" height="5%"> Introduction
Our project focuses on conducting periodic surveillance in difficult-to-reach areas, especially in emergency situations. To achieve this, we use a team of UAVs for aerial observation, which offers increased efficiency and ability to access hard-to-reach locations. To maintain communication and coordination between UAVs, we employ a relay chaining approach. The system is controlled using the Robot Operating System (ROS) framework, with nodes designed to implement the coverage algorithm and relay chain system. Our goal is to develop a method for efficient and effective reconnaissance in emergency situations in areas with difficult terrain, utilizing UAVs and relay chaining.

## <img src="https://user-images.githubusercontent.com/62358773/217943575-c86e500e-b4c0-458b-bd86-f08cfad44e68.png" width="5%" height="5%"> Tools used and brief description
* WLS Windows Subsystem for Linux (v. 1.0.3.0) is a compatibility layer that enables users to run Linux applications on Windows 10 seamlessly, without the need for a virtual machine.

* ROS: Robot Operating System (v. Noetic) is an open-source framework for building robot applications. It provides libraries, tools, messaging and communication protocols for exchanging data between components of a robot system.

* UE: Unreal Engine (v. 4.27.2) is a game engine and development platform for creating high-quality video games, architectural visualizations, and other interactive 3D experiences. It provides tools for 3D modeling, animation, and simulation, and a powerful scripting system.

* AirSim: (v. 1.8.1) is an open-source simulation platform for testing autonomous systems, especially drones, in a virtual environment. It provides a high-fidelity simulation environment that accurately models physics and dynamics of aerial vehicles, and a variety of sensor models for testing perception and control algorithms.

    * AirSim ROS Wrapper: (v. 1.8.1) is a software package that connects the AirSim simulation platform with the ROS framework. It allows users to interface AirSim with ROS and enables the use of ROS tools and libraries for developing and testing autonomous systems in a simulated environment.

## <img src="https://user-images.githubusercontent.com/62358773/217943552-91531ae2-6c37-4034-aac4-2f33d189d112.png" width="5%" height="5%"> Run the code
In order to run the code follow these steps:
```bash
# Run the AirSim ROS Wrapper
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=192.168.1.17

# Run the Drone Coverage
python ./run.py

# Load the map
rosservice call /graph_loader/load_graph "location: '/path_to_workspace/src/graph_loader/graphs/mountain_graph.json'"

# Compute a new path for the drones
rosservice call /graph_loader/compute_path "node_start: 'p0' node_goal: 'p2'"
```

## <img src="https://user-images.githubusercontent.com/62358773/217943537-115468ea-6e9b-4997-92ac-9ef5c790c1b9.png" width="5%" height="5%"> Possible improvements
The solution could be improved by transmitting information through a relay of drones controlled in Virtual Reality using a First Person View headset, providing an immersive and intuitive experience for the operator, leading to quicker and more effective information transmission. The VR-FPV interface should also allow individual control of each drone to monitor the mountain in real-time, enabling quick identification and response to potential hazards.

## <img src="https://user-images.githubusercontent.com/62358773/217944707-e4972167-1fa2-423b-a350-5a18715edb45.png" width="5%" height="5%"> Students and Contacts
[Davide Leo Parisi](https://github.com/dpareasy) <a href="mailto:s4329668@studenti.unige.it" >
<img align="center" src="https://user-images.githubusercontent.com/81308076/155858753-ef1238f1-5887-4e4d-9ac2-2b0bb82836e2.png" alt="dpareasy" height="20" width="20" /> </a> <br>
[Matteo Maragliano](https://github.com/mmatteo-hub) <a href="mailto:s4636216@studenti.unige.it" >
<img align="center" src="https://user-images.githubusercontent.com/81308076/155858753-ef1238f1-5887-4e4d-9ac2-2b0bb82836e2.png" alt="IlMusu" height="20" width="20" /> </a> <br>
[Mattia Musumeci](https://github.com/IlMusu) <a href="mailto:s4670261@studenti.unige.it" >
<img align="center" src="https://user-images.githubusercontent.com/81308076/155858753-ef1238f1-5887-4e4d-9ac2-2b0bb82836e2.png" alt="mmatteo-hub" height="20" width="20" /> </a> <br>
[Sara Sgambato](https://github.com/sarasgambato) <a href="mailto:s4648592@studenti.unige.it" >
<img align="center" src="https://user-images.githubusercontent.com/81308076/155858753-ef1238f1-5887-4e4d-9ac2-2b0bb82836e2.png" alt="sarasgambato" height="20" width="20" /> </a>
