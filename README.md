# Renkay - WRO 2026 Future Engineers<img width="50" alt="robotek" src="https://github.com/user-attachments/assets/bffadef9-b0aa-4810-93ff-13db445ac044" /><img src="https://em-content.zobj.net/source/apple/325/flag-peru_1f1f5-1f1ea.png" alt="Peru Flag" width="30" />

<p align="center">
  <img src="https://github.com/user-attachments/assets/0a4118c9-bc9c-422e-af6b-4e9b3267209f" alt="Banner Renkay" width="100%">
</p>

[![Instagram](https://img.shields.io/badge/Instagram-E4405F?style=for-the-badge&logo=instagram&logoColor=white)](https://www.instagram.com/robotekperu/)
[![YouTube](https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white)](https://www.youtube.com/@Robotekperu)
[![Facebook](https://img.shields.io/badge/Facebook-1877F2?style=for-the-badge&logo=facebook&logoColor=white)](https://www.facebook.com/p/Robotek-Per%C3%BA-61566493439700/)

**Welcome! 🐨✨**

We are Renkay, a team of three students participating at the 2026 World Robot Olympiad! This GitHub repository contains the documentation, code, and full development journey of our autonomous vehicle, designed and built to compete in the Future Engineers Challenge.

<br>

> [!NOTE]
> 🤔 Why is our team named *Renkay*?<br><br>
> The name *Renkay* is inspired by three words that define our team: resilience, minka, and wiñay. "**Resilience**" represents our determination to never give up; "**minka**", an ancestral tradition of working together for a common purpose, reflects our belief that the best ideas are built through collaboration; and wiñay, which means "to grow", symbolizes our commitment to continuous learning and improvement. Together, these words represent who we are as a team, and guide every step of our robotics journey.

<br>

---

# 📮 Table of Contents
1. [About the Competition](about-the-competition)
   + [World Robot Olympiad (WRO)](world-robotic-olympiad-(wro))
   + [Future Engineers Category](future-engineers-category)
   + [Challenges Overview](challenges-overview)
2. [Repository Structure](repository-structure)
3. [Meet the Team!](meet-the-team)
4. [Vehicle Overview](vehicle-overview)
   + [General Description of the Car](#general-description-of-the-car)
   + [Versions of the Car](#versions-of-the-car)
     
5. [System Setup](system-setup)
   + [Operating Environment Overview](#operating-environment-overview)
   + [Robot Operating System (ROS)](#robot-operating-system-ros-)
   + [Ubuntu](#ubuntu-)
   + [Raspberry Pi](#raspberry-pi-)
     
6. [Mobility Management](mobility-management)
   + [Steering System - Ackermann](#steering-system--ackermann)
   + [Motor and Drivetrain](#motor-and-drivetrain)
   + [3D Pieces](#3d-pieces)
   
7. [Power & Sense Management](#power--sense-management)
   + [Power Source](#power-source)
   + [Sensors Integration](#sensors-integration)
   + [BOM (Bill of Materials)](#bom-bill-of-materials)
   + [Wiring Diagram](#wiring-diagram)

8. [Obstacle Management](obstacle-management)
   + [Control Node Structure](#control-node-structure)
   + [Open Challenge](#open-challenge)
   + [Obstacle Challenge](#obstacle-challenge)
     
9. [Assembly Instructions](#7-assembly-instructions)
10. [Performance Videos](#8-performance-videos)
   

---

## 1. About the Competition
### **<ins>World Robot Olympiad (WRO)</ins>**
The **World Robot Olympiad (WRO)** is a global robotics competition which brings together students to develop their creativity, engineering, and problem-solving skills. The competition offers four categories with different challenges.

### **<ins>Future Engineers Category</ins>**
We participate in the **Future Engineers category**, designed for students aged 14–22. In this category, teams design and build an autonomous vehicle using freely chosen hardware and software. It focuses on autonomous driving, requiring the vehicle to navigate a track and respond to changing conditions.


### **<ins>Challenges Overview</ins>**
| 🔓 Open Challenge | 🚨 Obstacle Challenge |
|--------------------|--------------------|
| In the **Open Challenge**, the vehicle must autonomously navigate the track and complete three laps. The track layout and driving direction can change between rounds, and the vehicle should adapt to different configurations. | In the **Obstacle Challenge**, the vehicle must complete three laps and detects the obstacles placed on the track. After completing it, the vehicle must perform a parallel parking maneuver. |
| <img width="450" alt="Open Challenge" src="https://github.com/user-attachments/assets/4a27ae44-db66-49ba-a908-089bac1b55ff"/> | <img width="450" alt="Obstacle Challenge" src="https://github.com/user-attachments/assets/05564ce7-e8b3-4ec2-8f6c-ba8a3d05d400"/>


## 2. Repository Structure

| Folder | Content |
|--------|---------|
| [`t-photos`](t-photos) | Team photos (official one and funny one) |
| [`v-photos`](v-photos) | Photos of our vehicle (from every side, from top and bottom) |
| [`videos`](videos) | Demonstration video links of the robot in the challenges |
| [`schemes`](schemes) | Wiring diagram with pins and electronic components |
| [`src`](src) | Code of the robotic vehicle system |
| [`models`](models) | 3D printed parts and other chassis pieces |
| [`other`](other) | Extra documentation |

## 3. Meet the Team!

<table width="100%">
  <tr>
    <td width="75%" style="padding: 20px;">
      <h3><strong>Isabella Gonzales 🌟</strong></h3>
      <b>🔧 Role: </b>Team Member<br>
      <b>💬 About me:</b> Hello! My name is Isabella, I'm 17 years old, and I love robotics. I founded Robotek Perú, a club where students can learn robotics and join competitions, and this is my second time at the WRO. My favorite hobbies are singing with my choir, practicing taekwondo, and art.<br>
      <b>🌐 Contact:</b> isabellamilagros842@gmail.com
    </td>
    <td align="center" width="25%" style="padding: 20px;">
      <img src="https://github.com/user-attachments/assets/4f37460a-c3a6-42cd-93e6-db632a62fead" width="800" style="border-radius: 10px;">
    </td>
  </tr>

  <tr>
    <td width="75%" style="padding: 20px;">
      <h3><strong>Rodrigo Osorio 🦙</strong></h3>
      <b>🔧 Role: </b>Team Member<br>
      <b>💬 About me:</b> Hi. I'm Rodrigo, a 16-year-old teenager passionate about many kinds of knowledge across a wide variety of fields. This is my third time taking part of the World Robot Olympiad however, in the past I'd taken part of Robo Mission: Junior through the 2024 and 2025 seasons. Besides robotics, I'm thrilled about humanities, specially philosophy, a field where curiosity can be flow naturally. I'm excited to take part in this year's WRO season!!<br>
      <b>🌐 Contact:</b> rod10peru@gmail.com
    </td>
    <td align="center" width="25%" style="padding: 20px;">
      <img src="https://github.com/user-attachments/assets/c0f1333d-bda1-4e65-9402-9381d1af59e1" width="800" style="border-radius: 10px;">
    </td>
  </tr>

  <tr>
    <td width="75%" style="padding: 20px;">
      <h3><strong>Valeria Hurtado 🎷🐛</strong></h3>
      <b>🔧 Role: </b>Team Member<br>
      <b>💬 About me:</b> Hellooo! My name is Valeria and I’m 17 years old. I’m curious about science, technology, and how things work, which is what got me into robotics. This is my first time participating in the WRO, and I’m excited to figure things out along the way. Outside robotics, I enjoy baking, cycling, and designing things on Canva.<br>
      <b>🌐 Contact:</b> valeria.hurtado.delarosa@outlook.com
    </td>
    <td align="center" width="25%" style="padding: 20px;">
      <img src="https://github.com/user-attachments/assets/e68b56c1-9123-44ee-a194-316b82ebae8a" width="800" style="border-radius: 10px;">
    </td>
  </tr>

  <tr>
    <td width="75%" style="padding: 20px;">
      <h3><strong>Anthony Valladolid 🤓</strong></h3>
      <b>🔧 Role: </b>Coach<br>
      <b>💬 About me:</b> Hi, I am Anthony Valladolid, a Mechatronics Engineering graduate from Pontificia Universidad Católica del Perú, passionate about developing and researching emerging technologies. My main areas of interest are embedded systems, robotics, and applied artificial intelligence.<br>
      <b>🌐 Contact:</b> anthony.valladolid@pucp.edu.pe
    </td>
    <td align="center" width="25%" style="padding: 20px;">
      <img src="https://github.com/user-attachments/assets/2ce1a5c3-1f01-488f-a7aa-983ac89b57d3" width="800" style="border-radius: 10px;">
    </td>
  </tr>
</table>


## 4. Vehicle Overview
This is our car named **Sami**, it is the result of multiple versions and modifications since 2024. We have been studying, learning and testing different methods to improve our car. It has been an interesting and challenging journey in robotics, but every effort was worth it!

<p align="center">
  <img src="https://github.com/user-attachments/assets/491effc5-e55b-4c2b-aae7-920ffc49a3a9" width="70%">

</p>


### **<ins>General Description of the Car</ins>**
Our autonomous vehicle is built to take on both the Open and Obstacle Challenges at the Future Engineers competition. Running on Ubuntu with ROS, it can process information and make decisions in real time. The car uses an Ackermann steering system and a custom-built chassis to move smoothly through turns and straight paths. A LiDAR sensor helps it detect the field walls, while the camera identifies traffic signs and obstacles. With this setup, the car can adapt its route, count laps, and complete the course efficiently.
<br>

| Front | Left | Right |
| :--: | :--: | :--: | 
| <img src="https://github.com/user-attachments/assets/57818eb1-a1db-4454-9928-7ac42b46e3ae" width="90%" /> | <img src="https://github.com/user-attachments/assets/3c8d3351-0cf4-471f-be21-785de03897c0" width="90%" /> |  <img src="https://github.com/user-attachments/assets/b8d704a5-4bf8-4e6a-acd5-54b882e8ccc6" width="90%" /> |

| Back  | Top  | Bottom |
| :--: | :--: |:--: |  
| <img src="https://github.com/user-attachments/assets/1b9fdaee-b758-47dd-b572-509e3d5af35e" width="90%" /> | <img src="https://github.com/user-attachments/assets/19fba1bd-56ff-441b-8411-ddadb8295040" width="90%" /> |  <img src="https://github.com/user-attachments/assets/e8d83c7d-673f-43ad-bcc0-ec78e74bc9e3" width="90%" /> | 
> [!NOTE]
> 📸 Visit our [`v-photos`](https://github.com/vania020/wro2025-robotek/tree/main/v-photos) folder for more detailed photos of the car



### **<ins>Versions of the Car</ins>**
Our vehicle has gone through **10 versions** (and since we are always improving, there are still more to come). It all started with a cardboard prototype, then moved on to acrylic and metal chassis designs, and later, we made personalized adjustments to a HiWonder kit. Along the way, we also experimented with different wheel designs, repositioned components, tested sensors like LiDAR, and finally adopted a new operating environment with ROS and Ubuntu.

<table>  
  <tr>
    <th width="10%">Version</th>
    <th width="40%">Car Photo</th>
    <th width="50%">Description</th>
  </tr>
  
  <!-- Version 1 -->
  <tr>
    <td align="center"><i>Version N°1</i></td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/88cb9f71-715d-41d5-8dcd-54870af7fa87" width="330"height="230"/>
    </td>
    <td>
      We built a cardboard prototype to test and understand the Ackermann steering system and wheel movement. This served as a reference to learn about the placement of components and systems.<br><br>
      <a href="v-photos/vehicle-versions/README.md#version-1">➡️ See more photos 🚗</a>
    </td>
  </tr>

  <!-- Version 2 -->
  <tr>
    <td align="center"><i>Version N°2</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/14afa30d-317e-43f8-b6ef-d63897212c88" width="330" height="230"/>
    </td>
    <td>
      <br>We cut and incorporated an acrylic chassis and designed/3D-printed housing pieces for the camera, Ackermann system, and other components. The Ackermann used a stepper motor, and to perceive its surroundings, the car relied on infrared sensors and a webcam. 
      <br><br>The main controllers were a Raspberry Pi 4 and an Arduino Nano, powered by a power bank and lithium batteries. The car’s movement was driven by a single motor.<br><br> 
      <a href="v-photos/vehicle-versions/README.md#version-2">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 3 -->
  <tr>
    <td align="center"><i>Version N°3</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/5299a922-0fb3-43a8-9a32-9bdbc77225d7" width="330" height="230"/>
    </td>
    <td>
      <br> We replaced the power bank with a smaller one, adjusted component placement into a two-level car system, and installed new wheels. Lithium batteries were replaced with higher-current ones, and the infrared sensors were moved to the front, so the vehicle could make more precise turns.<br><br>
      <a href="v-photos/vehicle-versions/README.md#version-3">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 4 -->
  <tr>
    <td align="center"><i>Version N°4</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/80568426-a136-48a6-b219-9c05dd3e2584" width="330" height="230"/>
    </td>
    <td>
    <br> We upgraded the chassis to a modified HiWonder Kit and replaced the infrared sensors with a LiDAR for more accurate obstacle detection. The webcam was also switched to a monocular camera. The original car motor was replaced by two encoder motors, adapted with gears to drive a single wheel in compliance with competition guidelines.<br><br>

For the Ackermann steering, we replaced the stepper motor with a servomotor. On top of that, we moved away from the Arduino Nano and began implementing the ROS framework.<br><br>
<a href="v-photos/vehicle-versions/README.md#version-4">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 5 -->
  <tr>
    <td align="center"><i>Version N°5</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/fe6757a1-3fe5-4b12-b856-bedcc28a5b50" width="330" height="230"/>
    </td>
    <td>
      <br>Our main improvement was the chassis. We joined the two bases by drilling them together and carefully organized the components with the Raspberry Pi inside. The LiDAR was placed on top so nothing would block its view, and we also completed and installed the camera housing.<br><br>
      <a href="v-photos/vehicle-versions/README.md#version-5">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 6 -->
  <tr>
    <td align="center"><i>Version N°6</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/7e483e4c-2f35-48e9-8125-00a2123f06a2" width="330" height="230"/>
    </td>
    <td>
      <br> Wheels were changed to adjust the car’s height so the Lidar could detect walls within the 10 cm range (previously it was too high and failed). The housing material was upgraded from PLA to polycarbonate for greater resistance, and the Open Challenge (autonomous 3 rounds driving) was completed.<br><br>
      <a href="v-photos/vehicle-versions/README.md#version-6">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 7 -->
  <tr>
    <td align="center"><i>Version N°7</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/105155b6-ddb2-4885-aa63-7e07f1468315" width="330" height="230"/>
    </td>
    <td>
      <br> Using our HiWonder kit as the base, we designed and cut a completely new, smaller chassis with personalized mounting holes for all components, new housing pieces were created, and the two-motor system was replaced by a single motor in a gear system.<br><br>
    We also moved from a two-level structure to a single-level layout, placing all the components on the same surface to give the Raspberry Pi better airflow and easier access. The car successfully detected and avoided the first traffic signs.<br><br>
      <a href="v-photos/vehicle-versions/README.md#version-7">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 8 -->
  <tr>
    <td align="center"><i>Version N°8</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/5d66a0cd-a12c-4155-ae6f-aa4655cf6e0e" width="330" height="250"/>
    </td>
    <td>
      <br> <b>Car dimensions:</b> 15 x 23 cm <br><br>
      We 3D-printed new, slimmer front wheels because the original ones stuck out too much from the chassis. A custom housing was also printed for the batteries, and most importantly, the Ackermann steering servo was mounted vertically to save space and allow for a wider turning angle.<br><br>
During previous testing, we realized the LiDAR was struggling to properly detect the walls of the field, so we 3D-printed and implemented a small angled mount to give it a slight tilt.<br><br>
      <a href="v-photos/vehicle-versions/README.md#version-8">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

<!-- Version 9 -->
  <tr>
    <td align="center"><i>Version N°9</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/312c2e7e-4638-4917-b54a-fff2bb1a2965" width="330" height="450"/>
    </td>
    <td>
      <br> <b>Car dimensions:</b> 15 x 20 cm <br><br>
In this version, the components were arranged more efficiently to save space. We added a custom housing for the batteries, placed the Pi5 controller on top, and mounted the Raspberry above it, creating a layered system.<br><br> 
A new chassis base was printed in MDF, and the Ackermann was moved slightly because, in the previous version, the rack was colliding with the servo. To improve traction, we added a groove to the wheels and printed small cylinders between them to prevent contact with the screws. With these adjustments, the robot managed to complete a lap in <b>20 seconds</b>.<br><br>
      <a href="v-photos/vehicle-versions/README.md#version-9">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 10 -->
  <tr>
    <td align="center"><i>Version N°10</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/03a1e68f-fcf6-40c0-91ea-beff7a2833ab" width="330" height="450"/>
    </td>
    <td>
      <br> <b>Car dimensions:</b> 15 x 18 x 16 cm <br><br> 
We removed the LiDAR housing because, after testing the robot multiple times, we found that vision worked better without it. We also printed the rear wheels, so now all wheels are the same. We modified the gear system, which allowed the robot to complete the 3 laps faster. It now takes less than 10 seconds to complete an entire lap, and it manages to complete the 3 laps in 28 seconds. The small and big gears were interchanged so that the big gear is directly attached to the motor shaft
<br><br>
  <a href="v-photos/vehicle-versions/README.md#version-10">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>

  <!-- Version 11 -->
  <tr>
    <td align="center"><i>Version N°11</i></td>
    <td align="center"><img src="https://github.com/user-attachments/assets/deba8088-32d4-4846-b635-17d4c11e2836" width="330" height="450"/>
    </td>
    <td>
      <br> <b>Car dimensions:</b> N x N x N cm <br><br> 
We noticed that the room lights were affecting our color detection, so we added a 16-LED WS2812 RGB ring light (5V) to keep the lighting consistent. We also moved the LiDAR again. We printed a new housing and placed it at the front, right under the chassis, an adjustment that significantly improved wall detection.
Finally, we replaced the old L298N motor driver with a TB6612FNG. This made a big difference because we can now control the speed more smoothly, starting at around 35%, and we have much better control when reversing and stopping.
<br><br>
  <a href="v-photos/vehicle-versions/README.md#version-10">➡️ See more photos 🚗</a><br><br>
    </td>
  </tr>
  
</table>


<br>

> [!NOTE]
> 🌱 Visit our [`vehicle-versions`](v-photos/vehicle-versions/README.md) folder to see photos and videos of our car evolution.

<br>

## 5. System Setup
### **<ins>Operating Environment Overview</ins>**
The operating environment of our robotic car is designed as a structure that connects hardware, software, and middleware into a single functional system, shown in the diagram below:

<p align="center">
  <img src="https://github.com/user-attachments/assets/aed2f470-f8ad-4831-a153-82bd271397bb" width="90%">
</p>


### **<ins>Robot Operating System (ROS)</ins>** <img width="50" alt="ROS" src="https://github.com/user-attachments/assets/53574d65-315e-4dfd-a8d9-ffb38e892bab" />
ROS is a framework that connects a robot’s software and hardware, making sensors, motors, and programs work together so the robot can perform tasks smoothly. For this project, we use ROS 2 Jazzy, one of the latest versions of ROS 2. You can find the documentation here:  [ROS 2 Documentation: Jazzy](https://docs.ros.org/en/jazzy/index.html)<br>

Without ROS, everything would have to be written in one long, complicated program that’s hard to manage. We can now divide the system into smaller parts, or nodes, that each do one job, making the system easier to build, fix, and expand.<br>

<p align="center">
  <img src="https://github.com/user-attachments/assets/b94b35c9-c47d-48f0-9a54-57789c4cc455" alt="Nodes" width="80%">
</p>

> This is an official animation to better understand how nodes work. Each node does one clear job, like moving wheels or reading sensors, and they talk to each other using topics, services, actions, or parameters.
>  In practice, this is the process:
> 1. Sensors send data to ROS 2 by publishing it on topics.
> 2. Other nodes listen to that data, process it, and decide what the car should do.
> 3. Finally, a controller node sends commands, making the car move accordingly.

<br>

*<ins>Why we use ROS?</ins>*<br>

<table>
  <tr>
    <td><b>Sensor & Actuator Integration</b></td>
    <td>
      ROS connects all sensors and actuators in one system, ensuring seamless coordination. 
      This gives our autonomous car continuous information about its surroundings 
      and optimizes overall performance.
    </td>
  </tr>
  <tr>
    <td><b>Environmental Perception with LiDAR</b></td>
    <td>
      Most manufacturers of advanced sensors, such as LiDARs, provide an official package to use their hardware with ROS. In the case of the DTOF STL-19P, the manufacturer provides a package that automatically publishes the LiDAR data so it can be processed afterward.
    </td>
  </tr>
  <tr>
    <td><b>Support with Python</b></td>
    <td>
      ROS is fully compatible with Python, which allows for versatile and high-level code development. In addition, being open-source, it has a large community that provides support and assistance for robot development.
    </td>
  </tr>
  <tr>
    <td><b>Debugging & Simulation</b></td>
    <td>
      ROS includes tools to quickly debug the content published on topics. It also provides tools such as RViz, which allows real-time data visualization, and Gazebo, which enables running simulations.
    </td>
  </tr>
  <tr>
    <td><b>Efficient Simulation & Debugging</b></td>
    <td>
      Virtual environments like 
      <a href="https://gazebosim.org/home">Gazebo</a> let us fine-tune parameters before implementation. 
      Tools like 
      <a href="https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html">RViz</a> 
      allow real-time visualization of sensor data and car status, making debugging much easier.
    </td>
  </tr>
</table>



<br>

### **<ins>Ubuntu</ins>** <img width="60" alt="UBUNTU" src="https://github.com/user-attachments/assets/38f176af-2d07-41f9-bb14-3ef8eb2c0022" />
Ubuntu is a popular, free, open-source operating system based on Linux. It is known for being reliable, flexible, and widely used in both research and industry. In our project, we use Ubuntu 24.04 as the **foundation that runs on the Raspberry Pi**.<br>

Ubuntu manages the Raspberry Pi’s resources efficiently, ensuring it runs correctly. It provides drivers and compatibility for sensors and external hardware, making integration easier. Ubuntu also allows us to install and run ROS 2 Jazzy and gives us access to important libraries and tools that simplify tasks such as sensor communication and system development.<br>

We use Ubuntu because of its stability and compatibility with ROS 2 Jazzy. Since ROS 2 packages are officially distributed for Ubuntu, using this operating system guarantees that we can easily install and manage the software needed for our car.
<br><br>

### **<ins>Raspberry Pi</ins>** <img width="80" alt="Raspberry" src="https://github.com/user-attachments/assets/6e218c1a-8fe1-47a2-8ae5-1719956508fa" />
The Raspberry Pi is a small computer that works as the brain of our car. It is powerful enough to run Ubuntu, ROS 2, and our algorithms in real time. In our project, we use the Raspberry Pi 5, which you can find here: [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)<br>


**👉 Set up and configuration**<br>

With the [Raspberry Pi Imager](https://www.raspberrypi.com/software/), we flash Ubuntu 24.04 into the microSD card of the Raspberry. After that, we configure the basics like Wi-Fi, SSH for remote access, and hostname. And finally, we install ROS 2 Jazzy by following these steps: [Installation Ubuntu (deb packages)](https://www.google.com/url?q=https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html&sa=D&source=docs&ust=1756316032253380&usg=AOvVaw24eBtKhOAoYRVzp4xh2Rkh)

| Step 1 | Step 2 | Step 3 | Step 4 |
| :--: | :--: | :--: |  :--: |
| <img src="https://github.com/user-attachments/assets/f6ee9449-517c-443a-aad2-c7a4913f8334"/> | <img src="https://github.com/user-attachments/assets/0990b9f5-a0a7-408b-a90b-2e9086ae6032" /> | <img src="https://github.com/user-attachments/assets/8b716aad-e25b-4ff4-a92a-8e5e1b085319" /> | <img src="https://github.com/user-attachments/assets/c0f70529-b1b9-4ce7-b0c2-3803a02c44db"/> |


<br>

## 6. Mobility Management  

### <ins>**Steering System – Ackermann**</ins>

<p align="center">
  <img src="https://github.com/user-attachments/assets/8cfcffdb-48aa-4297-a41c-b494a0f222c0" alt="Ackermann" width="70%">
</p>

Our autonomous car uses an Ackermann steering system, controlled by a **15 kg·cm digital servo**, which provides precise and stable control for navigation and turns.  

The Ackermann steering geometry is designed to reduce tire slip by ensuring that all wheels align as radii of circles that share a common center when the car is turning. This configuration keeps the rear wheels fixed and places the center of rotation along a line extended from the rear axle. To achieve this geometry, the inside front wheel turns at a greater angle than the outside front wheel, allowing smoother and more efficient cornering.

---

### **Our own modifications ⚒️**

In the first versions of the car, we implemented the Ackermann steering system using a custom mechanism. We designed and 3D printed a gear connected to a stepper motor, along with a rack, which is a stick with grooves that fit into the gear teeth. When the motor rotated the gear, the rack would move, which in turn rotated the wheels.  

<p align="center">
  <img src="https://github.com/user-attachments/assets/211ba653-b04c-4f56-9fca-0c7498eb9aff" width="300" height="200">
  <img src="https://github.com/user-attachments/assets/e45a7e54-3c88-4524-b427-0ecff04898f5" width="300" height="200">
</p>

One of the challenges we faced was the 3D printing process itself. Printing small details such as gear teeth was difficult and often imprecise, which caused problems in the initial prototypes. To solve this, we made the gear teeth larger, and while this worked mechanically, the final design ended up taking too much space inside the chassis.

For this reason, we decided to switch to the system we currently use, which is part of the **HiWonder kit**, adapted to fit in our chassis base. Instead of gears, it uses a system of linkages connected by screws and supported by bearings. These linkages move and transfer the motion to the wheels, achieving the Ackermann steering effect in a more compact way.

<p align="center">
  <img src="https://github.com/user-attachments/assets/4843c1d0-fb98-4bd4-b9e3-5fe4925295cd" width="300" height="200">
</p>

> [!WARNING]
> At first, the Ackermann of the HiWonder kit worked fine — the car could turn and even get through some obstacles. But after a lot of testing, we noticed that whenever the car turned left, the Ackermann didn’t rotate as much as it did when turning right. This made obstacle avoidance harder, especially in the field corners, so we knew we had to make adjustments.

We realized that the servo needed to be repositioned. First, it was placed horizontally under the Ackermann linkages. We brainstormed how to make it more efficient and changed the position of the servomotor, mounting it vertically with the accessory facing downward under the chassis. This way, the linkages could move freely with more angles, and we also freed up extra space for the other components. This new placement also allowed us to reduce the length of the chassis.  

<table>  
  <tr>
    <th width="30%">Initial position</th>
    <th width="30%">Idea</th>
    <th width="30%">Final Position</th>
  </tr>
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/8f9ed661-b90d-4a0c-a93c-d991b6ed5e6a"/>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/f8daf76d-dcd0-4047-aad0-b892d945f7b1"/>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/a90c9a7e-319a-4859-b601-9cafb577e2aa"/>
    </td>
  </tr>
  <tr>
    <td align="center">Servo mounted horizontally (limited angles)</td>
    <td align="center">Servo repositioned vertically</td>
    <td align="center">Final placement with improved steering</td>
  </tr>
</table>

---

### <ins>**Motor and Drivetrain**</ins>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f636176b-af6a-4b72-bb31-53f025ab41b1" width="70%">
</p>

The drivetrain of our autonomous car is powered by a **25 mm metal gear DC motor**, chosen for its compact size and high torque. The motor is mounted on the chassis and directly connected to the rear axle through a system of gears, ensuring efficient transfer of power to the wheels.  

Our drivetrain includes a gear system:  
- The large gear is mounted on the motor shaft.  
- This large gear drives a smaller gear connected to the rear axle.  
- The result is an increase in the speed of the wheels.

<p align="center">
  <img src="https://github.com/user-attachments/assets/1b89468f-1234-4610-8944-20d835b95d5b" width="50%">
</p>

During assembly, we noticed a gap between the metal chassis part and the axle supports. This caused instability in the drivetrain. To fix it, we designed custom 3D-printed cylindrical spacers that fill the gap and keep the axle firmly in place. This simple solution reduces vibrations, prevents misalignment, and ensures smoother transmission of power from the motor to the wheels.

<p align="center">
  <img src="https://github.com/user-attachments/assets/6c35f5b0-4c04-4e40-a79d-c3ad8ee8d3b9" width="50%">
</p>

#### **Motor Driver Upgrade & Mobility Improvements**
For this year's vehicle, we implemented a critical upgrade to our motor driver to resolve severe mobility issues. In our previous version, when the car's speed was reduced to 50% or lower, it would stop completely. The motor lacked the necessary force to overcome ground friction, a problem worsened by the overall weight of the car. 

By changing the driver, we have achieved a significant improvement in mobility management:
- **Low-Speed Efficiency:** The car now operates smoothly and maintains torque from as low as 35% speed without stalling.
- **Multi-directional Movement:** The upgrade allows the vehicle to move in different directions effortlessly, finally giving us the ability to reverse reliably.
- **Precision Braking:** We can now brake and stop the vehicle with much higher accuracy.

#### **Speed Configuration**
Through rigorous testing, we established specific speed configurations to optimize our sensor's performance. For the **Open Challenge**, we decided to cap the maximum speed at **70%** instead of running at 100%. This controlled speed ensures that the LiDAR sensor has enough processing time to accurately detect walls, preventing crashes and maintaining a reliable trajectory throughout the laps.


---

### <ins>**3D Pieces**</ins>

This are the 3D model parts of our vehicle.  
Below, you will find a table with our 3D-printed parts and their descriptions.

<table align="center" width="100%" style="table-layout: fixed;">
  <thead>
    <tr>
      <th width="30%">Component</th>
      <th width="40%">Preview</th>
      <th width="30%">Folder</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center"><b>Vehicle Base</b></td>
      <td align="center">
        <img src="models/vehicle_base/vehicle_base.png" width="90%">
      </td>
      <td align="center">
        <a href="models/vehicle_base/" target="_blank">View</a>
      </td>
    </tr>
    <tr>
      <td align="center"><b>Vehicle Wheels</b></td>
      <td align="center">
        <img src="models/vehicle_wheels/vehicle_wheels.png" width="90%">
      </td>
      <td align="center">
        <a href="models/vehicle_wheels/" target="_blank">View</a>
      </td>
    </tr>
    <tr>
      <td align="center"><b>Camera Housing</b></td>
      <td align="center">
        <img src="models/camara_housing/camara_housing.png" width="90%">
      </td>
      <td align="center">
        <a href="models/camara_housing/" target="_blank">View</a>
      </td>
    </tr>
       <td align="center"><b>Raspberry Housing</b></td>
      <td align="center">
        <img src="models/raspberry_housing/raspberry_housing.png" width="90%">
      </td>
      <td align="center">
        <a href="models/raspberry_housing/" target="_blank">View</a>
      </td>
    </tr>
  <tr>
      <td align="center"><b>LiDar Housing</b></td>
      <td align="center">
        <img src="models/lidar_housing/lidar_housing.png" width="90%">
      </td>
      <td align="center">
        <a href="models/lidar_housing/" target="_blank">View</a>
      </td>
    </tr>
  </tbody>
</table>

## 7. Power & Sense Management 

### <ins>**Power Source**</ins>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f8bf1958-dde8-4bd7-8e4c-905e09efa7b0" width="80%">
</p>


Our autonomous car is powered by 7.4V Li-Po batteries (2000mAh, 20C). We chose Li-Po batteries because they provide a high energy density, meaning more power in a small and lightweight package. The battery provides two main energy lines: one for the electronics and another for the motor: <br>

1. **Electronics & Sensors**  
   The battery first connects to the **RRC Lite Controller**, which regulates the power down to a safe **5 V**.  
   From there, it distributes electricity to:  
   - **Raspberry Pi 5** → acts as the brain of the car, running ROS 2 and processing sensor data.  
   - **Digital servomotor** → controls the Ackermann steering system.  
   - **STL-19P TOF LiDAR & monocular camera** → provide vision and distance perception.  

2. **Drive Motor (via L298N Motor Driver)**  
   In parallel, the battery also powers the **L298N motor driver** directly with **7.4 V**.  
   This driver regulates how much current goes to the **25 mm metal DC motor**, which is responsible for moving the car forward.  

By splitting the power into two paths (one regulated for sensitive electronics and one direct for the motor), the system ensures stability. Motors usually demand sudden spikes of current, and separating their supply avoids crashes or interruptions in the Raspberry Pi and sensors.


### <ins>**Sensors Integration**</ins>
To perceive its environment and handle the Future Engineers challenges, the car uses a combination of:  

- **STL-19P TOF LiDAR**: provides 360° distance data.  
- **2DOF Monocular Camera**: detects colors and obstacles.
- **IMU (Built into the RRC Controller)**: tracks orientation, angular velocity, and acceleration.

Together, these sensors give the robot a better understanding of its environment by combining depth perception with visual input.


### <ins>**STL-19P TOF LiDAR**</ins>

<p align="center">
  <img src="https://github.com/user-attachments/assets/853f8730-a645-47b5-bacf-ded61b21a6c9" width="80%">
</p>

<br>

Unlike simpler sensors such as ultrasonic or infrared, which measure only in a single direction and have limited precision, the LiDAR sensor is capable of a **360° scanning** and provides precise distance measurements by using laser pulses. It helps the robot map its surroundings over a wide range, detect obstacles, and navigate more accurately in dynamic environments.

<br>

| Characteristic        | Vaue                        |
|------------------------|------------------------------|
| Ranging distance       | 0.03 – 12 m                 |
| Size                   | 38.59 x 38.59 x 34.8 mm     |
| Scanning Angle         | 360°                        |
| Scanning Frequency     | 5 – 13 Hz                   |
| Ranging Accuracy       | ±45 mm                      |
| Ranging Frequency      | 5000 Hz                     |

> [!IMPORTANT]
> **📍Placement:**
> For this version, we decided to lower the LiDAR In our previous design, a higher placement caused the sensor to occasionally look over the walls, missing the track boundaries or detecting noise outside the field. By lowering it, the LiDAR now aligns perfectly with the 10 cm high walls of the track. This guarantees a clear, reliable view of the limits and obstacles. All other components on the car were carefully arranged below this level to avoid blocking the LiDAR’s line of sight. <br><br>
  
### <ins>**2DOF Monocular Camera**</ins>

<p align = "center">
  <img src = "https://github.com/user-attachments/assets/546b6072-3ab9-4a02-b0a2-437478ac0b03" width="50%">
  </p>
<br>

The 2DOF Monocular Camera complements the LiDAR by adding visual perception. This allows the robot to recognize its environment beyond distance data, enabling future applications such as detecting the obstacles’ colors:

 + **Color detection**: Since graphical color detection is a core part of our  project, the camera can precisely identify various colors on the course. This allows our car to interact with the different traffic signs in the Obstacle challenge.
  + **Spatial awareness through data fusion**: When combining camera data with our sensor, the robot gains a richer understanding of its environment. The camera provides detailed visual context that leads to an adaptable navigation strategy.

> [!IMPORTANT]
> **📍Placement:**
> The camera is located in a special 3D mounting piece with a certain  inclination angle that points to the floor so it can better detect obstacles. At first it was placed at the front of the car but this location provided a limited vision and sticked out of the chassis lenght. See more detailed info of the housing piece in the [`3D Printed Parts`](#3d-printed-parts) section. <br><br> 
> <img width="1206"  src="https://github.com/user-attachments/assets/a286367c-681d-473b-807f-c9e470b2fb0d">

### **<ins>BOM (Bill of Materials)</ins>**
| Component | Quantity | Description | Image |
|-----------|----------|-------------|-------|
| Raspberry Pi 5 | 1 | Main processing unit for running **ROS**. Acts as the main brain and the Host Controller of the system, capable of running operating systems and handling complex processing tasks. | <img width="250" alt="Raspberry Pi 5" src="https://github.com/user-attachments/assets/657762ac-c99f-4375-9b41-e28a6fd0f865" /> |
| RRC Lite Controller | 1 | Integrates: ROS expansion board, High-Frequency PID Control, Motor Closed-Loop Control, Servo Control and Feedback, IMU Data Acquisition, Power Status Monitoring, and a Power Switch. | <img width="250" alt="Controller" src="https://github.com/user-attachments/assets/1703132a-77bc-4e78-95cc-8da6df285f75" /> |
| STL-19P TOF Lidar | 1 | Provides precise, 360-degree distance measurements for real-time navigation and obstacle detection in dynamic environments. | <img width="250" alt="Lidar" src="https://github.com/user-attachments/assets/a8cece7d-a10f-465d-8982-c96919f8e1bd" /> |
| Lidar Adapter Board | 1 | Converts the LiDAR’s UART signals to USB for PC connection and data reading. | <img width="250" alt="Adapter" src="https://github.com/user-attachments/assets/c6bdca97-f5a3-4a1e-b0ee-6b4bda3a2ee1" /> |
| 15 kg.cm Digital Servo | 1 | Provides accurate steering control. | <img width="250" alt="Servo" src="https://github.com/user-attachments/assets/14a9ca9e-b206-44cb-9bdd-cd3c675e7abc" /> |
| 25MM Metal Gear Motor | 1 | Core drive motor for powering the wheels with torque and speed. | <img width="250" alt="Motor" src="https://github.com/user-attachments/assets/e6695af1-8bcd-48a0-8b6f-802e3145f1be" /> |
| Suitcase Wheel Protector | 4 | Ensures 3d-printed wheels stability and grip on the floor. | <img width="250" alt="tire" src="https://github.com/user-attachments/assets/cbca9bf3-6264-49e9-9e0d-0eec7d5ae6ef" /> |
| Monocular Camera | 1 | A camera used for capturing images and videos, can be used for computer vision or live streaming. | <img width="250" alt="Camera" src="https://github.com/user-attachments/assets/b280e148-f627-4687-9174-b62485a19662" /> |
| TB6612FNGm Motor Driver | 1 | Controls motor direction and speed from the Raspberry Pi. | <img width="250" alt="Motor Driver" src="https://github.com/user-attachments/assets/67e06fc8-39aa-4951-a008-c2fbcb4ed46c" /> |
| Jumper Cables | 4–8 | Electrical connections between the motor driver and Pi. | <img width="250" alt="Jumper Cables" src="https://github.com/user-attachments/assets/e1c2d347-97a2-4d19-bac0-bf0af226dfde" /> |
| USB-USB Cable | 1 | A cable to connect the Raspberry Pi to the camera, Lidar, and Controller. | <img width="250" alt="USB Cable" src="https://github.com/user-attachments/assets/8f95630d-fcd1-4340-b176-5e16c1988f18" /> |
| Li-Po Battery 7.4 V 5000mAh 20C | 1 | A lithium polymer battery that provides portable, high-density power. | <img width="250" alt="Battery" src="https://github.com/user-attachments/assets/452710d7-b1f9-4be5-8f41-9b5a35c3246e" /> |
| 16-LED WS2812 RGB pixel ring light | 1 | A circular light module which make much easier to clearly see and distinguish the colors of the blocks. | <img width="250" alt="Battery" src="https://github.com/user-attachments/assets/3c9fbbec-c46e-43ac-8d64-a5ea1f1e1d52" /> |


### <ins>**Wiring Diagram**<ins>

The following visual wiring diagram illustrates the **physical connections** between the modules (Raspberry Pi, LiDAR, camera, motor, etc.) using **realistic component images**. It helps visualize the system layout and understand **how components are arranged and linked** in the robot.  

<p align="center">
  <img src="https://github.com/user-attachments/assets/743de701-c5f6-4f64-a87a-c588e286aba9" width="80%">
</p>


#### **1. Raspberry Pi 5 Connections**
The Raspberry Pi 5 works as the brain of the car. Its USB ports connect the main sensors and the controller:

+ Camera → provides visual input for detecting traffic signs and colored obstacles.
+ LiDAR → provides distance and wall detection for navigation.
+ RRC Lite Controller → interfaces with the digital servomotor and forwards control signals to the motor driver.

<p align="center">
  <img src="https://github.com/user-attachments/assets/832ce7dc-3c0e-4a98-9d64-ad4811c52f5d" width="80%">
</p>

#### **2. RRC Lite Controller**
The RRC Lite Controller acts as a bridge between the Raspberry Pi and the actuators. It regulates incoming power from the Li-Po battery (down to a safe 5 V) and distributes it to:  
- The **Raspberry Pi 5**  
- The **digital servomotor** for Ackermann steering  

It also handles communication with the motor driver to send PWM control signals.

---

#### **3. Motor Driver and DC Motor**
The **L298N Motor Driver** is powered directly by the **7.4 V Li-Po battery**.  
- It regulates both the **speed** and **direction** of the **25 mm metal gear DC motor**.  
- The motor then powers the rear axle through a gear reduction system, increasing torque for smoother acceleration.  

---

#### **4. Power Source**
The **7.4 V Li-Po battery** supplies energy to the entire system, split into two paths:  
- **Direct line (7.4 V):** powers the L298N motor driver and DC motor.  
- **Regulated line (via RRC Lite Controller):** delivers 5 V to the Raspberry Pi, servomotor, and sensors.  

<br>

This is also a diagram that shows the **electrical wiring** and pin-level connections between all components — including power lines, GPIOs, and communication ports. It focuses on **signal paths and voltage levels** rather than physical appearance.  

<p align="center">
  <img src="https://github.com/user-attachments/assets/49bfa6f3-0aa3-49bb-afd3-9eb036410a11" width="80%">
</p>

> [!NOTE]
> Visit our [`schemes`](https://github.com/vania020/wro2025-robotek/tree/main/schemes) folder to access all of our diagrams and schemes 🔑🚗


## 6. Obstacle Management 

### <ins>**Control Node Structure**</ins>

Since we use ROS 2 as the middleware that connects all the components of our autonomous vehicle, there isn't a single code that runs either the Open Challenge or the Obstacle Challenge. Instead, the system is built as a collection of independent but interconnected ROS 2 nodes, each performing a specific function such as reading the camera, processing LiDAR data, or controlling the motors. These nodes communicate constantly through topics and messages, allowing the car to behave as a cohesive, intelligent system.

This table better explains our ROS topics and messages:

| **Node**                                | **Role**           | **Topic**                      | **Message Type**                                                   | **What It Transmits**                                                                                                                       |
| --------------------------------------- | ------------------ | ------------------------------ | ------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------- |
| **LiDAR Node**                          | Publisher          | `/scan`                        | `sensor_msgs/LaserScan`                                            | It sends 360° distance readings from the LiDAR; each value represents how far an obstacle is at a given angle.                                 |
| **Camera Node**                         | Publisher          | `/obstaculos`                  | `std_msgs/String`                                                  | It publishes the detected obstacle color in a text form: (`rojo`, `verde`, or `ninguno`), which defines how the main controller adjusts the steering setpoint. |
| **AckerLidar Node** *(Main Controller)* | Central Controller | `/motor_vel`, `/positionServo` | `std_msgs/Float32`, `ros_robot_controller_msgs/SetAckerServoState` | `/motor_vel`: Motor duty cycle (speed %). `/positionServo`: Steering position (PWM in µs) for Ackermann control.                            |
| **Motor Node**                          | Subscriber         | `/motor_vel`                   | `std_msgs/Float32`                                                 | Receives motor velocity commands from the main controller and translates them into PWM signals for the DC motor driver (L298N).             |
| **Raspberry Pi 5 Controller Node**      | Subscriber         | `/positionServo`               | `ros_robot_controller_msgs/SetAckerServoState`                     | Executes servo position commands and moves the front wheels to reach the target angle.                                                      |
| **RRC Controller Button**               | Publisher          | `/button`                      | `ros_robot_controller_msgs/ButtonState`                            | Sends an activation signal when the onboard button is pressed, starting the robot’s control loop.                                           |

### **What is a Message type?**

A **message type** is like a template that defines **what kind of data** is sent through a *topic*. For example, ROS already includes many built-in types, such as:

* `std_msgs/String` → used to send text.
* `std_msgs/Float32` → used to send a decimal number.
* `sensor_msgs/LaserScan` → used to send LiDAR data (distance readings around 360°).
* `geometry_msgs/Twist` → used to send linear and angular velocities (very common in mobile robots).
* And you can also have **custom message types**, like the ones included in your own package `ros_robot_controller_msgs`.

### **Important clarifiactions**

* For simplicity, **some topic names were shortened** in the diagrams.

  * The real ROS topic `/ros_robot_controller/acker_servo/set_state` is represented as **`/positionServo`**.
  * The real ROS topic `/ros_robot_controller/button` is represented as **`/button`**.

* The button input `/ros_robot_controller/button` is processed internally by the main node and does not appear as a separate node because it doesn’t exchange messages with others.

* The naming convention was simplified only for documentation clarity; all logic and connections in the code remain unchanged.

* The **AckerLidar Node** acts as the core controller that links everything:
  it subscribes to LiDAR and camera data, processes the PID control, and publishes both the steering and velocity commands.



> [!NOTE]
> If you would like to see a detailed description and explanation of the code behind each node, please visit our [`src`](https://github.com/vania020/wro2025-robotek/tree/main/src) folder 🖥️🚗
> 

---


### <ins>**Open Challenge**</ins>

The **Open Challenge** is the first autonomous driving test. In this stage, the robot must complete three laps **without any external input** relying only on its onboard **LiDAR sensor**, **PID controller**, and **Ackermann steering system**. The goal is to keep the car centered between the inner and external walls of the challenge during the entire lap. To achieve that, the robot constantly measures two key distances: **D₁** → Distance from the car to the **left wall** and **D₂** → Distance from the car to the **right wall**

The **control goal** is defined by the equation:

> **D₁ - D₂ = 0**  
> *(Setpoint = 0 → car is centered)*

When this balance holds true, it means that both walls are equidistant, and the car is aligned in the center of the track.

<p align="center">
  <img src="https://github.com/user-attachments/assets/32a3a9b0-3693-403a-a52e-a4078647d5d0" width="80%">
</p>

### **How It Works: LiDAR + Control Loop**

The LiDAR sensor scans the surrounding environment in real time, detecting obstacles and measuring the distances around the car. From these scans, two zones are analyzed, one on the **left** and one on the **right** to extract D₁ and D₂.

These values are sent to the **AckerLidarNode**, the main control node in charge of:
- Reading and processing LiDAR data  
- Computing the distance difference `error = D₁ - D₂`  
- Sending control signals to the **steering** and **motor** nodes

The logic is the following:  
1. If **D₁ > D₂**, the car is too close to the right wall → it turns slightly **left**.  
2. If **D₁ < D₂**, the car is too close to the left wall → it turns slightly **right**.  
3. If **D₁ = D₂**, the car is centered → it continues straight.

This process runs in a **PID feedback loop**, where:
- The **P (Proportional)** term corrects small deviations quickly.  
- The **I (Integral)** term reduces long-term bias (not always needed).  
- The **D (Derivative)** term prevents oscillations and overshoot.  

The result is a **smooth, stable trajectory** that keeps the car aligned throughout the race.

<div align="center">
  <img src="https://github.com/user-attachments/assets/b83a0a21-d830-4516-a9d1-8e52b26a63bd" width="80%">
</div>


### **Open Challenge Flowchart**

<p align="center"> <img src="https://github.com/user-attachments/assets/f6272aa2-9634-472e-8685-449e77ced2c1" width="80%"> </p> 


#### Step-by-Step Description

1. **Start Robot & Initialization**  
   ROS2 nodes are launched: the LiDAR begins scanning, and the control node initializes all parameters (PID gains, setpoint, motor topics).

2. **Continuous Loop**  
   The system enters a continuous loop (`while rclpy.ok()`), running dozens of times per second. Each cycle updates sensor readings and steering actions.

3. **LiDAR Scan Environment**  
   The sensor performs a 360° scan to detect the walls and extract points on both sides of the track.

4. **Extract Wall Distances (D₁, D₂)**  
   The algorithm filters the LiDAR data to isolate the left and right regions, computes the average distance for each, and updates D₁ and D₂.

5. **Compute Error (D₁ - D₂)**  
   The difference between these distances represents how “off-center” the car is from the ideal middle of the lane.

6. **PID Steering Adjustment**  
   The PID controller processes this error and outputs an angle correction, which is sent to the **servo motor** using an Ackermann steering model.

7. **Update Lap Counter**  
   The control node counts laps based on internal flags or distance traveled (depending on the implementation in the ROS2 package).

8. **3 Laps Completed → Stop Vehicle**  
   After completing three full laps, the system safely reduces speed and stops the motor node.

### **Nodes and Communication**

During the Open Challenge, three ROS2 nodes work together in real time:

| Node | Role | Description |
|------|------|-------------|
| **`AckerLidarNode`** |Main Control Node | Subscribes to LiDAR data, calculates the distance difference, runs the PID controller, and sends steering/motor commands. |
| **`MotorPWMNode`** | Motor Control | Receives speed values (`Float32`) and controls the DC motor through PWM signals, ensuring smooth acceleration. |
| **`SetAckerServoState`** | Steering Control | Adjusts the steering servo angle according to PID output, maintaining Ackermann kinematics. |


---


### <ins>**Obstacle Challenge**</ins>

<p align="center">
  <img src="https://github.com/user-attachments/assets/d2ae5d4d-4fee-49d1-a17e-dc7cf31d3865" width="100%">
</p>

### **Obstacle Challenge Flowchart**

<p align="center">
  <img src="https://github.com/user-attachments/assets/1fe3b8fa-3e9d-4a10-896f-0c3e835c6a39" width="80%">
</p>


<br><br>
## 7. Assembly Instructions 

### **Ackermann System**
+ To build the Ackermann system on the car, first screw the fixed parts that attach the system to the base.
+ Next, attach the two movable pieces that steer the wheels to the sides of the fixed part. Make sure these pieces can still rotate freely. Each of these pieces has two holes use the hole that is closer to the wheels.
+ Then, connect the wheel pieces with a linkage that has bearings on both ends. The bearings should be screwed into the other holes of the wheel pieces, forming a movable, half-rectangular structure.
+ Attach a second linkage, identical to the first but shorter, to one side of the system. This linkage connects to one of the holes on the wheel pieces, the same ones linked to the bearings of the first linkage. This shorter linkage will move the steering system and must be connected directly to the servo motor, which should be mounted vertically on the base.
+ Important: The servo’s output shaft and the side where the second linkage is attached should be on opposite sides.
Finally, attach the wheel hubs, the parts that hold the actual wheels, to the movable pieces via bearings. These bearings should allow the wheels to rotate freely.
 
### **Motor System**
+ Screw the motor mount, which is the piece that holds both the motor and its axle, onto the base. Then, screw the motor to this mount.
+ Insert the axle into the mount. The mount has two circular openings that the axle passes through. The axle should be supported by two bearings, which keep it centered, and two locking rings, which prevent side movement.
+ Once the axle is in place, connect the motor shaft and the axle using spacers, ensuring they do not collide with the motor screws.
 
### **Housings**
After both systems are assembled, install the housings on the top side of the base. Mount the camera, LiDAR, batteries, and Raspberry Pi according to the model layout:
+ The LiDAR housing goes at the front, ensuring its field of view is unobstructed.
+ The camera housing goes right behind the LiDAR housing, which it’s taller, so its view remains clear.
+ The battery and Raspberry Pi housings go behind them, forming a sort of ceiling above the servo motor that protrudes from the base.
 
### **Electronics**
Finally, install all electronic components, attaching them to their designated housings and connecting them according to the wiring diagram.


<br><br>
## 8. Performance Videos 

<div align="center">

| Challenge | YouTube Video |
|----------|----------------|
| <img width="500" alt="Renkay 2" src="" /> | <a href="https://youtu.be/lE8SPT6_tL0?si=ECjmNy6-T6QCe0LI">Watch on YouTube 🎥</a> |
| <img width="500" alt="RenkayE 1" src="" /> | <a href="https://youtu.be/DUVe36ZpR18?si=IGmmhTL5FZdeEi8m">Watch on YouTube 🎥</a> |

</div>

