# ROS Dynamic Navigation Auto Rescue Car

An autonomous rescue vehicle implementing dynamic navigation and body detection using ROS and YOLO.  
**iCAN International Innovation Contest - Provincial Second Prize**

![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen)
![YOLO](https://img.shields.io/badge/YOLOv5-Object_Detection-red)
![Platform](https://img.shields.io/badge/Platform-Jetson_Nano-orange)
![License](https://img.shields.io/badge/License-MIT-blue)

<p align="center">
  <a href="#demo">View Demo</a> •
  <a href="https://github.com/yourusername/your-repo/issues">Report Bug</a> •
  <a href="https://github.com/yourusername/your-repo/issues">Request Feature</a>
</p>

## Table of Contents

- [About The Project](#about-the-project)
  - [Features](#features)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [Hardware Setup](#hardware-setup)
- [Project Structure](#project-structure)
- [Results](#results)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

## About The Project

[![Product Name Screen Shot][product-screenshot]](docs/demo.png)

This project implements an autonomous rescue car capable of:
- Dynamic path planning in unknown environments
- Real-time object detection using YOLOv5
- Autonomous navigation and obstacle avoidance
- Multi-sensor fusion (LiDAR, Camera, IMU)

### Features

- ✅ Real-time object detection and classification
- ✅ SLAM-based environment mapping
- ✅ Dynamic obstacle avoidance
- ✅ Efficient path planning algorithms
- ✅ ROS-based modular architecture

### Built With

* [ROS Noetic](http://wiki.ros.org/melodic)
* [YOLOv5](https://github.com/ultralytics/yolov5)
* [OpenCV](https://opencv.org/)
* [TensorRT](https://developer.nvidia.com/tensorrt)
* [STM32 HAL](https://www.st.com/en/embedded-software/stm32cube-mcu-packages.html)

## Getting Started

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Python 3.8+
- STM32CubeIDE
- Jetson Nano with JetPack 4.6

### Installation

1. Clone the repo
```bash
git clone https://github.com/yourusername/ROS_Dynamic_Navigation_Auto_Rescue_car.git
cd ROS_Dynamic_Navigation_Auto_Rescue_car