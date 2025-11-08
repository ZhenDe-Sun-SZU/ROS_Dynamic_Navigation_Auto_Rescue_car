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

![Product Screenshot](docs/demo.png)

This project implements an autonomous rescue car capable of:
- Dynamic path planning in unknown environments
- Real-time body detection using YOLOv8
- Autonomous navigation and obstacle avoidance
- Multi-sensor fusion (LiDAR, Camera, IMU)

### Features

- ✅ Real-time body detection and classification
- ✅ SLAM-based environment mapping
- ✅ Dynamic obstacle avoidance
- ✅ Efficient path planning algorithms
- ✅ ROS-based modular architecture

### Built With

* [ROS Noetic](http://wiki.ros.org/melodic)
* [YOLOv8](https://github.com/ultralytics/yolov8)
* [OpenCV](https://opencv.org/)
* [TensorRT](https://developer.nvidia.com/tensorrt)
* [STM32 HAL](https://www.st.com/en/embedded-software/stm32cube-mcu-packages.html)

## Getting Started

### Prerequisites

- ROS Melodic (Ubuntu 20.04)
- Python 3.8+
- STM32CubeIDE
- Jetson Nano with JetPack 4.6

### Installation

1. Clone the repo
```bash
git clone https://github.com/yourusername/ROS_Dynamic_Navigation_Auto_Rescue_car.git
cd ROS_Dynamic_Navigation_Auto_Rescue_car

## 🚀 Usage

<!--                         -->


## 🔧 Hardware Setup

| Component | Model / Specification | Purpose |
| :--- | :--- | :--- |
| **Main Controller** | Jetson Nano | Core Processing & Algorithm Deployment |
| **Microcontroller** | STM32F407 | Motor Control & Sensor Management |
| **LiDAR** | 2D LiDAR | Environment Perception & Mapping |
| **Depth Camera** | Astra Pro Plus | 3D Vision & Obstacle Detection |
| **Motors** | JG52-GF37B | Motion Execution |

**System Characteristics**:
- Total weight < 15kg, Dimensions: 40cm × 30cm × 20cm
- Supports operation in narrow spaces (min. passable diameter 30cm)
- Embedded architecture for real-time response

## 📁 Project Structure

<!--                         -->


## 📊 Results

![Product Screenshot](docs/show.jpg)

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Contact

- **Project Link**: [https://github.com/ZhenDe-Sun-52U/ROS_Dynamic_Navigation_Auto_Rescue_car](https://github.com/ZhenDe-Sun-52U/ROS_Dynamic_Navigation_Auto_Rescue_car)
- **Technical Report**: [Download here](docs/ican_integration_3.docx)
