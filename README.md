# Autonomous Luggage Follower

## Project Video

Watch the project video here.

## Table of Contents
- Project Overview
- Objective
- Project Video
- Design and Implementation
- Setup and Installation
- Usage
- Testing and Results
- Conclusion
- Future Work
- Contributing
- License
- Acknowledgments

## Project Overview
The Autonomous Luggage Follower project utilizes visual recognition technology to enable luggage to automatically recognize and follow the user. It consists of a camera-equipped robot with motorized wheels and a Raspberry Pi as the core control unit.

## Objective
The project aims to develop a system that allows luggage to follow a user through image processing and, in a later stage, to incorporate a Bluetooth module for remote control from a smartphone.

## Project Video
Watch the project video here.

## Design and Implementation
The design includes a camera module, Raspberry Pi, motor driver, and battery pack. The implementation follows a flowchart that outlines the process from image capture to motor control.

## Setup and Installation

### Hardware Setup
- Assemble the Raspberry Pi, Picamera, motor driver, and battery pack.
- Ensure all components are properly connected and powered.

### Software Setup
Install Raspberry Pi OS on the Raspberry Pi.

Set up the OpenCV environment using the following commands:

```bash
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install -y build-essential cmake make pkg-config
sudo apt-get install -y libjpeg-dev libtiff5-dev libjasper-dev libpng-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install -y libxvidcore-dev libx264-dev
sudo apt-get install -y libfontconfig1-dev libcairo2-dev libgdk-pixbuf2.0-dev libpango1.0-dev libgtk2.0-dev libgtk-3-dev
sudo apt-get install -y libatlas-base-dev gfortran
sudo apt-get install -y python3-dev
sudo apt-get install -y libhdf5-dev libhdf5-serial-dev libhdf5-103 python3-pyqt5
```

## Usage
- Configure the GPIO pins as per the code.
- Run the main script to start the autonomous luggage follower.

## Testing and Results
Testing confirmed that the system can reliably detect and follow the user. The results include successful hardware assembly, OpenCV setup, and color recognition program development.

## Conclusion
The project successfully enabled the luggage to follow the user automatically, using graphical identification and localization via camera.

## Future Work
- Enhance Bluetooth integration for accurate distance measurement.
- Implement predictive algorithms for smoother tracking.
- Develop a smartphone app for better control and interaction.

## Acknowledgments
Special thanks to our instructors and peers for their support and guidance. Inspired by the Raspberry Pi community and open-source contributors.
