# TCEBot Setup on Custom Raspberry Pi Image (ROS 2 Jazzy)

## Prerequisites
- A Raspberry Pi board (recommended: Raspberry Pi 4/5)
- microSD card (32GB or larger)
- Internet connection (for initial setup)
- A computer with an SD card writer

## Step 1: Download Custom Image
Download the pre-built custom image with ROS 2 Jazzy from the following link:

**[Download Image](https://drive.google.com/file/d/1pRz1IW64-c3jGCG04VXSExZg8M7-tjX-/view?usp=drive_link)**  

## Step 2: Flash the Image to microSD Card
Use a tool like **Raspberry Pi Imager** or **balenaEtcher** to flash the image:

1. Insert the microSD card into your computer.
2. Open **Raspberry Pi Imager** (or **balenaEtcher**).
3. Select the downloaded custom image.
4. Choose the microSD card as the target.
5. Click "Write" and wait for the process to complete.
6. Once done, eject the microSD card safely.

## Step 3: Boot Raspberry Pi
1. Insert the microSD card into the Raspberry Pi.
2. Power on the Raspberry Pi and wait for the system to boot.
3. The Raspberry Pi will automatically connect to the hotspot **tcebot** with the password `123456`.
4. Connect to the Raspberry Pi using SSH:
   ```bash
   ssh tcebot@<Raspberry_Pi_IP>
   ```
   Default password: `123456`

**Note:** You need to change the Wi-Fi settings for future proceedings.

![Alt text](https://raw.githubusercontent.com/dhanushshettigar/tcebot_bringup/refs/heads/main/media/SSH-Login.png)


## Step 4: Update System Packages
Run the following commands to ensure the system is up-to-date:
```bash
sudo apt update && sudo apt upgrade -y
```

## Step 5: Verify ROS 2 Installation
Check if ROS 2 Jazzy is installed correctly:
```bash
ros2 --version
```
Expected output should confirm ROS 2 Jazzy is installed.

## Step 6: Setup TCEBot Packages
Clone and build the required TCEBot packages:

```bash
mkdir -p ~/Documents/ros2_ws/src && cd ~/Documents/ros2_ws/src
git clone https://github.com/dhanushshettigar/tcebot_bringup.git
git clone https://github.com/dhanushshettigar/tcebot_description.git
git clone https://github.com/dhanushshettigar/tcebot_control.git
git clone https://github.com/dhanushshettigar/tcebot_imu.git
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ..
colcon build
source install/setup.bash

sudo chmod 666 /dev/ttyUSB0

sudo usermod -aG gpio tcebot
sudo chown root:gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
```

## Troubleshooting
- If SSH fails, connect a monitor and keyboard to troubleshoot.
- Ensure all dependencies are installed:
  ```bash
  sudo apt install ros-jazzy-image-transport-plugins
  sudo apt install ros-jazzy-robot-localization
  ```
- Check - https://github.com/dhanushshettigar/ros2_mpu6050_driver.git for MPU6050 Setup
---
Your TCEBot should now be up and running on ROS 2 Jazzy!
