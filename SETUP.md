# Setup

Everything another team needs to rebuild this car and run both challenges.
Target system: **Ubuntu 24.04 LTS arm64 on a Raspberry Pi 5**, **ROS 2 Jazzy**,
Python **3.12**.

Two independent controllers share one hardware layer:

| Package | Node | Challenge |
|---|---|---|
| `renkay_open` | `open_challenge` (`open_node`) | Open Challenge: laps only |
| `renkay_obstacle` | `obstacle_challenge` (`obstacle_node`) | Obstacle Challenge: wall following and red/green pillars |
| `renkay_bringup` | launch files | starts RRC Lite, LiDAR, camera and motor driver |
| `renkay_open` / `renkay_obstacle` | `tb6612fng_motor` (`motor_driver`) | TB6612FNG motor driver, same file in both packages |

## 1. Hardware

| Item | Model | Connection | Notes |
|---|---|---|---|
| Computer | Raspberry Pi 5 | - | Ubuntu 24.04 arm64, ROS 2 Jazzy |
| Controller board | HiWonder RRC Lite | USB, `1a86:55d4` | IMU 50 Hz, button, buzzer, steering servo |
| LiDAR | LDROBOT LD19 / STL-19P | USB-serial CP210x, `10c4:ea60` | 360 deg, 12 m, 230400 baud, publishes `/scan` |
| Camera | USB camera, 66.7 deg HFOV | USB, `32e6:9005` | single fixed forward camera, publishes `/usb_cam/image_raw` |
| Motor driver | TB6612FNG | GPIO chip 4: PWM A 12, AIN1 5, AIN2 6, STBY 13 | 1 kHz PWM |
| Battery | 2S Li-Po 7.4 V 5000 mAh 20C | XT60 to the RRC Lite | motors run from the battery through the TB6612 |
| Wheels | 61.0 mm diameter, 14.5 mm wide | - | measured with a caliper |
| Servo rail | 5 V regulated from the RRC Lite | - | steering servo only |

`ros_robot_controller_msgs` is a HiWonder package, not a standard ROS package.
It comes from the RRC Lite / MentorPi ROS image or from the
[HiWonder MentorPi-ROS](https://github.com/Hiwonder/MentorPi-ROS) repository
(branch `MentorPi-M1`, directory `driver/`); section 5 installs it.

## 2. Operating system

1. Flash **Ubuntu Server 24.04 LTS (64-bit) for Raspberry Pi** with Raspberry Pi
   Imager to the Pi 5 NVMe/SD card.
2. In the Imager advanced settings set the hostname, the user, the Wi-Fi and
   enable SSH.
3. First boot, update the system:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

## 3. ROS 2 Jazzy

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-ros-base
source /opt/ros/jazzy/setup.bash
```

Check it:

```bash
printenv ROS_DISTRO        # jazzy
ros2 pkg list | head
```

Add the source line to `~/.bashrc` if you want it in every shell:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 4. Clone the repository

```bash
sudo apt install -y git
git clone https://github.com/strvaeq/Robotek-WRO2026.git
cd Robotek-WRO2026
git checkout audit/30-points      # branch with this setup; after the merge, main
```

## 5. Dependencies

### 5.1 Debian packages

```bash
sudo apt install -y \
  build-essential cmake rsync v4l-utils \
  ros-jazzy-cv-bridge ros-jazzy-usb-cam ros-jazzy-tf2-ros \
  python3-numpy python3-opencv python3-serial python3-yaml python3-lgpio
```

`python3-lgpio` is required by the TB6612FNG driver; without it the node starts
with the motor disabled and logs `lgpio missing; motor disabled`.

### 5.2 Python packages

`requirements.txt` lists only pip packages, pinned to the versions CI uses:
`numpy`, `opencv-python-headless`, `pytest`, `pyyaml`. ROS provides `rclpy`,
`cv_bridge`, `std_msgs`, `sensor_msgs` and `ros_robot_controller_msgs`.

On Ubuntu 24.04 the pip packages are also available from apt, so the car needs
no pip install:

```bash
sudo apt install -y python3-pytest
```

For CI parity in a virtual environment:

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
```

### 5.3 HiWonder board packages (not in this repository)

`ros_robot_controller_msgs` and the RRC Lite driver come from Hiwonder. If the
board image already has `~/ros2_ws/src/driver`, use it. Otherwise clone what is
needed into a separate workspace so this repository stays clean:

```bash
mkdir -p ~/renkay_deps_ws/src && cd ~/renkay_deps_ws/src
git clone --depth 1 -b MentorPi-M1 https://github.com/Hiwonder/MentorPi-ROS.git mentorpi
cp -r mentorpi/driver/ros_robot_controller_msgs .
cp -r mentorpi/driver/ros_robot_controller .
git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

cd ~/renkay_deps_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  ros_robot_controller_msgs ros_robot_controller ldlidar_stl_ros2
source install/setup.bash
```

`ros_robot_controller_msgs` provides the messages the controllers import:
`SetAckerServoState`, `ButtonState`, `BuzzerState`. The LiDAR package provides
the `ldlidar_stl_ros2_node` executable that publishes `/scan`.

## 6. Build

```bash
cd ~/Robotek-WRO2026
source /opt/ros/jazzy/setup.bash
source ~/renkay_deps_ws/install/setup.bash     # if you used section 5.3
colcon build --symlink-install
source install/setup.bash
```

`colcon build` compiles `renkay_open`, `renkay_obstacle` and `renkay_bringup`.
A shorter path: `bash tools/setup-pi.sh` does sections 3, 5.1, 7 and 6 in one
step (`--check` only inspects, `--dry-run` prints the commands).

Check the packages are visible:

```bash
ros2 pkg list | grep renkay
# renkay_bringup
# renkay_obstacle
# renkay_open
```

## 7. Device permissions

The car uses stable device names instead of `/dev/ttyACM0` and `/dev/ttyUSB0`:

| Device | udev name | USB ID |
|---|---|---|
| RRC Lite board | `/dev/renkay_rrc` | `1a86:55d4` |
| LD19 LiDAR | `/dev/renkay_lidar` | `10c4:ea60` |
| Camera | `/dev/v4l/by-id/...-video-index0` | `32e6:9005` (no rule needed) |
| Motor GPIO | `/dev/gpiochip4` | group `gpio` |

Install the rules and join the groups:

```bash
bash tools/setup-pi.sh
# or by hand:
sudo cp tools/99-renkay.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,video,gpio "$USER"
```

Log out and back in (groups only apply to new sessions), unplug and replug both
USB cables, then check:

```bash
ls -l /dev/renkay_rrc /dev/renkay_lidar
bash tools/setup-pi.sh --check
```

## 8. Running each challenge

Before every first run of the day: **put the car on blocks so the wheels are off
the ground**, then confirm the topics exist before the car moves.

```bash
cd ~/Robotek-WRO2026
source /opt/ros/jazzy/setup.bash
source ~/renkay_deps_ws/install/setup.bash
source install/setup.bash
```

### 8.1 Open Challenge

```bash
ros2 launch renkay_bringup open.launch.py
```

This starts the RRC Lite driver, the LD19 driver, the USB camera, the TB6612FNG
driver and the `open_challenge` node. In a second terminal, the expected topics:

```console
$ ros2 topic list
/motor_vel
/parameter_events
/ros_robot_controller/acker_servo/set_state
/ros_robot_controller/button
/ros_robot_controller/imu_raw
/ros_robot_controller/set_buzzer
/rosout
/scan
/usb_cam/camera_info
/usb_cam/image_raw
```

```console
$ ros2 node list
/LD19
/open_challenge
/ros_robot_controller
/tb6612fng_motor
/usb_cam
```

Press the RRC Lite button once. The node beeps, keeps the car still for 4 s to
calibrate the gyroscope bias, then drives until 3 laps are counted (end beep:
two tones at 784 Hz). Press the button again to stop at any time.

Useful launch arguments:

```bash
ros2 launch renkay_bringup open.launch.py --show-args
ros2 launch renkay_bringup open.launch.py lidar_port:=/dev/ttyUSB0
ros2 launch renkay_bringup open.launch.py video_device:=/dev/video2
```

`use_sim_time` is `false` on the car. `params_file` exists for ROS parameter
overrides, but the controllers read their tuning from
`renkay_open/config.py` / `renkay_obstacle/config.py`, so `config/open.yaml`
and `config/obstacle.yaml` are reference documents, not live parameters.

### 8.2 Obstacle Challenge

```bash
ros2 launch renkay_bringup obstacle.launch.py
```

Same hardware plus the camera subscription. The first confirmed red or green
block selects the wall to follow (red: right wall, green: left wall) and the
node corrects heading around the block. After 3 laps the node starts the
parallel parking manoeuvre (`PARKING_ENABLED` in `config.py`): it goes through
APPROACH, ALIGN, REVERSE_ARC and STRAIGHTEN, then stops; it aborts and stops if
the front clearance drops below 0.10 m or after `PARK_MAX_TIME_S`. Topics are
the same as 8.1 with `/obstacle_challenge` in `ros2 node list`. Check the camera
before driving:

```bash
ros2 topic hz /usb_cam/image_raw     # the pipeline runs at about 6 fps
ros2 topic echo /scan --once | head
```

### 8.3 Hardware only

```bash
ros2 launch renkay_bringup bringup.launch.py
```

Starts RRC Lite, LiDAR, camera and motor driver without a controller, for
sensor checks (`ros2 topic hz /scan`, camera calibration) before a run.

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'ros_robot_controller_msgs' not found` | Hiwonder packages missing or not sourced | Section 5.3, then rebuild and `source ~/renkay_deps_ws/install/setup.bash` |
| `Package 'ldlidar_stl_ros2' not found` | LiDAR driver not built | Section 5.3 |
| `No module named cv2` | OpenCV not installed for system Python | `sudo apt install python3-opencv` |
| `lgpio missing; motor disabled` | `python3-lgpio` missing | `sudo apt install python3-lgpio` |
| Node runs, the car does not move | no command on `/motor_vel` or GPIO error | `ros2 topic echo /motor_vel`; `ros2 topic info /motor_vel -v`; check the log for `GPIO chip 4 not usable` |
| `ros2 topic hz /scan` prints nothing | wrong LiDAR port or permissions | `ls -l /dev/renkay_lidar`; try `lidar_port:=/dev/ttyUSB0`; check the groups of section 7 |
| `/usb_cam/image_raw` empty | wrong `video_device` | `v4l2-ctl --list-devices`; pass `video_device:=/dev/videoN` |
| Camera image is upside down | the camera is mounted inverted on this car | the current detector does not rotate the frame, so the horizontal offset sign is mirrored; rotate the camera or flip the frame in `color_vision.py` |
| The car steers the wrong way | sign convention | edit `SERVO_SIGN` or `GYRO_SIGN` in `config.py` before touching the gains |
| Stops right after starting | watchdog saw a sensor gap | check `/scan` and `/ros_robot_controller/imu_raw` rates; the limits are `IMU_TIMEOUT_S` and `LIDAR_TIMEOUT_S` in `config.py` |
| Two nodes fight over `/motor_vel` | both challenges running | stop one launch (`Ctrl+C`), only one controller may run |
