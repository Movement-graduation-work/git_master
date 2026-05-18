# Master Operation

This directory contains only the files needed to run the current working master vehicle control path.

## Hardware Mapping

- Master IP: `192.168.0.116`
- Slave IP: `192.168.0.140`
- UDP target: `192.168.0.140:5005`
- Right front/rear wheels: `/dev/ttyUSB2`, MD400T command `0x82`
- Left front/rear wheels: `/dev/ttyUSB1`, MD400T command `0x82`
- Unused for driving: `/dev/ttyUSB0`, `/dev/ttyUSB2` command `0x83`

## Files

- `run_master_wheel_keyboard.sh`
  - Main entrypoint.
  - Loads ROS2 Jazzy.
  - Starts master wheel bridge.
  - Starts UDP sender to slave.
  - Starts keyboard teleop UI.
  - Sends stop command on exit.

- `pretty_master_wheel_teleop.py`
  - Reads keyboard input.
  - Publishes ROS2 `/cmd_vel`.
  - Hold `i` to move forward, release to auto-stop.

- `md400t_usb2_cmdvel_bridge.py`
  - Subscribes to `/cmd_vel`.
  - Converts `linear.x` and `angular.z` to left/right wheel driver commands.
  - Sends serial MD400T commands to `/dev/ttyUSB2` and `/dev/ttyUSB1`.

- `cmd_vel_udp_sender.py`
  - Subscribes to `/cmd_vel`.
  - Sends UDP JSON packets to the slave.
  - Sends both raw `/cmd_vel` values and calculated wheel commands.

- `md_port_test_header.py`
  - Manual per-port/per-command motor test helper.
  - Use it to verify which USB serial port controls which wheel pair.

- `setup_master_operation.sh`
  - Installs runtime packages needed by these scripts.
  - Checks that ROS2 Jazzy exists.
  - Sets executable bits and runs Python syntax checks.

## Setup

Run once on the master machine:

```bash
cd ~/git_master/operation
bash setup_master_operation.sh
```

If the script adds the user to `dialout`, log out and log back in before running the vehicle.

## Run

Run on the master machine:

```bash
cd ~/git_master/operation
bash run_master_wheel_keyboard.sh
```

Keyboard controls:

- `i`: forward
- `,`: backward
- `j`: turn left
- `l`: turn right
- `k` or `space`: stop
- `w` / `x`: linear speed up/down
- `e` / `c`: turn speed up/down
- `Ctrl-C`: stop and exit

## `/cmd_vel`

The master publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.

Used fields:

- `linear.x`: forward/backward velocity
- `angular.z`: yaw/turn velocity

Example:

```text
linear.x = 0.200
angular.z = 0.000
```

This means forward motion without turning.

## UDP Packet To Slave

The UDP sender sends JSON to `192.168.0.140:5005`.

Example payload:

```json
{
  "source": "master",
  "seq": 123,
  "stamp": 1777960000.123,
  "linear_x": 0.2,
  "angular_z": 0.0,
  "driver_right": 80,
  "driver_left": 80
}
```

UDP is used because this is real-time motion control. The newest command matters more than reliable delivery of old commands. If one packet drops, the next command arrives immediately.

The slave reads `driver_right` and `driver_left` and writes those values to its MD400T motor driver.

## Manual Port Tests

Right wheel pair:

```bash
python3 md_port_test_header.py --port /dev/ttyUSB2 --cmd 0x82 --speed 150 --repeat 20 --interval 0.05 --auto_stop 1
```

Left wheel pair:

```bash
python3 md_port_test_header.py --port /dev/ttyUSB1 --cmd 0x82 --speed 150 --repeat 20 --interval 0.05 --auto_stop 1
```
