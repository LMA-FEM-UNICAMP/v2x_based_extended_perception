# v2x_extended_perception

## Overview

The `v2x_extended_perception` package is a meta-package and bring-up layer for the V2X extended perception stack. It contains the primary launch files, shell scripts, and configuration parameters required to initialize the UDP drivers, the ETSI message parser, and the downstream perception node.

Rather than housing C++ source code, this package glues the hardware communication layer (such as a Cohda Wireless MK6 OBU) to the ROS 2 Autoware environment.

## Launch Details

The main entry point for the V2X stack is `v2x_extended_perception.launch.py`. This launch file manages the bring-up of multiple nodes and handles the lifecycle transitions for the UDP drivers automatically.

```bash
ros2 launch v2x_extended_perception v2x_extended_perception.launch.py
```

### Launched Nodes
Depending on the launch arguments, this file initializes:
1. **UDP Receiver & Sender (`udp_driver`)**: Handles raw UDP socket communication with the V2X hardware.
2. **ETSI Parser (`etsi_its_conversion`)**: Translates raw UDP payloads into structured ROS 2 ETSI ITS messages.
3. **CAM Extended Perception (`v2x_cam_extended_perception`)**: Ingests the parsed messages and outputs Autoware detected objects.

### Lifecycle Management
The `udp_driver` nodes are implemented as ROS 2 lifecycle nodes. The launch file automatically executes the `scripts/udp_lifecyclenode_transtions.sh` script during startup, which transitions the `udp_receiver` and `udp_sender` nodes through the `configure` (1) and `activate` (3) states so they begin processing traffic immediately.

## Launch Arguments

| Argument | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `launch_drivers` | boolean | `true` | If true, launches the `udp_driver` sender/receiver nodes and the `etsi_its_conversion_node`. |
| `launch_cam_perception` | boolean | `true` | If true, launches the `v2x_cam_extended_perception` node to generate Autoware objects. |

## Node Configurations

The node parameters are defined in the `config/` directory.

### `etsi_its_messages.param.yaml`
Configures the `etsi_its_conversion` node, specifically tailored for hardware like the Cohda Wireless MK-5.

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `has_btp_destination_port` | bool | `true` | Indicates if the incoming message contains a BTP destination port header. |
| `using_cohda_btp_headers` | bool | `true` | Enable specific parsing for Cohda BTP headers. |
| `btp_destination_port_offset` | int | `8` | Byte offset for the BTP destination port. |
| `etsi_message_payload_offset` | int | `78` | Byte offset where the actual ETSI message payload begins. |
| `ros2udp_etsi_types` | list | `[cam]` | The list of ETSI message types to parse from ROS 2 to UDP. |
| `udp2ros_etsi_types` | list | `[cam]` | The list of ETSI message types to parse from UDP to ROS 2. |
| `subscriber_queue_size` / `publisher_queue_size` | int | `10` | Queue sizes for the ROS 2 publishers and subscribers. |

### `udp_driver_receiver.param.yaml`
Configures the incoming UDP connection.
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `ip` | string | `"192.168.140.4"` | Host IP to bind to for receiving messages (ItsUdpBtpIfIndHostName). |
| `port` | int | `4400` | Port to listen on (ItsUdpBtpIfIndPort). |

### `udp_driver_sender.param.yaml`
Configures the outgoing UDP connection.
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `ip` | string | `"192.168.140.6"` | Destination IP to send messages to. |
| `port` | int | `4401` | Destination port for outgoing messages. |

## Future work

This package presents a modular structure designed to easily integrate further implementations of ETSI messages, such as CPM, MAPEM, and SPATEM, as extended perception inputs.