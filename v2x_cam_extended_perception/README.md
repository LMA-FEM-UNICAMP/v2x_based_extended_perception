# v2x_cam_extended_perception

## Overview

The `v2x_cam_extended_perception` package contains a C++ ROS 2 node that converts incoming ETSI  Cooperative Awareness Messages (CAM) into extended perception objects. By extracting the GNSS position, heading, velocity, and dimensions of other connected vehicles from CAM messages, this node projects them into the local map frame and outputs them as standard Autoware detected objects.

This node also utilizes Autoware's `map_height_fitter` and `geography_utils` to accurately project GNSS coordinates (Latitude, Longitude, Altitude) to the local coordinate system and match the Z-coordinate to the pointcloud map.

## Node: `v2x_cam_extended_perception`

### Subscribed Topics

| Topic | Type | Description | 
| ----- | ----- | ----- | 
| `cam/out` | `etsi_its_cam_msgs::msg::CAM` | Ingests the parsed Cooperative Awareness Messages from the V2X receiver. | 
| *(Dynamic via Component Interface)* | `MapProjectorInfo::Message` | Map projector information used to project WGS84 GNSS coordinates into the local `map` frame. |
| `/map/pointcloud_map` | `sensor_msgs::msg::PointCloud2` | Point cloud map for fitting the Z-coordinate. |

### Published Topics

| Topic | Type | Description | 
| ----- | ----- | ----- | 
| `/perception/object_recognition/detection/objects` | `autoware_perception_msgs::msg::DetectedObjects` | Publishes the perceived V2X entities as standard Autoware detected objects. | 

### Parameters

Parameters are typically defined in `config/v2x_cam_extended_perception.param.yaml`.

| Parameter | Type | Default | Description | 
| ----- | ----- | ----- | ----- | 
| `base_frame` | string | `base_link` | The base coordinate frame of the ego vehicle. | 
| `map_frame` | string | `map` | The coordinate frame of the map. | 
| `map_height_fitter.target` | string | `pointcloud_map` | The target map type used for Z-height fitting. | 
| `map_height_fitter.map_loader_name` | string | `/map/pointcloud_map_loader` | The node/service name of the map loader to query for height fitting. | 

## Object Classification Mapping

The node automatically converts ETSI ITS Station Types into Autoware Object Classifications:

*   `PEDESTRIAN` ➔ `PEDESTRIAN`
*   `CYCLIST` ➔ `BICYCLE`
*   `MOPED`, `MOTORCYCLE` ➔ `MOTORCYCLE`
*   `PASSENGER_CAR` ➔ `CAR`
*   `BUS` ➔ `BUS`
*   `LIGHT_TRUCK`, `HEAVY_TRUCK` ➔ `TRUCK`
*   `TRAILER` ➔ `TRAILER`
*   *Other/Unknown* ➔ `UNKNOWN`

> *As ETSI CAM does not present object height, it was choose based in an average value for each class.*

## Usage

You can launch the perception node using the provided XML launch file. This launch file automatically loads the parameters from the `config` directory.

```bash
ros2 launch v2x_cam_extended_perception v2x_cam_extended_perception.xml
```

## Acknowledgments

GNSS to MGRS position conversion based in [autoware_gnss_poser](https://github.com/autowarefoundation/autoware_core/tree/main/sensing/autoware_gnss_poser).