# RAVEN - Brain Stack ("The Intelligence Layer")

![Raven Brain](https://img.shields.io/badge/Component-Brain-blue) ![Status](https://img.shields.io/badge/Status-Active-success)

The **Brain Stack** is the high-level decision making unit of the Raven autonomous vehicle, running on the **Raspberry Pi 4 / Jetson**. It handles computer vision, state management, and communication with the embedded platform.

## 📚 Documentation
> **Full Technical Documentation:** [bosch-future-mobility-challenge-documentation.readthedocs-hosted.com](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com)

---

## 🚀 Key Features

| Task ID | Feature Name | Description |
| :--- | :--- | :--- |
| **[001b]** | **Video Stream Handler** | Bridges Gazebo/ROS images to OpenCV for processing. |
| **[002a]** | **IPM Calibration** | Tools to calculate Inverse Perspective Mapping (Birds-eye view) matrices. |
| **[002b]** | **Lane Segmentation** | Extracts lane lines using ROI masking and thresholding. |
| **[002c]** | **Lateral Control** | Calculates steering error from lane centroid to keep the car centered. |
| **[005a]** | **Serial Handler** | Robust Python-based serial communication with the Nucleo board (10Hz). |
| **[006a]** | **FSM (State Machine)** | Central logic managing states (Idle, Lane Keeping, Intersection, Parking). |
| **[008b]** | **Sign Recognition** | YOLOv8-based detection of Stop Signs, Traffic Lights, and Pedestrians. |

## 🛠️ Usage

### Running the Stack
```bash
# Start the main state machine and vision pipeline
python3 src/main.py
```

### Configuration
Configuration files are located in `src/utils/config/`.
