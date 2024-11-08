# Gesture Recognition and Robotic Arm Control System

## TensorFlow Lite Gesture Recognition Model

| Software       | Version | URL                                      |
|----------------|---------|------------------------------------------|
| Python         | 3.10.12 | [Python](https://www.python.org)         |
| TensorFlow     | 2.17.0  | [TensorFlow](https://www.tensorflow.org) |

The gesture recognition model was trained on Google Colab. Data was prepared using two CSV files, one containing records of 'fist closure' gestures and the other containing 'random' gestures.

1. **Data Processing**: The CSV files are read, processed, and each gesture record is extracted.
2. **Data Augmentation**: Augmentation techniques are applied to increase data diversity.
3. **Normalization and Splitting**: Data is normalized and split into training, testing, and validation sets.

The model uses a Convolutional Neural Network (CNN) architecture in TensorFlow to perform gesture recognition based on input accelerometer and gyroscope data. This CNN processes 3D temporal sequences, leveraging convolutional layers to capture patterns across time steps and features. 

- **Compilation**: Adam optimizer, cross-entropy loss function, and accuracy as a metric.
- **Training**: Trained for 30 epochs with a batch size of 64, using a validation set for performance tracking.

The model is then converted into TFLite format for deployment on the Adafruit microcontroller.

---

## Adafruit Feather nRF52840 Sense Microcontroller

| Software                  | Version | URL                                                    |
|---------------------------|---------|--------------------------------------------------------|
| Arduino IDE               | 2.3.2   | [Arduino IDE](https://www.arduino.cc/en/software)      |
| Adafruit AHRS             | 2.3.6   | [Adafruit AHRS](https://github.com/adafruit/Adafruit_AHRS) |
| Adafruit TensorFlow Lite  | 1.2.5   | [Adafruit TFLite](https://github.com/adafruit/Adafruit_TFLite) |

The setup allows the Adafruit microcontroller to send data to other devices over BLE. Live data from the accelerometer, gyroscope, and magnetometer are read from the microcontroller and converted to Euler angles using the NXP Sensor Fusion algorithm from the Adafruit AHRS library. The Adafruit TensorFlow Lite library is used to run the gesture recognition model on the live accelerometer and gyroscope data to detect fist gestures.

---

## YOLOv4-Tiny Image Recognition Model

| Software | Version | URL                                      |
|----------|---------|------------------------------------------|
| darknet  | 2.0     | [darknet](https://github.com/hank-ai/darknet) |

The YOLOv4-Tiny model was trained on a Windows 10 PC with an Nvidia RTX 3060 GPU.

1. **Setup**: Prepare `obj` folder with images and annotations, `train.txt` for image paths, `obj.names` for class labels, and `obj.data` for training configuration.
2. **Configuration**: Edit `yolov4-tiny-custom.cfg` with desired model parameters.
3. **Training**: Run the training command:

    ```bash
    ./darknet detector train obj.data yolov4-tiny-custom.cfg yolov4-tiny.conv.29
    ```

---

## Yahboom DOFBOT Robotic Arm on RPi4

| Software       | Version | URL                                      |
|----------------|---------|------------------------------------------|
| Python         | 3.8.0   | [Python](https://www.python.org)         |
| bleak          | 0.22.2  | [bleak](https://pypi.org/project/bleak/) |
| Arm_Lib        | 0.0.5   | [Arm_Lib](https://pypi.org/project/Arm_lib/) |
| opencv-python  | 4.9.0.80| [opencv-python](https://pypi.org/project/opencv-python/) |
| cvlib          | 0.2.7   | [cvlib](https://pypi.org/project/cvlib/) |
| redis          | 5.0.8   | [redis](https://pypi.org/project/redis/) |

The main script runs in a Docker container on the RPi4, managing BLE communication, robotic arm control, and object detection.

- **BLE Communication**: The `bleak` library connects to two Adafruit microcontrollers. One provides arm position data, and the other handles button presses for arm control.
- **Robotic Arm Control**: `Arm_Lib` is a python library provided by Yahboom to control the servos of the robotic arm by providing the angles of each servo.
- **Object Detection**: Captures frames from the camera with `opencv-python`, using `cvlib` and YOLOv4-Tiny for object detection.
- **Remote Display**: Streams frames to a remote display server using the `redis` library.

Upon powering the Adafruit microcontrollers, the script initiates BLE connections, establishing control between the robotic arm, microcontrollers, and the user's computer.

---

## Operation Steps

1. **On the user's computer**: Start the Redis server and run the display script.
  
    ```bash
    redis-server
    python display_script.py
    ```

2. **On the RPi4**: Start the Docker container from the root directory.

    ```bash
    docker start robot_arm_container
    ```

3. **Power on both Adafruit microcontrollers**.

4. **Run the ROS service and the main Python script** on the RPi4:

    ```bash
    roslaunch yahboom_dofbot_ros arm.launch
    python main_script.py
    ```

5. **Begin robotic arm control**.
