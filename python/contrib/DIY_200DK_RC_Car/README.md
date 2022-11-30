# **DIY_200DK_RC-CAR**

Do you want to DIY a toy car platform powerred by Atlas 200 DK? This repo will help you build it on your own.

Four projects are presented here. They are based on the same RC car platform: [Elegoo Smart Robot Car Kit](https://www.amazon.ca/ELEGOO-Ultrasonic-Bluetooth-Intelligent-Educational/dp/B07485YQP8), which is easy to buy or find its equivalents.

Note that, the repos are verfied on **CANN 3.0.0** / **C73** / **NPU 20.0**, but should be easily transferred to newer **CANN** versions.


| Project | Description | Features | Programming Language | Used Models |
| ------ | ------ | ------ |  ------ | ------ |
| [HandposeRCcar](https://github.com/Atlas200dk/sample-handposeRCcar) | Use hand gesture to remote control the car moving around | <ul><li>Two setup options: I2C and UART</li> <li>bluetooth for remote control</li> <li>Aduino for motor control</li> </ul>| C++ | handpose |
| [Automated Traffic Police Signal Recognition System](https://github.com/kelvinkoon/ece491-group30) | Firstly, based on headpose to decide whether the controller is looking at the car, then recognize the controller's body gesture to move around | <ul><li>Python UART Interface</li> <li>State Machine Implementation</li> <li>DK installed on RC car (powered by power bank)</li> <li>Aduino for motor control</li></ul> | Python | <ul><li>bodypose</li> <li>object detection </li> <li> headpose </li></ul>  |
| [Hand Gesture Controlled Robot Pet](https://github.com/diannakan1998/hand_gesture_controlled_robot_pet) | The Robot Pet (RC car) will trace your object, following your hand gesture to take a picture and move around  | <ul><li>Servo Moter for Camera</li> <li>RasperryPi with LCD for display</li> <li>UDP connection between 200 DK with RasperryPi </li> <li>Aduino for motor control </li></ul>| Python | <ul><li>handpose </li> <li>hand detection</li><li> object detection </li></ul> |
| [Body Gesture Controlled Robot Pet (ROS)](https://github.com/Ascend-Huawei/gesture_controlled_robot_pet) | The Robot Pet (RC car) will trace your object, following your body gesture to take a picture and move around  | <ul><li>Servo Moter for Camera</li> <li>RasperryPi with LCD for display</li> <li>UDP connection between 200 DK with RasperryPi </li> <li>Aduino for motor control </li></ul>| Python (ROS) | <ul><li>bodypose </li> </ul> |

These examples can be good start for your own DIY project. For example, you can add 

Possible extensions: 
- adding a [Wifi Module](https://www.amazon.ca/TP-LINK-TL-WR902AC-Wireless-Travel-Router/dp/B01N5RCZQH?th=1) for remote display/control
- adding a filter for stable poses estimation
- more stable bluetooth hardware
- SLAM
- etc. 
