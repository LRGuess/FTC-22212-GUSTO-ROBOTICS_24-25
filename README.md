## Welcome!
This is the official competition repository of William Aberhart High School team 22212 Gusto Robotics, (aka 'Blue team'), and contains the code that will be used throughout the competition season. At the end of the season, this project will be archived. Please do not contribute to the project if you are outside of our team but feel free to use code to help your own team out. Always remember, *gracious professionalism* is at the core of *First* Robotics!

## Requirements
To use this Android Studio project, you will need Android Studio 2021.2 (codename Chipmunk) or later.

To program your robot in Blocks or OnBot Java, you do not need Android Studio.

## Getting Started
If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html), and then migrate to the [OnBot Java Tool](https://ftc-docs.firstinspires.org/programming_resources/onbot_java/OnBot-Java-Tutorial.html) or to [Android Studio](https://ftc-docs.firstinspires.org/programming_resources/android_studio_java/Android-Studio-Tutorial.html) afterwards.

## Downloading the Project
If you are an Android Studio programmer, there are several ways to download this repo.  Note that if you use the Blocks or OnBot Java Tool to program your robot, then you do not need to download this repository.

* If you are a git user, you can clone the most current version of the repository:

<p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git</p>

* Or, if you prefer, you can use the "Download Zip" button available through the main repository page.  Downloading the project as a .ZIP file will keep the size of the download manageable.

* You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the [Releases](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) page for this repository.

* The Releases page also contains prebuilt APKs.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/doc/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FIRST Tech Challenge Community site:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.

# Release Information

## Version 10.0  (20240828-111152)

### Breaking Changes
* Java classes and Blocks for TensorFlow Object Detection have been removed.
* `AngularVelocity.unit` which was of type `AngleUnit` has been renamed `AngularVelocity.angleUnit` of type `UnnormalizedAngleUnit`

### Enhancements
* Sample for REV Digital Indicator has been added - ConceptRevLED
* Adds support for the [Sparkfun QWIIC LED Stick](https://www.sparkfun.com/products/18354)
  * To connect it directly, you need this [cable](https://www.sparkfun.com/products/25596)
* Adds ConceptLEDStick OpMode
* Adds Blocks for colors black, blue, cyan, dkgray, gray, green, ltgray, magenta, red, white, and yellow.
* Adds an "evaluate but ignore result" Block that executes the connected block and ignores the result. Allows you to call a function and ignore the return value.
* Adds I2C driver for Maxbotix Maxsonar I2CXL sonar rangefinder
* Adds Blocks for setPwmEnable, setPwmDisable, and isPwmEnabled for servos and CR servos.
* In the Blocks editor: a \n in the ExportToBlocks annotation's comment field is displayed as a line break.
* Telemetry has new method setNumDecimalPlaces
* Telemetry now formats doubles and floats (not inside objects, just by themselves)
* Adds support for the Limelight 3A.
* Adds initial support for the REV Servo Hub
  * Both the Robot Controller and Driver Station need to be updated to version 10.0 in order for Servo Hubs to be
    configurable as Servo Hubs. If the app on either device is outdated, the Servo Hub will show up as an Expansion Hub,
    and some functionality will not work as expected. You should wait to create a configuration that includes a Servo Hub
    until both the Driver Station and Robot Controller apps have been updated to version 10.0.
  * Updating the Servo Hub's firmware and changing its address can only be done using the REV Hardware Client at this time
* Adds support for the REV 9-Axis IMU (REV-31-3332)
  * The REV 9-Axis IMU is only supported by the [Universal IMU interface](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html)
  * Adds `Rev9AxisImuOrientationOnRobot` Java class.
  * If you mentally substitute this IMU's I2C port for the Control Hub's USB ports, `RevHubOrientationOnRobot` is also compatible with this sensor
  * Adds Blocks for Rev9AxisImuOrientationOnRobot, including RevHubImuOrientationOnRobot.xyzOrientation and  RevHubImuOrientationOnRobot.zyxOrientation.
  * Adds Blocks samples SensorRev9AxisIMUOrthogonal and SensorRev9AxisIMUNonOrthogonal.
* Improves Blocks support for RevHubImuOrientationOnRobot.
  * Adds Blocks for RevHubImuOrientationOnRobot.xyzOrientation and  RevHubImuOrientationOnRobot.zyxOrientation.
  * Adds Blocks samples SensorHubIMUOrthogonal (replaces SensorIMU) and SensorHubIMUNonOrthogonal.
* Updates EasyOpenCV, AprilTag, OpenCV, and `libjpeg-turbo` versions
* Adds Blocks for max and min that take two numbers.
* Adds Blocks OpModes ConceptRevSPARKMini, RobotAutoDriveByEncoder, RobotAutoDriveByGyro, RobotAutoDriveByTime, RobotAutoDriveToAprilTagOmni, and RobotAutoDriveToAprilTagTank.
* Two OpModes with the same name now automatically get renamed with the name followed by a "-" and the class name allowing them to both be on the device.
* Shows the name of the active configuration on the Manage page of the Robot Controller Console
* Updated AprilTag Library for INTO THE DEEP. Notably, `getCurrentGameTagLibrary()` now returns INTO THE DEEP tags.
* Adds Blocks for Telemetry.setMsTransmissionInterval and Telemetry.getMsTransmissionInterval.
* Adds Blocks sample SensorOctoQuad.

### Bug Fixes
* Fixes a bug where the RevBlinkinLedDriver Blocks were under Actuators in the Blocks editor toolbox. They are now Other Devices.
* Fixes a bug where `Exception`s thrown in user code after a stop was requested by the Driver Station would be silently eaten
* Fixed a bug where if you asked for `AngularVelocity` in a unit different than the device reported it in, it would normalize it between -PI and PI for radians, and -180 and 180 for degrees.
