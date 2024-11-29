/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.COMP.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.Constants;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Systems Check", group="COMP")
public class SystemsCheck extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor boomMotor = null;
    private DcMotor armMotor = null;

    private CRServo intakeServo = null;

    private IMU internalIMU = null;
    private ColorSensor lineSensor;

    private TouchSensor minTouchSensor = null;
    private TouchSensor maxTouchSensor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables
        leftDrive  = hardwareMap.get(DcMotor.class, Constants.DriveConfiguations.LeftMotorConfigName);
        rightDrive = hardwareMap.get(DcMotor.class, Constants.DriveConfiguations.RightMotorConfigName);
        boomMotor = hardwareMap.get(DcMotor.class, Constants.StructureConfigurations.BoomMotorConfigName);
        armMotor = hardwareMap.get(DcMotor.class, Constants.StructureConfigurations.ArmMotorConfigName);
        intakeServo = hardwareMap.get(CRServo.class, Constants.IntakeConfiguration.IntakeServoConfigName);
        lineSensor = hardwareMap.get(ColorSensor.class, Constants.ColorSensorConfigurations.LineSensorConfigName);
        internalIMU = hardwareMap.get(IMU.class, "imu");

        minTouchSensor = hardwareMap.get(TouchSensor.class, Constants.TouchSensorConfigurations.MinTouchSensorConfigName);
        maxTouchSensor = hardwareMap.get(TouchSensor.class, Constants.TouchSensorConfigurations.MaxTouchSensorConfigName);

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        internalIMU.initialize(new IMU.Parameters(orientationOnRobot));

        // Set motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        lineSensor.enableLed(true);

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double boomPower;
        double armPower;

        // Tank Mode uses one stick to control each wheel.
        leftPower  = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        boomPower = -gamepad2.left_stick_y;
        armPower = -gamepad2.right_stick_y * 0.8;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        armMotor.setPower(armPower);
        boomMotor.setPower(boomPower);

        if(gamepad2.y) {
            intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeServo.setPower(1);
        }
        else if (gamepad2.a){
            intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeServo.setPower(1);
        }
        else{
            intakeServo.setPower(0);
        }


        telemetry.addLine()
                .addData("Red", lineSensor.red())
                .addData("Green", lineSensor.green())
                .addData("Blue", lineSensor.blue());
        telemetry.addLine();
        telemetry.addData("Alpha", lineSensor.alpha());

        telemetry.addLine();
        telemetry.addData("Min", minTouchSensor.isPressed());
        telemetry.addData("Max", maxTouchSensor.isPressed());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        boomMotor.setPower(0);
        armMotor.setPower(0);
    }

}
