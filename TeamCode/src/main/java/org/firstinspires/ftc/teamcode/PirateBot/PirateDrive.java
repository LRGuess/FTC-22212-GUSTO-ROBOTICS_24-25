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

package org.firstinspires.ftc.teamcode.PirateBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

@TeleOp(name="Pirate Drive", group="V1")
public class PirateDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor boomMotor = null;
    private DcMotor armMotor = null;
    private Servo pivotServo = null;

    private PIDFCoefficients pidCoefficients = new PIDFCoefficients(1.0, 0.0, 0.0, 0.0);
    //private CRServo intakeServo = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables
        leftDrive  = hardwareMap.get(DcMotor.class, "LeftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "RightMotor");
        boomMotor = hardwareMap.get(DcMotor.class, "boomMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");

        // Set motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset and initialize the encoders
        boomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PID coefficients for boomMotor

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
    // Declare variables to store top and bottom positions
    // Declare variables to store top and bottom positions for boomMotor
    private int boomTopPosition = 1000; // Default top position
    private int boomBottomPosition = 0; // Default bottom position

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double boomPower;
        double armPower;

        // Tank Mode uses one stick to control each wheel.
        leftPower  = -gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        if (gamepad1.dpad_left)
            armPower = 0.5;
        else if (gamepad1.dpad_right)
            armPower = -0.5;
        else
            armPower = 0;

        if (gamepad1.dpad_up)
            boomPower = 0.7;
        else if (gamepad1.dpad_down)
            boomPower = -0.7;
        else
            boomPower = 0;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Send calculated power to arm motor
        armMotor.setPower(armPower);

        // Set brake behavior for arm motor
        if (armPower == 0) {
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Maintain boom motor position
        if (boomPower != 0) {
            boomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            boomMotor.setPower(boomPower);
        } else {
            boomMotor.setTargetPosition(boomMotor.getCurrentPosition());
            boomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            boomMotor.setPower(1); // Small power to hold position
        }

        // Set top and bottom positions dynamically for boomMotor
        if (gamepad1.right_bumper && gamepad1.y) {
            boomTopPosition = boomMotor.getCurrentPosition();
        } else if (gamepad1.right_bumper && gamepad1.a) {
            boomBottomPosition = boomMotor.getCurrentPosition();
        }

        // Automatically move boom to top or bottom position
        if (gamepad1.y) {
            boomMotor.setTargetPosition(boomTopPosition);
            boomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            boomMotor.setPower(1.0); // Full power to reach position
        } else if (gamepad1.a) {
            boomMotor.setTargetPosition(boomBottomPosition);
            boomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            boomMotor.setPower(1.0); // Full power to reach position
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Boom Motor Encoder", boomMotor.getCurrentPosition());
        telemetry.addData("Arm Motor Position", armMotor.getCurrentPosition());
        telemetry.addData("Boom Top Position", boomTopPosition);
        telemetry.addData("Boom Bottom Position", boomBottomPosition);
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
