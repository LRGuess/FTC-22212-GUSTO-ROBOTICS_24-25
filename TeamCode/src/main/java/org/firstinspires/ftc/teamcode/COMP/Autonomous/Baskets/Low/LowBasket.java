/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.COMP.Autonomous.Baskets.Low;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.COMP.Autonomous.Actions.AutonomousRobotActions;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "Auto Low Basket", group = "COMP")
public class LowBasket extends LinearOpMode
{
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AutonomousRobotActions robot = new AutonomousRobotActions();
    private final ElapsedTime runtime = new ElapsedTime();

    DcMotor LeftMotor;
    DcMotor RightMotor;
    DcMotor boomMotor;
    DcMotor armMotor;

    Servo gateServo;
    CRServo intakeServo;

    TouchSensor boomMax;
    TouchSensor boomMin;
    TouchSensor armMax;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.1016;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        LeftMotor = hardwareMap.get(DcMotor.class, Constants.DriveConfiguations.LeftMotorConfigName);
        RightMotor = hardwareMap.get(DcMotor.class, Constants.DriveConfiguations.RightMotorConfigName);
        boomMax = hardwareMap.get(TouchSensor.class, Constants.TouchSensorConfigurations.MaxBoomTouchSensorConfigName);
        boomMin = hardwareMap.get(TouchSensor.class, Constants.TouchSensorConfigurations.MinBoomTouchSensorConfigName);
        armMax = hardwareMap.get(TouchSensor.class, Constants.TouchSensorConfigurations.MaxArmTouchSensorConfigName);
        boomMotor = hardwareMap.get(DcMotor.class, Constants.StructureConfigurations.BoomMotorConfigName);
        armMotor = hardwareMap.get(DcMotor.class, Constants.StructureConfigurations.ArmMotorConfigName);

        gateServo = hardwareMap.get(Servo.class, Constants.GateConfigurations.GateServoConfigName);
        intakeServo = hardwareMap.get(CRServo.class, Constants.IntakeConfiguration.IntakeServoConfigName);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        runtime.reset();

        // e.g.
        LeftMotor.setPower(1.0);
        RightMotor.setPower(1.0);


        // do something else
        telemetry.addLine().addData("-->", "16");
        telemetry.update();

        boomMax();
        while (boomMotor.isBusy() && opModeIsActive()) {idle();}
        while(true){
            if (boomMax.isPressed()){
                boomMotor.setPower(0);
                break;
            }
            else {
                boomMotor.setPower(0.65);
            }
        }

        armMax();
        while (armMotor.isBusy() && opModeIsActive()) {idle();}
        while (true) {
            if (armMax.isPressed()) {
                armMotor.setPower(0);
                break;
            } else {
                armMotor.setPower(-1);
            }
        }

        robot.moveForward(-500, LeftMotor, RightMotor);
        while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

        robot.turnRight(100, LeftMotor, RightMotor);
        while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

        armMotor.setPower(-0.5);
        sleep(2000);
        armMotor.setPower(0);

        outtake(4000, 1);
        intake(2000, 1);

        robot.moveForward(500, LeftMotor, RightMotor);
        while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

        robot.turnLeft(500, LeftMotor, RightMotor);
        while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

        robot.moveForward(-2200, LeftMotor, RightMotor);
        while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

        robot.turnLeft(600, LeftMotor, RightMotor);
        while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

        robot.moveForward(-500, LeftMotor, RightMotor);
        while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

        boomMotor.setPower(-0.45);
        while (boomMotor.isBusy() && opModeIsActive()) {idle();}
        sleep(2000);
        boomMotor.setPower(0);
    }

    private void outtake(int milliseconds, double power) {
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServo.setPower(power);
        sleep(milliseconds);
        intakeServo.setPower(0);
    }

    private void intake(int milliseconds, double power) {
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPower(power);
        sleep(milliseconds);
        intakeServo.setPower(0);
    }

    public void boomMax(){
        if (boomMax.isPressed()){
            boomMotor.setPower(0);
            return;
        }
        boomMotor.setPower(0.65);
    }

    public void armMax(){
        if (armMax.isPressed()){
            armMotor.setPower(0);
            return;
        }
        armMotor.setPower(-1);
    }

    private void stopMotors() {
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }
}