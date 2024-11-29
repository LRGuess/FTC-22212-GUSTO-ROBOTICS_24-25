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

package org.firstinspires.ftc.teamcode.COMP.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.COMP.Autonomous.Actions.AutonomousRobotActions;
import org.firstinspires.ftc.teamcode.ComputerVision.CustomATPipline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "AutoRED16", group = "COMP")
public class AutoRED16 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AutonomousRobotActions robot = new AutonomousRobotActions();

    DcMotor LeftMotor;
    DcMotor RightMotor;

    Servo gateServo;

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

        gateServo = hardwareMap.get(Servo.class, Constants.GateConfigurations.GateServoConfigName);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 16)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */

            telemetry.addData(">", "Running useless loop");

        }
        else
        {
            telemetry.addData("Tag of ",tagOfInterest);
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            LeftMotor.setPower(1.0);
            RightMotor.setPower(1.0);

            if(tagOfInterest.id == 16) {
                // do something else
                telemetry.addLine().addData("-->", "16");
                telemetry.update();

                closeGate();

                robot.curveRight(1780, LeftMotor, RightMotor);
                while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

                robot.turnLeft(190, LeftMotor, RightMotor);
                while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

                robot.moveForward(250, LeftMotor, RightMotor);
                while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

                openGate();

                robot.turnLeft(75, LeftMotor, RightMotor);
                while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

                robot.moveBackwards(1700, LeftMotor, RightMotor);
                while (LeftMotor.isBusy() && opModeIsActive()) {idle();}

                closeGate();
//
//                robot.turnRight(15, LeftMotor, RightMotor);
//
//                robot.moveForward(1700, LeftMotor, RightMotor);
//                while (LeftMotor.isBusy() && opModeIsActive()) {idle();}
//
//                robot.turnLeft(25, LeftMotor, RightMotor);
//                while (LeftMotor.isBusy() && opModeIsActive()) {idle();}
            }
        }
    }

    private void openGate(){
        gateServo.setPosition(Constants.GateConfigurations.GateOpenPos);
    }
    private void closeGate(){
        gateServo.setPosition(Constants.GateConfigurations.GateClosePos);
    }

    private void stopMotors() {
        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }
}