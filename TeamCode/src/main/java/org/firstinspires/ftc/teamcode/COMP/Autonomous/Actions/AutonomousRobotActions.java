package org.firstinspires.ftc.teamcode.COMP.Autonomous.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AutonomousRobotActions {

    public void moveForward(int duration, DcMotor _leftMotor, DcMotor _rightMotor) {
        _leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _leftMotor.setTargetPosition(duration);
        _rightMotor.setTargetPosition(duration);

        _leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _leftMotor.setPower(0.3);
        _rightMotor.setPower(0.3);
    }

    public void moveBackwards(int duration, DcMotor _LeftMotor, DcMotor _RightMotor){
        _LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _LeftMotor.setTargetPosition(-duration);
        _RightMotor.setTargetPosition(-duration);

        _LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _LeftMotor.setPower(-0.3);
        _RightMotor.setPower(-0.3);
    }

    public void turnLeft(int duration, DcMotor _LeftMotor, DcMotor _RightMotor){
        _LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        _LeftMotor.setTargetPosition(duration);
        _RightMotor.setTargetPosition(duration);

        _LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _LeftMotor.setPower(0.2);
        _RightMotor.setPower(0.2);
    }

    public void turnRight(int duration, DcMotor _LeftMotor, DcMotor _RightMotor){
        _LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        _LeftMotor.setTargetPosition(duration);
        _RightMotor.setTargetPosition(duration);

        _LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _LeftMotor.setPower(0.2);
        _RightMotor.setPower(0.2);
    }
}
