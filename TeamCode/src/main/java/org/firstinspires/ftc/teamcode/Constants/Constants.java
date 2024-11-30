package org.firstinspires.ftc.teamcode.Constants;

public class Constants {

    public static class DriveConfiguations {
        public static String LeftMotorConfigName = "LeftMotor";
        public static String RightMotorConfigName = "RightMotor";
        public static double MaxDriveMotorSpeed = 0.5;
    }

    public static class StructureConfigurations {
        public static String BoomMotorConfigName = "boomMotor";
        public static double MaxBoomMotorSpeed = 0.8;
        public static String ArmMotorConfigName = "armMotor";
        public static double MaxArmMotorSpeed = 0.8;
    }

    public static class IntakeConfiguration {
        public static String IntakeServoConfigName = "intakeServo";
    }

    public static class GateConfigurations {
        public static String GateServoConfigName = "gateServo";
        public static double GateOpenPos = 0.02;
        public static double GateClosePos = 0.4;
    }

    public static class ColorSensorConfigurations {
        public static String LineSensorConfigName = "lineSensor";
        public static float LineSensorGain = 2;
    }

    public static class TouchSensorConfigurations {
        public static String MinBoomTouchSensorConfigName = "minBoomTouchSensor";
        public static String MaxBoomTouchSensorConfigName = "maxBoomTouchSensor";
        public static String MaxArmTouchSensorConfigName = "maxArmTouchSensor";
    }
    //public static int nintyDegreeTurn = 190;
}
