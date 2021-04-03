package org.firstinspires.ftc.teamcode.DashConstants;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_Servos {

    public static double INTAKE_ARM_SERVO_DIFFERENCE = 0.06;

    public static double INTAKE_ARM_LSERVO_HOME     = 0.53;
    public static double INTAKE_ARM_LSERVO_MIN      = 0.53;
    public static double INTAKE_ARM_LSERVO_MAX      = 1.0;
    public static double INTAKE_ARM_LSERVO_MID      = INTAKE_ARM_LSERVO_MAX - INTAKE_ARM_SERVO_DIFFERENCE;

    public static double INTAKE_ARM_RSERVO_HOME     = 0.48;
    public static double INTAKE_ARM_RSERVO_MIN      = 0.0;
    public static double INTAKE_ARM_RSERVO_MAX      = 0.48;
    public static double INTAKE_ARM_RSERVO_MID      = INTAKE_ARM_RSERVO_MIN + INTAKE_ARM_SERVO_DIFFERENCE;

    public static double SHOOT_SERVO_HOME           = 0.86;
    public static double SHOOT_SERVO_MIN            = 0.65;
    public static double SHOOT_SERVO_MAX            = 0.86;

    public static double LOCK_SERVO_HOME            = 0.47;
    public static double LOCK_SERVO_MIN             = 0.3;
    public static double LOCK_SERVO_MAX             = 0.47;

    public static double CLAW_RSERVO_HOME           = 0.6;
    public static double CLAW_RSERVO_MIN            = 0.18;
    public static double CLAW_RSERVO_MAX            = 0.6;

    public static double CLAW_LSERVO_HOME           = 0.2;
    public static double CLAW_LSERVO_MIN            = 0.2;
    public static double CLAW_LSERVO_MAX            = 0.65;

}