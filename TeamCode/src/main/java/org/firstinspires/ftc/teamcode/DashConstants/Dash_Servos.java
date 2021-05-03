package org.firstinspires.ftc.teamcode.DashConstants;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_Servos {

    public static double INTAKE_ARM_SERVO_DIFFERENCE = 0.06;

    public static double INTAKE_ARM_LSERVO_HOME     = 0.30;
    public static double INTAKE_ARM_LSERVO_MIN      = 0.08;
    public static double INTAKE_ARM_LSERVO_MAX      = 0.4;
    public static double INTAKE_ARM_LSERVO_MID      = INTAKE_ARM_LSERVO_MAX - INTAKE_ARM_SERVO_DIFFERENCE;

    public static double INTAKE_ARM_RSERVO_HOME     = 0.18;
    public static double INTAKE_ARM_RSERVO_MIN      = 0.08;
    public static double INTAKE_ARM_RSERVO_MAX      = 0.4;
    public static double INTAKE_ARM_RSERVO_MID      = INTAKE_ARM_RSERVO_MIN + INTAKE_ARM_SERVO_DIFFERENCE;

    public static double SHOOT_SERVO_HOME           = 0.86;
    public static double SHOOT_SERVO_MIN            = 0.65;
    public static double SHOOT_SERVO_MAX            = 0.86;

    public static double LOCK_SERVO_HOME            = 0.3;
    public static double LOCK_SERVO_MIN             = 0.3;
    public static double LOCK_SERVO_MAX             = 0.3;

    public static double CLAW_RSERVO_HOME           = 0.7;
    public static double CLAW_RSERVO_MIN            = 0.25;
    public static double CLAW_RSERVO_MAX            = 0.7;

    public static double CLAW_LSERVO_HOME           = 0.0;
    public static double CLAW_LSERVO_MIN            = 0.0;
    public static double CLAW_LSERVO_MAX            = 0.5;

    public static double OUT_POS                    = 0.0;
    public static double UP2_POS                    = 0.59;
    public static double UP_POS                     = 0.7;
    public static double IN_POS                     = 0.9;

    public static double R_WING_OUT                 = 0.26;
    public static double R_WING_MID                 = 0.55;
    public static double R_WING_UP                  = 0.61;

    public static double L_WING_OUT                 = 0.81;
    public static double L_WING_MID                 = 0.55;
    public static double L_WING_UP                  = 0.48;

}