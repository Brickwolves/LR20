package org.firstinspires.ftc.teamcode.Utilities.DashConstants;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_Servos {

    public static double SERVO_DIFFERENCE = 0.06;

    public static double LEFT_ARM_SERVO_HOME = 0.53;
    public static double LEFT_ARM_SERVO_MIN = 0.53;
    public static double LEFT_ARM_SERVO_MAX = 1.0;
    public static double LEFT_ARM_SERVO_MID = LEFT_ARM_SERVO_MAX - SERVO_DIFFERENCE;

    public static double RIGHT_ARM_SERVO_HOME = 0.48;
    public static double RIGHT_ARM_SERVO_MIN = 0.0;
    public static double RIGHT_ARM_SERVO_MAX = 0.48;
    public static double RIGHT_ARM_SERVO_MID = RIGHT_ARM_SERVO_MIN + SERVO_DIFFERENCE;

    public static double SHOOT_SERVO_HOME = 0.86;
    public static double SHOOT_SERVO_MIN = 0.65;
    public static double SHOOT_SERVO_MAX = 0.86;

    public static double LOCK_SERVO_HOME = 0.47;
    public static double LOCK_SERVO_MIN = 0.3;
    public static double LOCK_SERVO_MAX = 0.47;
}