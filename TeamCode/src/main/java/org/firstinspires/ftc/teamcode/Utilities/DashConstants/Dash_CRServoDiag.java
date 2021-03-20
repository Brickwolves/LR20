package org.firstinspires.ftc.teamcode.Utilities.DashConstants;
import com.acmerobotics.dashboard.config.Config;

//@Config
public class Dash_CRServoDiag {
    public static String CRSERVO_ID = "claw";
    public static double CRSERVO_POWER = 1;
    public static double millis_to_open = 1772;
    public static double millis_to_close = 1870;

    public static double GLOBAL_CLOSED_POSITION  = -53;
    public static double GLOBAL_OPEN_POSITION    = 1900;
    public static double RELATIVE_CLOSED_POSITION  = -1755;
    public static double RELATIVE_OPEN_POSITION    = 0;

    public static double MOE = 25;


    public static boolean STATE_MACHINE_ON = false;
}