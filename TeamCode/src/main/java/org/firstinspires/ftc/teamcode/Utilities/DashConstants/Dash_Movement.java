package org.firstinspires.ftc.teamcode.Utilities.DashConstants;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_Movement {
    /*
    TELEOP WORKS PERFECTLY
    public static double p = .1;
    public static double i = .00000001;
    public static double d = .005;
     */

    public static double turn_min = 0.00000001; // 10e-8
    public static double p = .08;
    public static double i = .0;
    public static double d = .003;
    public static double velocity = 1;
    public static double turn_offset = 5;

    public static double diagnostic_inches = 28;
    public static double diagnostic_turn1 = 270;
    public static double diagnostic_turn2 = 0;
    public static double MOE = 0.01;
}
