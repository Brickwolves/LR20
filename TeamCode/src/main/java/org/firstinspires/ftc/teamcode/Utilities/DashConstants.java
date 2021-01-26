package org.firstinspires.ftc.teamcode.Utilities;
import com.acmerobotics.dashboard.config.Config;

@Config
public class DashConstants {

    public static double p = .1;
    public static double i = .00000001;
    public static double d = .005;


    /* ------- VISION --------- */
    // Top rectangle starting percentages
    public static double rectTopX1Percent = 0.62; public static double rectTopX2Percent = 0.84;
    public static double rectTopY1Percent = 0.5; public static double rectTopY2Percent = .58;

    // Bottom rectangle starting percentages
    public static double rectBottomX1Percent = 0.62; public static double rectBottomX2Percent = 0.84;
    public static double rectBottomY1Percent = 0.58; public static double rectBottomY2Percent = 0.62;

    // Generally, orange should be around 90-100
    public static double orangeMax = 110;
    public static double orangeMin = 80;

    public static double power;

    public static double diagnosticInches = 28;
    public static double velocity = 1;


    public static double learning_rate = 0.01;

    public static double diagnostic_ring_count = 0;

    public static double servo_max = 0.0;
    public static double servo_min = 0.0;
    public static double servo_home = 0.0;




}
