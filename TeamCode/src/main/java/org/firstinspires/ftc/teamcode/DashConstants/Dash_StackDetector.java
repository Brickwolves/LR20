package org.firstinspires.ftc.teamcode.DashConstants;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_StackDetector {
    /* ------- VISION --------- */
    // Top rectangle starting percentages
    public static double rectTopX1Percent = 1; public static double rectTopX2Percent = 1.2;
    public static double rectTopY1Percent = 0.48; public static double rectTopY2Percent = 0.53;

    // Bottom rectangle starting percentages
    public static double rectBottomX1Percent = 1; public static double rectBottomX2Percent = 1.2;
    public static double rectBottomY1Percent = 0.57; public static double rectBottomY2Percent = 0.6;

    // Generally, orange should be around 90-100
    public static double orangeMax = 115;
    public static double orangeMin = 80;

    public static int ring_count = 1;
}
