package org.firstinspires.ftc.teamcode.DashConstants;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class Dash_AimBot {

    // GOAL FINDER
    public static boolean BLUE_ALLIANCE = false;
    public static Scalar MAX_COLORSET = new Scalar(0, 0, 0);
    public static Scalar MIN_COLORSET = new Scalar(0, 0, 0);
    public static int[] BLUE_MARGINS = {25, 90, 50};
    public static int[] RED_MARGINS = {25, 25, 25};

    public static int blur = 5;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static int goalWidth = 100;
    public static double horizonLineRatio = 1;

    public static double PS_RIGHT_OFFSET    = 2;
    public static double PS_MIDDLE_OFFSET   = 4;
    public static double PS_LEFT_OFFSET     = 3;
    public static double xVelocityMultiplier  = 0.07; // 0.15

    // Debugging tools
    public static boolean DEBUG_MODE_ON     = true;
    public static boolean INIT_COMPLETED = true;
    public static int INIT_RECT_SIDELENGTH = 10;

}
