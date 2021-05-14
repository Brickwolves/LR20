package org.firstinspires.ftc.teamcode.DashConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_AimBot {

    // GOAL FINDER
    public static int MIN_H = 100;
    public static int MAX_H = 120;

    public static int MIN_S = 110;
    public static int MAX_S = 255;

    public static int MIN_V = 90;
    public static int MAX_V = 220;

    public static int blur = 5;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static int goalWidth = 100;
    public static double horizonLineRatio = 1;

    public static double PS_RIGHT_OFFSET    = 0;
    public static double PS_MIDDLE_OFFSET   = 0;
    public static double PS_LEFT_OFFSET     = 0;
    public static double xVelocityMultiplier  = 1000000;
    public static boolean DEBUG_MODE_ON     = false;

}