package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.floorMod;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class OpModeUtils {

    public static HardwareMap hardwareMap;
    public static OpMode opMode;

    public static Telemetry telemetry;
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static MultipleTelemetry multTelemetry;


    private static boolean isLinearOpMode;

    // Only use if it is in fact a LinearOpMode
    public static LinearOpMode linearOpMode = null;

    /**
     * Sets the OpMode
     * @param opMode
     */
    public static void setOpMode(OpMode opMode) {
        OpModeUtils.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        multTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        isLinearOpMode = (opMode instanceof LinearOpMode);
        if (isLinearOpMode) {
            linearOpMode = (LinearOpMode) opMode;
        }
    }


    public static boolean isActive(){
        if (isLinearOpMode) return linearOpMode.opModeIsActive();
        return true;
    }

    /**
     * I'm lazy
     * @param str
     */
    public static void print(String str){
        System.out.println(str);
    }

    public static double lessThan1000TicksToCentimeters(double ticks){
        return (0.0748 * pow(ticks, 2)) + (.677 * ticks) + 87.3;
    }


    public static double centimeters2Ticks(double c){
        return (0.118 * pow(c, 2)) + (3.66 * c) + 7;
    }

    public static double ticks2Centimeters(double ticks){
        double rootTerm = 4 * 0.118 * (7 - ticks);
        if (rootTerm < 0) return 0;

        double numer = sqrt(pow(3.66, 2) - rootTerm);
        double denom = 2 * 0.118;
        double answ1 = (-3.66 + numer) / denom;
        double answ2 = (-3.66 - numer) / denom;
        return (answ1 > 0) ? answ1 : answ2;
    }

    public static double convertInches2Ticks(double ticks){
        return (ticks - 4.38) / 0.0207; // Calculated using desmos
    }

    public static double convertTicks2Inches(double inches){
        return (0.0207 * inches) + 4.38; // Calculated using desmos
    }

    public static double map(double x, double a_min, double a_max, double b_min, double b_max){
        return (x - a_min) / (a_max - a_min) * (b_max - b_min) + b_min;
    }
}