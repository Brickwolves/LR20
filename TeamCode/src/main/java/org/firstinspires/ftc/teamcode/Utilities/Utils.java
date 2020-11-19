package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Utils {

    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;

    public static void setHardwareMap(HardwareMap hardwareMap){ Utils.hardwareMap = hardwareMap; }
    public static void setTelemetry(Telemetry telemetry) { Utils.telemetry = telemetry; }

    /**
     * Super simple method to check toggles on buttons
     * @param current
     * @param previous
     * @return
     */
    public static Boolean buttonTapped(boolean current, boolean previous){
        if (current && !previous )return true;
        else if (!current) return false;
        else return previous;
    }


    /**
     * @param baseRGB
     * @param currentRGB
     * @return
     */
    public static double distance2Color(double[] baseRGB, double[] currentRGB){
        return Math.sqrt(Math.pow(baseRGB[0] - currentRGB[0], 2) + Math.pow(baseRGB[1] - currentRGB[1], 2) + Math.pow(baseRGB[2] - currentRGB[2], 2));
    }

    /**
     * @param angle
     * @return coTermAngle
     */
    public static double coTerminal(double angle){
        double coTermAngle = (angle + 180) % 360;
        coTermAngle -= 180;
        return coTermAngle;
    }
}
