package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Utils {

    private static HardwareMap hardwareMap;

    public static HardwareMap getHardwareMap(){
        return hardwareMap;
    }

    public static void setHardwareMap(HardwareMap hardwareMap){
        Utils.hardwareMap = hardwareMap;
    }

    /**
     * @param baseRGB
     * @param currentRGB
     * @return
     */
    public static double distance2Color(double[] baseRGB, double[] currentRGB){
        return Math.sqrt(Math.pow(baseRGB[0] - currentRGB[0], 2) + Math.pow(baseRGB[1] - currentRGB[1], 2) + Math.pow(baseRGB[2] - currentRGB[2], 2));
    }


}
