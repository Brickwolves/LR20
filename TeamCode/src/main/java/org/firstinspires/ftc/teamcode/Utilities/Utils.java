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

}
