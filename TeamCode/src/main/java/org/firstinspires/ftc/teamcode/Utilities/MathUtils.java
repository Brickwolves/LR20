package org.firstinspires.ftc.teamcode.Utilities;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.Navigation.Point;

import static java.lang.Math.floorMod;
import static java.lang.Math.toDegrees;
import static java.lang.StrictMath.abs;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.pow;
import static java.lang.StrictMath.sin;
import static java.lang.StrictMath.toRadians;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.angleMode.RADIANS;

public class MathUtils {

    public enum angleMode {
        RADIANS, DEGREES
    }

    /**
     * @param x
     * @param a_min
     * @param a_max
     * @param b_min
     * @param b_max
     * @return x but mapped from [a_min, a_max] to [b_min, b_max]
     */
    public static double map(double x, double a_min, double a_max, double b_min, double b_max){
        return (x - a_min) / (a_max - a_min) * (b_max - b_min) + b_min;
    }

    /*
                        T R I G    E Q U A T I O N S
                                                                        */

    public static double cos(double value, angleMode mode){
        return (mode == RADIANS) ? Math.cos(value) : toDegrees(Math.cos(value));
    }
    public static double sin(double value, angleMode mode){
        return (mode == RADIANS) ? Math.sin(value) : toDegrees(Math.sin(value));
    }
    public static double tan(double value, angleMode mode){
        return (mode == RADIANS) ? Math.tan(value) : toDegrees(Math.tan(value));
    }



    /**
     * @param x
     * @param y
     * @param shiftAngle
     * @return (x, y) coordinate is mapped relative to robot's current heading
     */
    public static Point shift(double x, double y, double shiftAngle){
        double r = toRadians(shiftAngle);
        double shiftedX = (x * Math.sin(r)) + (y * Math.cos(r));
        double shiftedY = (x * Math.cos(r)) - (y * Math.sin(r));
        return new Point(shiftedX, shiftedY);
    }

    /**
     * @param x
     * @param y
     * @param shiftAngle
     * @return (x, y) relative robot coordinate mapped to field
     */
    public static Point unShift(double x, double y, double shiftAngle){
        double r = toRadians(shiftAngle);
        double unShiftedY = ((x * Math.cos(r)) - (y * Math.sin(r)))   /  (pow(Math.cos(r), 2) + pow(Math.sin(r), 2));
        double unShiftedX = (x - (unShiftedY * Math.cos(r))) / Math.sin(r);
        if (Math.sin(r) == 0) unShiftedX = 0;
        return new Point(unShiftedX, unShiftedY);
    }



    /**
     * @param targetAngle
     * @param currentAngle
     * @return the closest relative target angle
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double findClosestAngle(double targetAngle, double currentAngle) {
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return abs(simpleTargetDelta) <= abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }


}
