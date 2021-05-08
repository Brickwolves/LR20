package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

public class VisionUtils {

    public static OpenCvCamera webcam_front;
    public static OpenCvCamera webcam_back;
    public static double IMG_WIDTH = 320;
    public static double IMG_HEIGHT = 240;
    public static final double X_FOV = 72;
    public static final double Y_FOV = 43;
    public static double PS_LEFT_DIST = 0.415;
    public static double PS_MIDDLE_DIST = 0.64;
    public static double PS_RIGHT_DIST = 0.80;
    public static final double CAMERA_HEIGHT = 0.19;
    public static final double BACK_WEBCAM_HEIGHT = 0.19;
    // Note: All measurements in meters

    public enum Target {
        GOAL, POWERSHOTS
    }

    public enum PowerShot {
        PS_LEFT, PS_MIDDLE, PS_RIGHT
    }

    public static enum RECT_OPTION {
        AREA, WIDTH, HEIGHT, X, Y
    }
    public static enum AXES {
        X, Y
    }

    public static double pixels2Degrees(double pixels, AXES axe) {
        return (axe == AXES.X) ? pixels * (X_FOV / IMG_WIDTH) : pixels * (Y_FOV / IMG_HEIGHT);
    }

    public static int getMaxIndex(List<Rect> rects, RECT_OPTION option){
        int alpha_index = 0;
        double max = Integer.MIN_VALUE;
        double cur = 0;
        for (int i=0; i < rects.size(); i++){

            switch (option){
                case X:
                    cur = rects.get(i).x;
                    break;

                case Y:
                    cur = rects.get(i).y;
                    break;

                case WIDTH:
                    cur = rects.get(i).width;
                    break;

                case HEIGHT:
                    cur = rects.get(i).height;
                    break;

                case AREA:
                    cur = rects.get(i).width * rects.get(i).height;
                    break;

            }
            if (cur > max) {
                max = cur;
                alpha_index = i;
            }
        }
        return alpha_index;
    }

    public static int getMinIndex(List<Rect> rects, RECT_OPTION option){
        int beta_index = 0;
        double min = Integer.MAX_VALUE;
        double cur = 0;
        for (int i=0; i < rects.size(); i++){

            switch (option){
                case X:
                    cur = rects.get(i).x;
                    break;

                case Y:
                    cur = rects.get(i).y;
                    break;

                case WIDTH:
                    cur = rects.get(i).width;
                    break;

                case HEIGHT:
                    cur = rects.get(i).height;
                    break;

                case AREA:
                    cur = rects.get(i).width * rects.get(i).height;
                    break;
            }
            if (cur < min) {
                min = cur;
                beta_index = i;
            }
        }
        return beta_index;
    }


    public static List<Rect> sortRectsByMinOption(int n, RECT_OPTION option, List<Rect> rects){
        List<Rect> sorted_rects = new ArrayList<>();
        for (int j=0; j < n; j++){
            int beta_index = getMinIndex(rects, option);
            sorted_rects.add(rects.get(beta_index));

            rects.remove(beta_index);
            if (rects.size() == 0) break;
        }
        return sorted_rects;
    }

    public static List<Rect> sortRectsByMaxOption(int n, RECT_OPTION option, List<Rect> rects){
        List<Rect> sorted_rects = new ArrayList<>();
        for (int j=0; j < n; j++){
            int alpha_i = getMaxIndex(rects, option);
            sorted_rects.add(rects.get(alpha_i));

            rects.remove(alpha_i);
            if (rects.size() == 0) break;
        }
        return sorted_rects;
    }
}
