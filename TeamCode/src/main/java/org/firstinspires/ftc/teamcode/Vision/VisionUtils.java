package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

public class VisionUtils {

    public static OpenCvCamera webcam;
    public static double IMG_WIDTH = 320;
    public static double IMG_HEIGHT = 240;
    public static final double FOV = 72;
    public static final double FOCAL_LENGTH = 1;
    public static final double RING_HEIGHT = 20;
    public static final double SENSOR_HEIGHT = 1;

    public static double getDistance2Object(double object_pixel_height, double object_height) {
        if (object_pixel_height == 0) return 0;
        return (FOCAL_LENGTH * object_height * IMG_HEIGHT) / (object_pixel_height * SENSOR_HEIGHT);
    }

    public static double pixels2Degrees(double pixels) {
        return pixels * (FOV / IMG_WIDTH);
    }

    public static enum RECT_OPTION {
        AREA, WIDTH, HEIGHT, X, Y
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
