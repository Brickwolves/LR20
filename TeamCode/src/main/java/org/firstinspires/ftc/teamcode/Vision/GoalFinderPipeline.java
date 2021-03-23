package org.firstinspires.ftc.teamcode.Vision;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.NoSuchElementException;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision.MAX_H;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision.MAX_S;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision.MAX_V;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision.MIN_H;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision.MIN_S;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision.MIN_V;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_32F;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

class GoalFinderPipeline extends OpenCvPipeline
{
    private boolean viewportPaused;

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();

    // Thresholding values
    Scalar MIN_HSV, MAX_HSV;

    // Rectangle settings
    private Scalar color = new Scalar(0, 0, 255);
    private double thickness = 2;

    @Override
    public Mat processFrame(Mat input)
    {
        // Convert & Copy to outPut image
        Imgproc.cvtColor(input, modified, Imgproc.COLOR_RGB2HSV);
        input.copyTo(output);

        // Blurring
        GaussianBlur(input, input, new Size(35, 35), 0);

        // Thresholding
        MIN_HSV = new Scalar(MIN_H, MIN_S, MIN_V);
        MAX_HSV = new Scalar(MAX_H, MAX_S, MAX_V);
        inRange(input, MIN_HSV, MAX_HSV, input);

        // Erosion and Dilation
        erode(input, input, new Mat(5, 5, CV_32F));
        dilate(input, input, new Mat(5, 5, CV_32F));

        // Find contours of goal
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(input, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) return output;

        int outer_index = findLargestContour(contours);
        contours.remove(outer_index);
        int inner_index = findLargestContour(contours);

        int xo, yo, wo, ho;
        boundingRect(contours.get(outer_index));
        boundingRect(contours.get(inner_index));

        /*
        MatOfPoint goalCnt = contours
                .stream()
                .min(Comparator.comparing(Imgproc::contourArea))
                .orElseThrow(NoSuchElementException::new);
         */


        // VISUALIZATION: Create rectangles and scalars, then draw them onto outPut
        //Rect rectTop = new Rect(rectTopX1, rectTopY1, rectTopX2, rectTopY2);
        //Imgproc.rectangle(outPut, rectTop, rectangleColor, 2);




        // Return altered image
        return output;
    }

    public int findLargestContour(List<MatOfPoint> contours){
        int index = 0;
        double maxArea = 0;
        for (int i=0; i < contours.size(); i++){
            MatOfPoint cnt = contours.get(i);
            double area = contourArea(cnt);
            if (area > maxArea) {
                maxArea = area;
                index = i;
            }
        }
        return index;
    }

    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;
        /*
        if(viewportPaused)  webcam.pauseViewport();
        else                webcam.resumeViewport();
         */
    }
}
