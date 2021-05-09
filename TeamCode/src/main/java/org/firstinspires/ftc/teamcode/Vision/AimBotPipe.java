package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.atan2;
import static java.lang.Math.tan;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.abs;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.pow;
import static java.lang.StrictMath.sin;
import static java.lang.StrictMath.sqrt;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.MAX_H;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.MAX_S;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.MAX_V;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.MIN_H;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.MIN_S;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.MIN_V;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.blur;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.dilate_const;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.erode_const;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_GoalFinder.goalWidth;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PS_LEFT_DIST;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PS_MIDDLE_DIST;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PS_RIGHT_DIST;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.RECT_OPTION.AREA;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.pixels2Degrees;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.sortRectsByMaxOption;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.line;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;

public class AimBotPipe extends OpenCvPipeline {
    private boolean viewportPaused;

    private double goalDistance;
    private double goalDegreeError = 0;
    private boolean goalFound = false;
    private Rect goalRect = new Rect(0, 0, 0, 0);

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours;

    // Thresholding values
    Scalar MIN_HSV, MAX_HSV;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    @Override
    public Mat processFrame(Mat input) {

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Copy to output
        input.copyTo(output);

        // Convert & Copy to outPut image
        cvtColor(input, modified, Imgproc.COLOR_RGB2HSV);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // Thresholding
        MIN_HSV = new Scalar(MIN_H, MIN_S, MIN_V);
        MAX_HSV = new Scalar(MAX_H, MAX_S, MAX_V);
        inRange(modified, MIN_HSV, MAX_HSV, modified);

        // Erosion and Dilation
        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        // Find contours of goal
        contours = new ArrayList<>();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Check if no goal is found
        if (contours.size() == 0) {
            goalFound = false;
            goalDegreeError = 0;
            goalDistance = 0;
            return output;
        }
        goalFound = true;

        // Retrieve all rects
        List<Rect> rects = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        // Retrieve goal contours and make into one large rectangle
        List<Rect> largest_rects = sortRectsByMaxOption(2, AREA, rects);
        goalRect = mergeRects(largest_rects);

        // Calculate Center
        int center_x = goalRect.x + (goalRect.width / 2);
        int center_y = goalRect.y + (goalRect.height / 2);
        Point center = new Point(center_x, center_y);

        // Calculate Error
        double pixel_error = (IMG_WIDTH / 2) - center_x;
        goalDegreeError = pixels2Degrees(pixel_error, VisionUtils.AXES.X) + 4;
        goalDistance = getGoalDistance();

        // Logging Shapes and Degree & Pixel Data
        rectangle(output, goalRect, color, thickness);
        line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);
        Point text_center = new Point(5, IMG_HEIGHT - 50);
        putText(output, "Degree Error: " + goalDegreeError, text_center, font, 0.4, new Scalar(255, 255, 0));
        putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));


        /*
        // Release all captures
        input.release();
        releaseAllCaptures();
         */

        // Return altered image
        return output;

    }


    public double getPowerShotDegreeError(VisionUtils.PowerShot powerShot, double curAngle){
        if (!isGoalFound()) return 0;
        double g = goalDistance;
        double alpha = (curAngle % 360);
        double angle2Turn2Goal = (alpha + goalDegreeError);
        double theta = toRadians(angle2Turn2Goal - 180);
        double x = g * cos(theta);
        double y = g * sin(theta);
        double d;
        switch (powerShot) {
            case PS_LEFT:
                d = PS_LEFT_DIST - y;
                break;
            case PS_MIDDLE:
                d = PS_MIDDLE_DIST - y;
                break;
            case PS_RIGHT:
                d = PS_RIGHT_DIST - y;
                break;
            default:
                d = 0;
        }
        // Note: gamma < 0 always
        double gamma = toDegrees(atan2(d, x));
        double powerShotFieldAngle = 180 - gamma;

        return powerShotFieldAngle;
    }

    public double calcRPM(){
        if (isGoalFound() && goalDistance > 1.9){
            double numerator = 9.8 * pow(goalDistance, 3);
            double denominator = (0.79 * goalDistance) - 1.185;
            return (denominator == 0) ? 0 : 250 * sqrt(numerator / denominator);
        }
        return 3400;
    }

    public boolean isGoalFound(){
        return goalFound;
    }

    public double getGoalDegreeError(){
        return (isGoalFound()) ? goalDegreeError : 0;
    }

    public double getGoalDistance(){
        if (!isGoalFound() || goalRect.y == 0) return 0;
        double opp = 240 - goalRect.y + 10;
        double thetaRads = opp / 240 * 0.75;
        return (90 / tan(thetaRads) + 20) / 100;
    }

    public Rect getGoalRect(){
        return  goalRect;
    }

    private Rect mergeRects(List<Rect> rects) {

        // Return first contour if there is only one
        Rect goalRect = rects.get(0);

        // Extrapolate overarching rectangle if there are two
        if (rects.size() == 2) {

            // Init coords of both rectangles
            Rect left = new Rect(0, 0, 0, 0);
            Rect right = new Rect(0, 0, 0, 0);

            // Get bounding rects of second rectangle
            Rect secondRect = rects.get(1);

            // Check second rect is within goal width
            int diff = abs(goalRect.x - secondRect.x);
            if (diff > goalWidth) return goalRect;

            // Check which side rectangles are on, and calculate surrounding box
            if (goalRect.x < secondRect.x) {
                left.x = goalRect.x;
                left.y = goalRect.y;
                right.x = secondRect.x;
                right.y = secondRect.y;
                right.width = secondRect.width;
                right.height = secondRect.height;
            } else {
                left.x = secondRect.x;
                left.y = secondRect.y;
                right.x = goalRect.x;
                right.y = goalRect.y;
                right.width = goalRect.width;
                right.height = goalRect.height;
            }
            goalRect.x = left.x;
            goalRect.y = left.y;
            goalRect.width = abs(right.x - left.x) + right.width;
            goalRect.height = abs(right.y - left.y) + right.height;
        }

        return goalRect;
    }

    public void releaseAllCaptures(){
        modified.release();
        hierarchy.release();
        if (contours != null){
            for (MatOfPoint cnt : contours){
                cnt.release();
            }
        }
    }

    @Override
    public void onViewportTapped() {
        viewportPaused =        !viewportPaused;
        if (viewportPaused)     VisionUtils.webcam_front.pauseViewport();
        else                    VisionUtils.webcam_front.resumeViewport();
    }
}