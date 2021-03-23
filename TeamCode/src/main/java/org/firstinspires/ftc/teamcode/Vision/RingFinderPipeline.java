package org.firstinspires.ftc.teamcode.Vision;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import static java.lang.StrictMath.abs;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_RingFinder.*;
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
import java.util.ArrayList;
import java.util.List;

public class RingFinderPipeline extends OpenCvPipeline
{
    private boolean viewportPaused;

    // Constants
    private int IMG_WIDTH = 0;
    private int IMG_HEIGHT = 0;
    private int FOV = 72;
    private int ring_count = 0;
    private double error = 0;

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();

    // Thresholding values
    Scalar MIN_YCrCb, MAX_YCrCb;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    @Override
    public Mat processFrame(Mat input)
    {
        // Get image dimensions
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Copy to output
        input.copyTo(output);

        // Convert & Copy to outPut image
        cvtColor(input, modified, Imgproc.COLOR_RGB2YCrCb);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // Thresholding
        MIN_YCrCb = new Scalar(MIN_Y, MIN_Cr, MIN_Cb);
        MAX_YCrCb = new Scalar(MAX_Y, MAX_Cr, MAX_Cb);
        inRange(modified, MIN_YCrCb, MAX_YCrCb, modified);

        // Erosion and Dilation
        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Check if we have detected any orange objects and assume ring_count is 0
        ring_count = 0;
        if (contours.size() > 0) {

            // Retrieve widest (closest) rect
            MatOfPoint widest_contour = findNWidestContours(1, contours).get(0);
            Rect widest_rect = boundingRect(widest_contour);

            // Check if it is below horizon line
            if (widest_rect.y < IMG_HEIGHT * horizonLineRatio) return output;

            // Calculate error
            int center_x = widest_rect.x + (widest_rect.width / 2);
            int center_y = widest_rect.y + (widest_rect.height / 2);
            Point center = new Point(center_x, center_y);
            int pixel_error = (IMG_WIDTH / 2) - center_x;
            error = pixels2Degrees(pixel_error);
            line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);

            // Log center
            String coords = "(" + center_x + ", " + center_y + ")";
            putText(output, coords, center, font, 0.5, color);

            // Update ring count
            ring_count = (widest_rect.height < (0.5 * widest_rect.width)) ? 1 : 4;
        }

        // Return altered image
        return output;
    }

    public double pixels2Degrees(double pixels){
        if (IMG_WIDTH == 0) return 0;
        return pixels * (FOV / IMG_WIDTH);
    }

    public int getRingCount(){
        return ring_count;
    }

    public int findWidestContourIndex(List<MatOfPoint> contours){
        int index = 0;
        double maxWidth = 0;
        for (int i=0; i < contours.size(); i++){
            MatOfPoint cnt = contours.get(i);
            double width = boundingRect(cnt).width;
            if (width > maxWidth) {
                maxWidth = width;
                index = i;
            }
        }
        return index;
    }

    public List<MatOfPoint> findNWidestContours(int n, List<MatOfPoint> contours){
        List<MatOfPoint> widest_contours = new ArrayList<>();
        for (int j=0; j < n; j++){
            int largest_index = findWidestContourIndex(contours);
            widest_contours.add(contours.get(largest_index));

            contours.remove(largest_index);
            if (contours.size() == 0) break;
        }
        return widest_contours;
    }

    @Override
    public void onViewportTapped()
    {
        /*
        viewportPaused = !viewportPaused;
        if(viewportPaused)  webcam.pauseViewport();
        else                webcam.resumeViewport();
         */
    }
}