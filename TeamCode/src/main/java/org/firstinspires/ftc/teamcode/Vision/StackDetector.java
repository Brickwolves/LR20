package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

public class StackDetector extends OpMode {

    OpenCvCamera phoneCam = null;
    static double ringCount = 0;

    @Override
    public void init(){

    }

    @Override
    public void init_loop(){

        // Init the phone
        int cameraMonitorViewId = Utils.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", Utils.hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // Pipeline is just a section of code that will be passed the image to then be processed
        phoneCam.setPipeline(new RingDetectingPipeline());

        // Start Streaming
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    @Override
    public void loop() {

        if (ringCount == 1.0){
            // drive somewhere
        }
        else if (ringCount == 4.0){
            // drive somewhere else
        }
        else {
            // drive somewhere other than else
        }

    }

    class RingDetectingPipeline extends OpenCvPipeline {

        // Init mats here so we don't repeat
        Mat YCbCr = new Mat();
        Mat outPut = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();

        // Rectangles starting coordinates
        int rectTopX1; int rectTopX2;
        int rectTopY1; int rectTopY2;

        // Rectangles starting coordinates
        int rectBottomX1; int rectBottomX2;
        int rectBottomY1; int rectBottomY2;

        final double[] ORANGE = {};

        @Override
        public Mat processFrame(Mat input) {

            // Convert
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            input.copyTo(outPut);


            // Dimensions for top rectangle
            rectTopX1 = (int) Math.round(YCbCr.rows() * .18);
            rectTopX2 = (int) Math.round(YCbCr.cols() * );
            rectTopY1 = (int) Math.round(YCbCr.cols() * );
            rectTopY2 = (int) Math.round(YCbCr.cols() * );

            // Dimensions for bottom rectangle
            rectBottomX1 = (int) Math.round(YCbCr.rows() * .18);
            rectBottomX2 = (int) Math.round(YCbCr.cols() * );
            rectBottomY1 = (int) Math.round(YCbCr.cols() * );
            rectBottomY2 = (int) Math.round(YCbCr.cols() * );

            // VISUALIZATION: Create rectangles and scalars
            Scalar rectangleColor = new Scalar(0, 0, 255);
            Rect rectTop = new Rect(rectTopX1, rectTopY2, rectTopX2, rectTopY2);
            Rect rectBottom = new Rect(rectBottomX1, rectBottomY1, rectBottomX2, rectBottomY2);
            Imgproc.rectangle(outPut, rectTop, rectangleColor, 2);
            Imgproc.rectangle(outPut, rectBottom, rectangleColor, 2);


            /*
            ACTUAL VISION DETECTION -------------------------------------------------
                - Crop
                - Extract Cb Channel
                - Store ave of each Cb color
                - Compare ave to predefined values
             */

            // Crop
            upperCrop = YCbCr.submat(rectTop);
            lowerCrop = YCbCr.submat(rectBottom);

            // Extract Channels [Y, Cr, Cb], where 2 = index of Cb channel
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            Core.extractChannel(upperCrop, upperCrop, 2);

            // Store Averages
            Scalar lowerAveOrange = Core.mean(lowerCrop);
            Scalar upperAveOrange = Core.mean(upperCrop);
            double finalLowerAve = lowerAveOrange.val[0];
            double finalUpperAve = upperAveOrange.val[0];


            // Check bounds
            if (
                    finalLowerAve > 15 &&
                    finalLowerAve < 130 &&
                    finalUpperAve < 130

            ) ringCount = 4.0;
            else if (
                    finalLowerAve > 10 &&
                    finalUpperAve < 15 &&
                    finalLowerAve > 10 &&
                    finalUpperAve < 15
            ) ringCount = 0.0;
            else ringCount = 1.0;

            // Return altered image
            return outPut;
        }
    }

}
