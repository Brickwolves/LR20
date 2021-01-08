package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import static android.os.SystemClock.sleep;

@Autonomous(name = "WebCamStackDetectorAuto", group = "Autonomous")
@Disabled
public class WebCamStackDetector extends OpMode {

    OpenCvCamera webCam = null;
    static double ringCount = 0;

    @Override
    public void init(){
        Utils.setOpMode(this);
    }

    @Override
    public void init_loop(){

        // Init the phone
        int cameraMonitorViewId = Utils.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(Utils.hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webCam.setPipeline(new RingDetectingPipeline());
        // Start Streaming
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    @Override
    public void loop() {

        if (ringCount == 1.0){
            // drive somewhere
            Utils.telemetry.addData("RingCount", 1.0);
        }
        else if (ringCount == 4.0){
            // drive somewhere else
            Utils.telemetry.addData("RingCount", 4.0);
        }
        else {
            // drive somewhere other than else
            Utils.telemetry.addData("RingCount", 0.0);
        }

    }

    class RingDetectingPipeline extends OpenCvPipeline {

        // Init mats here so we don't repeat
        Mat YCbCr = new Mat();
        Mat outPut = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();

        // Rectangles starting coordinates      // Rectangles starting percentages
        int rectTopX1; int rectTopX2;           double rectTopX1Percent = 0; double rectTopX2Percent = 0;
        int rectTopY1; int rectTopY2;           double rectTopY1Percent = 0; double rectTopY2Percent = 0;

        // Rectangles starting coordinates      // Rectangles starting percentages
        int rectBottomX1; int rectBottomX2;     double rectBottomX1Percent = 0; double rectBottomX2Percent = 0;
        int rectBottomY1; int rectBottomY2;     double rectBottomY1Percent = 0; double rectBottomY2Percent = 0;

        final double[] ORANGE = {  };

        @Override
        public Mat processFrame(Mat input) {

            // Convert
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            input.copyTo(outPut);

            // Dimensions for top rectangle
            rectTopX1 = (int) Math.round(YCbCr.rows() * rectTopX1Percent);
            rectTopX2 = (int) Math.round(YCbCr.rows() * rectTopX2Percent);
            rectTopY1 = (int) Math.round(YCbCr.cols() * rectTopY1Percent);
            rectTopY2 = (int) Math.round(YCbCr.cols() * rectTopY2Percent);

            // Dimensions for bottom rectangle
            rectBottomX1 = (int) Math.round(YCbCr.rows() * rectBottomX1Percent);
            rectBottomX2 = (int) Math.round(YCbCr.rows() * rectBottomX2Percent);
            rectBottomY1 = (int) Math.round(YCbCr.cols() * rectBottomY1Percent);
            rectBottomY2 = (int) Math.round(YCbCr.cols() * rectBottomY2Percent);

            // VISUALIZATION: Create rectangles and scalars, then draw them onto outPut
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
