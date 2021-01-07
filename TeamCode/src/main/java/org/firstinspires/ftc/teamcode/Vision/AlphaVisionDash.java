package org.firstinspires.ftc.teamcode.Vision;


import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="AlphaVisionDash", group="TeleOp Linear Opmode")
public class AlphaVisionDash extends LinearOpMode
{
    private MecanumRobot mecanumRobot;

    OpenCvCamera webcam;
    private static double ringCount = 0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private MultipleTelemetry multTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    public void initialize(){
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
    }

    @Override
    public void runOpMode()
    {

        //initialize();


        /*
        Set up camera, and pipeline
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(new RingDetectingPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });



        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();


        /*
        ACTION
         */
        while (opModeIsActive())
        {

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

            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.update();

            if(gamepad1.a)
            {
                webcam.stopStreaming();
            }
            sleep(100);
        }
    }

    class RingDetectingPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        // Init mats here so we don't repeat
        Mat YCbCr = new Mat();
        Mat outPut = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();

        // Rectangles starting coordinates      // Rectangles starting percentages
        int rectTopX1; int rectTopX2;           //double rectTopX1Percent = 0; double rectTopX2Percent = 0;
        int rectTopY1; int rectTopY2;           //double rectTopY1Percent = 0; double rectTopY2Percent = 0;

        // Rectangles starting coordinates      // Rectangles starting percentages
        int rectBottomX1; int rectBottomX2;     //double rectBottomX1Percent = 0; double rectBottomX2Percent = 0;
        int rectBottomY1; int rectBottomY2;     //double rectBottomY1Percent = 0; double rectBottomY2Percent = 0;


        @Override
        public Mat processFrame(Mat input)
        {
            // Convert & Copy to outPut image
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            input.copyTo(outPut);

            /*
            Imgproc.rectangle(
                    outPut,
                    new Point(
                            (int) outPut.cols() * DashConstants.rectTopX1Percent,
                            (int) outPut.rows() * DashConstants.rectTopY1Percent
                    ),
                    new Point(
                            (int) outPut.cols() * DashConstants.rectTopX2Percent,
                            (int) outPut.rows() * DashConstants.rectTopY2Percent
                    ),
                    new Scalar(0, 255, 0), 4);
            Imgproc.rectangle(
                    outPut,
                    new Point(
                            (int) outPut.cols() * DashConstants.rectBottomX1Percent,
                            (int) outPut.rows() * DashConstants.rectBottomY1Percent
                    ),
                    new Point(
                            (int) outPut.cols() * DashConstants.rectBottomX2Percent,
                            (int) outPut.rows() * DashConstants.rectBottomY2Percent
                    ),
                    new Scalar(0, 255, 0), 4);
            */
/*
            // Dimensions for top rectangle
            rectTopX1 = (int) (YCbCr.rows() * DashConstants.rectTopX1Percent);
            rectTopX2 = (int) (YCbCr.rows() * DashConstants.rectTopX2Percent);
            rectTopY1 = (int) (YCbCr.cols() * DashConstants.rectTopY1Percent);
            rectTopY2 = (int) (YCbCr.cols() * DashConstants.rectTopY2Percent);

            // Dimensions for bottom rectangle
            rectBottomX1 = (int) (YCbCr.rows() * DashConstants.rectBottomX1Percent);
            rectBottomX2 = (int) (YCbCr.rows() * DashConstants.rectBottomX2Percent);
            rectBottomY1 = (int) (YCbCr.cols() * DashConstants.rectBottomY1Percent);
            rectBottomY2 = (int) (YCbCr.cols() * DashConstants.rectBottomY2Percent);
*/
            // Dimensions for top rectangle
            rectTopX1 = (int) (input.rows() * DashConstants.rectTopX1Percent);
            rectTopX2 = (int) (input.rows() * DashConstants.rectTopX2Percent) - rectTopX1;
            rectTopY1 = (int) (input.cols() * DashConstants.rectTopY1Percent);
            rectTopY2 = (int) (input.cols() * DashConstants.rectTopY2Percent) - rectTopY1;

            // Dimensions for bottom rectangle
            rectBottomX1 = (int) (input.rows() * DashConstants.rectBottomX1Percent);
            rectBottomX2 = (int) (input.rows() * DashConstants.rectBottomX2Percent) - rectBottomX1;
            rectBottomY1 = (int) (input.cols() * DashConstants.rectBottomY1Percent);
            rectBottomY2 = (int) (input.cols() * DashConstants.rectBottomY2Percent) - rectBottomY1;

            // VISUALIZATION: Create rectangles and scalars, then draw them onto outPut
            Scalar rectangleColor = new Scalar(0, 0, 255);
            Rect rectTop = new Rect(rectTopX1, rectTopY1, rectTopX2, rectTopY2);
            Rect rectBottom = new Rect(rectBottomX1, rectBottomY1, rectBottomX2, rectBottomY2);
            Imgproc.rectangle(outPut, rectTop, rectangleColor, 2);
            Imgproc.rectangle(outPut, rectBottom, rectangleColor, 2);




           // IDENTIFY RINGS //

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




            /**
             * RECT_BOTTOM_X1: 0.75
             * RECT_BOTTOM_X2: 0.9
             * RECT_BOTTOM_Y1: 0.38
             * RECT_BOTTOM_Y2: 0.42
             * RECT_TOP_X1: 0.75
             * RECT_TOP_X2: 0.9
             * RECT_TOP_Y1: 0.3
             * RECT_TOP_Y2: 0.38
             * Given a distance of around 3ft from rings
             */

            multTelemetry.addData("RECT_TOP_X1", DashConstants.rectTopX1Percent);
            multTelemetry.addData("RECT_TOP_Y1", DashConstants.rectTopY1Percent);
            multTelemetry.addData("RECT_TOP_X2", DashConstants.rectTopX2Percent);
            multTelemetry.addData("RECT_TOP_Y2", DashConstants.rectTopY2Percent);
            multTelemetry.addData("RECT_BOTTOM_X1", DashConstants.rectBottomX1Percent);
            multTelemetry.addData("RECT_BOTTOM_Y1", DashConstants.rectBottomY1Percent);
            multTelemetry.addData("RECT_BOTTOM_X2", DashConstants.rectBottomX2Percent);
            multTelemetry.addData("RECT_BOTTOM_Y2", DashConstants.rectBottomY2Percent);


            multTelemetry.addData("Upper", upperCrop);
            multTelemetry.addData("Lower", lowerCrop);
            multTelemetry.addData("lowerAveOrange", lowerAveOrange);
            multTelemetry.addData("upperAveOrange", upperAveOrange);
            multTelemetry.addData("finalLowerAve: ", finalLowerAve);
            multTelemetry.addData("finalUpperAve: ", finalUpperAve);
            multTelemetry.addData("RingCount: ", ringCount);

            multTelemetry.update();

            // Return altered image
            return outPut;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}