package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.ps_rpm;

@Autonomous(name="MarkIV - Scrimmage", group="Autonomous Linear Opmode")
public class MarkIV extends LinearOpMode
{

    private MecanumRobot robot;

    OpenCvCamera webcam;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private MultipleTelemetry multTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);


    public static double ringCount = 0.0;
    private boolean ringsFound = false;

    public void initialize(){
        Utils.setOpMode(this);
        robot = new MecanumRobot();
    }


    public void shoot(int rings){
        boolean shooter_on = true;
        boolean reached_RPM = false;
        double shoot_time = Dash_Shooter.millis * rings;
        ElapsedTime t = new ElapsedTime();

        while (shooter_on){
            robot.shooter.setRPM(ps_rpm);
            double rpm_error = Math.abs(ps_rpm - robot.shooter.getRPM());
            if (rpm_error < 500){
                reached_RPM = true;
                t.reset();
            }
            if (reached_RPM && t.milliseconds() < shoot_time){
                robot.shooter.feederState(true);
            }
            else if (t.milliseconds() >= shoot_time) shooter_on = false;
        }

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void powerShot(double startAngle){
        startAngle = 82;
        for (int i=0; i < 3; i++){
            robot.turn(startAngle * (7 + i), 0.01);
            shoot(1);
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();


        /*
        Set up camera, and pipeline

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
        */


        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();


        /*
        ACTION
         */

        robot.strafe(-26, 60, 90, 0.1, null);
        powerShot(82);

/*        double startAngle = 82;

        while (robot.shooter.getRPM() < ps_rpm){
            robot.shooter.setRPM(ps_rpm);
            multTelemetry.addData("RPM", robot.shooter.getRPM());
            multTelemetry.update();
        };
        robot.turn(startAngle, 0.01);
        shoot(Dash_Shooter.millis);

        while (robot.shooter.getRPM() < ps_rpm){
            robot.shooter.setRPM(ps_rpm);
            multTelemetry.addData("RPM", robot.shooter.getRPM());
            multTelemetry.update();
        };
        robot.turn(startAngle + 7, 0.01);
        shoot(Dash_Shooter.millis);

        while (robot.shooter.getRPM() < ps_rpm){
            robot.shooter.setRPM(ps_rpm);
            multTelemetry.addData("RPM", robot.shooter.getRPM());
            multTelemetry.update();
        };
        robot.turn(startAngle + (7 * 2), 0.01);
        shoot(Dash_Shooter.millis);*/
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

            // Dimensions for top rectangle
            rectTopX1 = (int) (input.rows() * Dash_Vision.rectTopX1Percent);
            rectTopX2 = (int) (input.rows() * Dash_Vision.rectTopX2Percent) - rectTopX1;
            rectTopY1 = (int) (input.cols() * Dash_Vision.rectTopY1Percent);
            rectTopY2 = (int) (input.cols() * Dash_Vision.rectTopY2Percent) - rectTopY1;

            // Dimensions for bottom rectangle
            rectBottomX1 = (int) (input.rows() * Dash_Vision.rectBottomX1Percent);
            rectBottomX2 = (int) (input.rows() * Dash_Vision.rectBottomX2Percent) - rectBottomX1;
            rectBottomY1 = (int) (input.cols() * Dash_Vision.rectBottomY1Percent);
            rectBottomY2 = (int) (input.cols() * Dash_Vision.rectBottomY2Percent) - rectBottomY1;

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


            // Check 4 rings
            if (

                    finalUpperAve > Dash_Vision.orangeMin &&
                            finalUpperAve < Dash_Vision.orangeMax

            ) ringCount = 4.0;
                // Check 0 rings
            else if (

                    finalLowerAve > Dash_Vision.orangeMax ||
                            finalLowerAve < Dash_Vision.orangeMin

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

            /*
            multTelemetry.addData("RECT_TOP_X1", DashConstants.rectTopX1Percent);
            multTelemetry.addData("RECT_TOP_Y1", DashConstants.rectTopY1Percent);
            multTelemetry.addData("RECT_TOP_X2", DashConstants.rectTopX2Percent);
            multTelemetry.addData("RECT_TOP_Y2", DashConstants.rectTopY2Percent);
            multTelemetry.addData("RECT_BOTTOM_X1", DashConstants.rectBottomX1Percent);
            multTelemetry.addData("RECT_BOTTOM_Y1", DashConstants.rectBottomY1Percent);
            multTelemetry.addData("RECT_BOTTOM_X2", DashConstants.rectBottomX2Percent);
            multTelemetry.addData("RECT_BOTTOM_Y2", DashConstants.rectBottomY2Percent);
            */

            multTelemetry.addData("Ring Count", ringCount);
            multTelemetry.addData("finalLowerAve: ", finalLowerAve);
            multTelemetry.addData("finalUpperAve: ", finalUpperAve);
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
