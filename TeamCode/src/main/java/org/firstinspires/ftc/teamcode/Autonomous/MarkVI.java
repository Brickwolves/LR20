package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement.diag_deg;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement.diagnostic_inches;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Vision.ring_count;

@Autonomous(name="MarkVI", group="Autonomous Linear Opmode")
public class MarkVI extends LinearOpMode {

    private MecanumRobot robot;
    private ControllerCollin controller;
    private ElapsedTime time;

    private int ringCount;
    OpenCvCamera webcam;

    public void initialize(){
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller = new ControllerCollin(gamepad1);
        time = new ElapsedTime();
    }

    public void BREAKPOINT(){
        while (true){
            Utils.multTelemetry.addData("Status", "Holding");
            Utils.multTelemetry.update();
            if (controller.src.cross) break;
        }
    }

    public void shoot(int rings){
        time.reset();
        while (true) {
            robot.shooter.setRPM(3700);

            if (time.seconds() > 2) {
                if (robot.shooter.feederCount() < rings) robot.shooter.feederState(true);
                else break;
            }

            Utils.multTelemetry.addData("Position", robot.shooter.getPosition());
            Utils.multTelemetry.addData("RPM", robot.shooter.getRPM());

            Utils.multTelemetry.update();
        }
        robot.shooter.setPower(0);
        robot.shooter.setFeederCount(0);
    }

    public void startVision(){
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
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        startVision();

        while (!controller.src.cross){
            robot.claw.open();
        }
        time.reset();
        while (time.seconds() < 2){
            robot.claw.close();
        }

        waitForStart();

        if (opModeIsActive()){

            webcam.stopStreaming();
            if (ringCount == 0) A();
            else if (ringCount == 1) B();
            else C();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void A(){
        robot.strafe(0, 40, 90, 0.1, () -> robot.intake.armDown());
        robot.strafe(-90, 14, 90, 0.1, null);

        shoot(4);

        robot.strafe(30, 46, 90, 0.1, null);
        robot.strafe(90, 17, 90, 0.1, null);


        time.reset();
        while (time.seconds() < 1){ robot.arm.out(); }

        time.reset();
        while (time.seconds() < 1){ robot.claw.open(); }

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void B(){

        // Strafe to shooting position
        robot.strafe(0, 40, 90, 0.1, () -> robot.intake.armDown());
        robot.strafe(-90, 14, 90, 0.1, null);
        shoot(4);

        // Intake Rings
        robot.strafe(90, 10, 90, 0.1, null);
        robot.turn(0, 0.01);
        robot.intake.setIntakePower(1);
        robot.strafe(0, 20, 0, 0.1, null);
        sleep(500);
        robot.intake.setIntakePower(0);


        robot.strafe(180, 10, 0, 0.05, () -> robot.intake.armDown());
        robot.turn(90, 0.05);
        robot.strafe(0, 5, 90, 0.1, null);

        shoot(2);

        robot.strafe(-10, 40, 90, 0.1, null);

        robot.turn(-90, 0.05);

        time.reset();
        while (time.seconds() < 1){
            robot.arm.out();
        }

        time.reset();
        while (time.seconds() < 1){
            robot.claw.open();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void C(){

        // Strafe to shooting position
        robot.strafe(0, 40, 90, 0.1, () -> robot.intake.armMid());
        robot.strafe(-90, 14, 90, 0.1, null);
        shoot(4);

        // Knock of the top ring
        robot.strafe(180, 10, 90, 0.1, null);
        robot.strafe(-90, 14, 90, 0.1, null);

        // Strafe back to put down front bumper
        robot.strafe(150, 7, 90, 0.1, null);

        time.reset();
        while (time.seconds() < 1){ robot.intake.armDown(); }

        // Intake rings
        robot.intake.setIntakePower(1);
        robot.strafe(0, 25, 90, 0.01, null);
        sleep(1000);
        robot.intake.setIntakePower(0);

        // Strafe back to shooting position and shoot
        robot.strafe(105, 14, 90, 0.05, null);
        shoot(4);

        // Strafe to C for WG
        robot.strafe(0, 70, 90, 0.5, null);
        robot.turn(-50, 0.01);

        time.reset();
        while (time.seconds() < 1){ robot.arm.out(); }

        time.reset();
        while (time.seconds() < 1){ robot.claw.open(); }

        /*

        auto for getting the last ring

        // Get ready to intake last ring
        robot.strafe(-100, 30, 90, 0.1, null);

        // Intake last ring while strafing
        robot.intake.setIntakePower(1);
        robot.strafe(0, 25, 90, 0.01, null);
        sleep(1000);
        robot.intake.setIntakePower(0);

        // Strafe to shooting position and shoot last ring
        robot.strafe(110, 34, 90, 0.1, null);
        shoot(2);

        // Strafe to C for WG
        robot.strafe(0, diagnostic_inches, 90, 0.1, null);
        robot.turn(diag_deg, 0.01);
         */

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

            ) ringCount = 4;
                // Check 0 rings
            else if (

                    finalLowerAve > Dash_Vision.orangeMax ||
                            finalLowerAve < Dash_Vision.orangeMin

            ) ringCount = 0;
            else ringCount = 1;

            Utils.multTelemetry.addData("Ring Count", ringCount);
            Utils.multTelemetry.addData("finalLowerAve: ", finalLowerAve);
            Utils.multTelemetry.addData("finalUpperAve: ", finalUpperAve);
            Utils.multTelemetry.update();

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
