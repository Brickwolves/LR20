package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_StackDetector;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_Shooter.goal_rpm;
import static org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_StackDetector.ring_count;

@Disabled
@Autonomous(name="MarkVIII", group="Autonomous Linear Opmode")
public class Mark8 extends LinearOpMode {

    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime time;

    private int ringCount;
    OpenCvCamera webcam;

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);
        time = new ElapsedTime();
    }

    public void BREAKPOINT(){
        while (true){
            OpModeUtils.multTelemetry.addData("Status", "Holding");
            OpModeUtils.multTelemetry.update();
            if (BC.get(CROSS, DOWN)) break;
        }
    }

    public void shoot(int rings, double waitSeconds){
        time.reset();
        while (true) {
            robot.shooter.setRPM(goal_rpm);

            if (time.seconds() > waitSeconds) {
                if (robot.shooter.getFeederCount() < rings) robot.shooter.feederState(true);
                else break;
            }

            OpModeUtils.multTelemetry.addData("Position", robot.shooter.getPosition());
            OpModeUtils.multTelemetry.addData("RPM", robot.shooter.getRPM());

            OpModeUtils.multTelemetry.update();
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
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        startVision();

        /*
        while (!BC.get(CROSS, DOWN)){
            robot.claw.open();
        }
        time.reset();
        while (time.seconds() < 2){
            robot.claw.close();
        }
         */

        waitForStart();

        if (opModeIsActive()){

            webcam.stopStreaming();
            ringCount = ring_count;
            if (ringCount == 0) A();
            else if (ringCount == 1) B();
            else C();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void A(){

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void B(){

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void C(){

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
            rectTopX1 = (int) (input.rows() * Dash_StackDetector.rectTopX1Percent);
            rectTopX2 = (int) (input.rows() * Dash_StackDetector.rectTopX2Percent) - rectTopX1;
            rectTopY1 = (int) (input.cols() * Dash_StackDetector.rectTopY1Percent);
            rectTopY2 = (int) (input.cols() * Dash_StackDetector.rectTopY2Percent) - rectTopY1;

            // Dimensions for bottom rectangle
            rectBottomX1 = (int) (input.rows() * Dash_StackDetector.rectBottomX1Percent);
            rectBottomX2 = (int) (input.rows() * Dash_StackDetector.rectBottomX2Percent) - rectBottomX1;
            rectBottomY1 = (int) (input.cols() * Dash_StackDetector.rectBottomY1Percent);
            rectBottomY2 = (int) (input.cols() * Dash_StackDetector.rectBottomY2Percent) - rectBottomY1;

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

                    finalUpperAve > Dash_StackDetector.orangeMin &&
                            finalUpperAve < Dash_StackDetector.orangeMax

            ) ringCount = 4;
                // Check 0 rings
            else if (

                    finalLowerAve > Dash_StackDetector.orangeMax ||
                            finalLowerAve < Dash_StackDetector.orangeMin

            ) ringCount = 0;
            else ringCount = 1;

            OpModeUtils.multTelemetry.addData("Ring Count", ringCount);
            OpModeUtils.multTelemetry.addData("finalLowerAve: ", finalLowerAve);
            OpModeUtils.multTelemetry.addData("finalUpperAve: ", finalUpperAve);
            OpModeUtils.multTelemetry.update();

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
