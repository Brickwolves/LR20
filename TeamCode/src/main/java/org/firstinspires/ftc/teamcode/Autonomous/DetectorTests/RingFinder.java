package org.firstinspires.ftc.teamcode.Autonomous.DetectorTests;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.SanicPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

@Autonomous(name="RingFinder", group="Autonomous Linear Opmode")
public class RingFinder extends LinearOpMode
{
    private SanicPipe ringFinder = new SanicPipe();
    public static IMU imu;
    private Mecanum robot;
    private ButtonControls BC;

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);

        initVision();
    }

    public void initVision(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam2"), cameraMonitorViewId);
        VisionUtils.webcam.setPipeline(ringFinder);
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();

        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();

        double degree_error = ringFinder.getRingAngle();
        double target_angle = robot.imu.getAngle() + degree_error;
        double ringFieldAngle = target_angle - 180;
        double ringDistance = ringFinder.getDistance2Ring();

        multTelemetry.addData("Status", "Turning to " + target_angle);
        multTelemetry.update();

        robot.linearTurn(ringFieldAngle, 0.05);

        while (opModeIsActive()){

            multTelemetry.addData("Initial DegError", degree_error);
            multTelemetry.addData("Target Angle", target_angle);
            multTelemetry.addData("ringFieldAngle", ringFieldAngle);
            multTelemetry.addData("ringDistance", ringDistance);


            /*
            multTelemetry.addData("Ring Distance", ringFinder.getDistance2Ring());
            multTelemetry.addData("Ring Count", ringFinder.getRingCount());
             */

            multTelemetry.addData("FPS", String.format("%.2f", VisionUtils.webcam.getFps()));
            multTelemetry.update();
        }
    }
}
