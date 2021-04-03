package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.firstinspires.ftc.teamcode.Vision.GoalFinderPipeline;
import org.firstinspires.ftc.teamcode.Vision.RingFinderPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;

@Autonomous(name="RingFinder", group="Autonomous Linear Opmode")
public class RingFinder extends LinearOpMode
{
    private RingFinderPipeline ringFinder = new RingFinderPipeline();
    public static IMU imu;
    private Mecanum robot;
    private ButtonControls BC;

    public void initialize(){
        Utils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);

        imu = new IMU("imu");
        initVision();
    }

    public void initVision(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        VisionUtils.webcam.setPipeline(ringFinder);
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();

        Utils.multTelemetry.addLine("Waiting for start");
        Utils.multTelemetry.update();
        waitForStart();

        double degree_error = ringFinder.getRingAngle();
        double target_angle = robot.imu.getAngle() + degree_error;

        Utils.multTelemetry.addData("Status", "Turning to " + target_angle);
        Utils.multTelemetry.update();

        robot.turn(target_angle, 0.1);

        while (opModeIsActive()){

            Utils.multTelemetry.addData("Ring Count", ringFinder.getRingCount());
            Utils.multTelemetry.addData("FPS", String.format("%.2f", VisionUtils.webcam.getFps()));
            Utils.multTelemetry.update();
        }
    }
}
