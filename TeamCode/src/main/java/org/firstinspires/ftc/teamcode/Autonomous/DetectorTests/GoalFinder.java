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
import org.firstinspires.ftc.teamcode.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="GoalFinder", group="Autonomous Linear Opmode")
public class GoalFinder extends LinearOpMode
{
    private AimBotPipe goalFinder = new AimBotPipe();
    public static IMU imu;
    private Mecanum robot;
    private ButtonControls BC;

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);

        imu = new IMU("imu");
        initVision();
    }

    public void initVision(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        VisionUtils.webcam.setPipeline(goalFinder);
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();

        OpModeUtils.multTelemetry.addLine("Waiting for start");
        OpModeUtils.multTelemetry.update();
        waitForStart();

        double degree_error = goalFinder.getGoalDegreeError();
        double target_angle = robot.imu.getAngle() + degree_error;

        OpModeUtils.multTelemetry.addData("Status", "Turning to " + target_angle);
        OpModeUtils.multTelemetry.update();

        robot.linearTurn(target_angle, 0.1);

        while (opModeIsActive()){

            degree_error = goalFinder.getGoalDegreeError();
            double turn = robot.rotationPID.update(degree_error) * -1;
            robot.setDrivePower(0, 0, turn, 1);

            OpModeUtils.multTelemetry.addData("Goal Error", goalFinder.getGoalDegreeError());
            OpModeUtils.multTelemetry.addData("FPS", String.format("%.2f", VisionUtils.webcam.getFps()));
            OpModeUtils.multTelemetry.update();
        }
    }
}
