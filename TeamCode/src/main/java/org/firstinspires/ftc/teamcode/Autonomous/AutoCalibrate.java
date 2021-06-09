package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Navigation.Oracle;
import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.Vision.SanicPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_AimBot.INIT_COMPLETED;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.START_ANGLE;
import static org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_Intake.INTAKE_RPM_FORWARDS;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.getAngle;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setAngle;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setBLPosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setBRPosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setFLPosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setFRPosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setIntakePosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setUpdateTask;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setXPosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setYPosition;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.closestAngle;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

@Autonomous(name="AutoCalibrate", group="Autonomous Linear Opmode")
public class AutoCalibrate extends LinearOpMode
{
    private SanicPipe ringFinder = new SanicPipe();
    private AimBotPipe aimBot = new AimBotPipe();
    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime t = new ElapsedTime();

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        robot.imu.setOffsetAngle(START_ANGLE);
        BC = new ButtonControls(gamepad1);

        // Initialize Oracle
        setUpdateTask(() -> {
            setAngle(robot.imu.getAngle());
            setXPosition(robot.getX());
            setYPosition(robot.getYComp());
            setIntakePosition(robot.intake.getIntakePosition());
            setFRPosition(robot.fr.getCurrentPosition());
            setFLPosition(robot.fl.getCurrentPosition());
            setBRPosition(robot.br.getCurrentPosition());
            setBLPosition(robot.bl.getCurrentPosition());
        });

        // Move down for camera to init
        robot.intake.rollerMid();

        setUpVision();

        // Move it back up
        robot.intake.rollerUp();

        gripperSequence();
    }


    public void gripperSequence(){
        t.reset();
        while (t.seconds() < 1){
            robot.claw.open();
            robot.arm.up();
        }
        t.reset();
        while (!BC.get(CROSS, DOWN) && t.seconds() < 2){
            ButtonControls.update();
            multTelemetry.addData("Status", "Press X to close gripper.");
            multTelemetry.update();
        }
        t.reset();
        while (t.seconds() < 1){
            robot.claw.close();
        }
    }

    public void setUpVision(){

        int cameraMonitorViewId1 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VisionUtils.webcam_back = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam2"));
        VisionUtils.webcam_back.setPipeline(ringFinder);
        VisionUtils.webcam_back.openCameraDeviceAsync(() -> VisionUtils.webcam_back.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        VisionUtils.webcam_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId1);
        VisionUtils.webcam_front.setPipeline(aimBot);
        VisionUtils.webcam_front.openCameraDeviceAsync(() -> VisionUtils.webcam_front.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));





        while (!BC.get(CROSS, DOWN) && t.seconds() < 2){
            ButtonControls.update();
            INIT_COMPLETED = false;

            multTelemetry.addData("Status", "Press X to Stop Camera Calibration.");
            multTelemetry.update();
        }
        INIT_COMPLETED = true;

    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();

        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();

        while (opModeIsActive()){

            multTelemetry.addData("Status", "Running auto");
            multTelemetry.update();

        }
    }
}
