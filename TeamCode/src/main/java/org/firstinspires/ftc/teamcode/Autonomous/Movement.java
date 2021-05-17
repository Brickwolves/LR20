package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.SanicPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.ACCELERATION;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.FACE_ANGLE;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.D_TICKS;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.STRAFE_ANGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

@Disabled
@Autonomous(name="Movement", group="Autonomous Linear Opmode")
public class Movement extends LinearOpMode
{
    private SanicPipe ringFinder = new SanicPipe();
    private Mecanum robot;
    private ButtonControls BC;

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        robot.imu.setOffsetAngle(0);
        BC = new ButtonControls(gamepad1);

        initVision();
    }

    public void initVision(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam2"), cameraMonitorViewId);
        VisionUtils.webcam_front.setPipeline(ringFinder);
        VisionUtils.webcam_front.openCameraDeviceAsync(() -> VisionUtils.webcam_front.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    public void BREAKPOINT(){
        ButtonControls.update();
        Orientation curO = robot.odom.getOrientation();
        while (!BC.get(CROSS, DOWN)){

            ButtonControls.update();

            multTelemetry.addData("X", curO.x);
            multTelemetry.addData("Y", curO.y);
            multTelemetry.addData("A", curO.a);
            multTelemetry.update();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();

        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();


        Orientation HOME = new Orientation(0,0, 180);

        if (opModeIsActive()){

            BREAKPOINT();

            robot.strafePowerRamp(STRAFE_ANGLE, D_TICKS, ACCELERATION, FACE_ANGLE, 0, 0, null);
            robot.intake.rollerMid();
            //robot.linearStrafe(new Orientation(X_TICKS, Y_TICKS, ANGLE), ACCELERATION, null);

            BREAKPOINT();

        }
   }
}
