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
import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.Vision.SanicPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.getAngle;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

@Disabled
@Autonomous(name="Mark 10", group="Autonomous Linear Opmode")
public class Mark10 extends LinearOpMode
{
    private SanicPipe ringFinder = new SanicPipe();
    private AimBotPipe aimBot = new AimBotPipe();
    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime time = new ElapsedTime();

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        robot.imu.setOffsetAngle(0);
        BC = new ButtonControls(gamepad1);

        initVision();

        gripperSequence();
    }


    public void gripperSequence(){
        time.reset();
        while (time.seconds() < 1){
            robot.claw.open();
            robot.arm.up();
        }
        time.reset();
        while (!BC.get(CROSS, DOWN) && time.seconds() < 2){
            ButtonControls.update();
            multTelemetry.addData("Status", "Press X to close gripper.");
            multTelemetry.update();
        }
        time.reset();
        while (time.seconds() < 1){
            robot.claw.close();
        }
    }

    public void initVision(){
        int cameraMonitorViewId1 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId1);
        VisionUtils.webcam_front.setPipeline(aimBot);
        VisionUtils.webcam_front.openCameraDeviceAsync(() -> VisionUtils.webcam_front.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        VisionUtils.webcam_back = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam2"));
        VisionUtils.webcam_back.setPipeline(ringFinder);
        VisionUtils.webcam_back.openCameraDeviceAsync(() -> VisionUtils.webcam_back.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
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

    public void shoot(int rings, double waitSeconds){
        time.reset();
        while (true) {
            robot.shooter.setRPM(3200);

            if (time.seconds() > waitSeconds) {
                if (robot.shooter.getFeederCount() < rings) robot.shooter.feederState(true);
                else break;
            }

            multTelemetry.addData("Position", robot.shooter.getPosition());
            multTelemetry.addData("RPM", robot.shooter.getRPM());
            multTelemetry.update();
        }
        //robot.shooter.setPower(0);
        robot.shooter.setFeederCount(0);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();

        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();

        if (opModeIsActive()){
            C();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void doPowerShot(VisionUtils.PowerShot powerShot){
        time.reset();
        while (time.seconds() < 2){
            robot.shooter.setRPM(aimBot.calcRPM() - 300);
            double psAngle = aimBot.getPowerShotAngle(powerShot, getAngle());
            robot.setDrivePowerPS(0, 0, psAngle, 1);
        }
        robot.setDrivePower(0, 0, 0, 0);
        shoot(1, 0.2);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void C(){

        // Strafe to PowerShots
        robot.strafePowerRamp(170, 1200, 0.3, 175, 0.1, 0, () -> {
            robot.shooter.setRPM(3200); //3200
            robot.wings.mid();
        });
        robot.intake.rollerMid();

        sleep(200);

        /*

        // Do powershots
        doPowerShot(PS_MIDDLE);
        doPowerShot(PS_RIGHT);
        doPowerShot(PS_LEFT);

        // Shooter off
        robot.shooter.setPower(0);

         */

        BREAKPOINT();



        // Drop to C
        time.reset();
        robot.strafePowerRamp(210, 1800, 0.4, 20, 0, 0,
                () -> {
            if (time.seconds() > 1) robot.arm.out();
        });

        /*
        time.reset();
        while (time.seconds() < 0.5){
            robot.claw.open();
            robot.arm.up();
        }
         */

        // Return to other wobble goal
        time.reset();
        robot.strafePowerRamp(-10, 2000, 0.3, 180, 0.45, 0,
                () -> {
                    robot.wings.up();
                    robot.intake.rollerUp();
                    robot.claw.open();
                    if (time.seconds() > 0.4 && time.seconds() < 0.7) robot.arm.up();
                    if (time.seconds() > 0.7) robot.arm.out();
                });

        time.reset();
        while (time.seconds() > 0.5 && time.seconds() < 1){
            robot.claw.close();
        }

        // Prepare to knock down rings
        time.reset();
        robot.strafePowerRamp(90, 700, 0.05, 180, 0, 0,
            () -> {
                robot.claw.close();
                if (time.seconds() > 0.6) robot.arm.up();
            });

        robot.intake.rollerMid();

    }
}
