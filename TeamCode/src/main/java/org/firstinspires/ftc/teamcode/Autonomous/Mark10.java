package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.findClosestAngle;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

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

    public double getPowerShotAngle(VisionUtils.PowerShot powerShot){

        // Get degree error and correct
        double rpm = aimBot.calcRPM();
        double errorToGoal = (abs(robot.imu.getModAngle()) - 180);
        double goalDegreeError;
        double powerShotFieldAngle;
        goalDegreeError = aimBot.getGoalDegreeError();
        powerShotFieldAngle = aimBot.getPowerShotDegreeError(powerShot, robot.imu.getAngle());
        return powerShotFieldAngle;
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

            robot.linearStrafe(175, 1100, 0.2, 175, 0.1, () -> {
                robot.shooter.setRPM(3200);
                robot.wings.mid();
            });
            robot.intake.armDown();

            sleep(200);

            // Turn to 1st
            double psRightAngle = findClosestAngle(getPowerShotAngle(VisionUtils.PowerShot.PS_RIGHT), robot.imu.getAngle()) + 4;
            System.out.println("PSRIGHT: " + psRightAngle);
            robot.linearTurn(psRightAngle, 2, () -> robot.shooter.setRPM(3200));

            sleep(200);


            shoot(1, 0.2);

            // Turn to 2nd
            double psMiddleAngle = findClosestAngle(getPowerShotAngle(VisionUtils.PowerShot.PS_MIDDLE), robot.imu.getAngle());
            System.out.println("PSMIDDLE: " + psMiddleAngle);
            robot.linearTurn(psMiddleAngle, 2, () -> robot.shooter.setRPM(3200));
            shoot(1, 0.2);

            // Turn to 3rd
            double psLeftAngle = findClosestAngle(getPowerShotAngle(VisionUtils.PowerShot.PS_LEFT), robot.imu.getAngle()) - 4;
            System.out.println("PSLEFT: " + psLeftAngle);
            robot.linearTurn(psLeftAngle, 2, () -> robot.shooter.setRPM(3200));
            shoot(1, 0.2);

        }
    }
}
