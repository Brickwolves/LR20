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
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.ACCELERATION;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.findClosestAngle;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

@Autonomous(name="Mark 9", group="Autonomous Linear Opmode")
public class Mark9 extends LinearOpMode
{
    private SanicPipe ringFinder = new SanicPipe();
    private AimBotPipe aimBot = new AimBotPipe();
    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime time = new ElapsedTime();

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);

        initVision();
    }

    public void initVision(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        VisionUtils.webcam.setPipeline(aimBot);
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
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
        robot.shooter.setPower(0);
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


        Orientation HOME = new Orientation(0,0, 180);
        Orientation POWER_SHOTS = new Orientation(-2300,0, 177);

        if (opModeIsActive()){

            BREAKPOINT();

            robot.linearStrafe(POWER_SHOTS, ACCELERATION, null);

            BREAKPOINT();

            // Turn to 1st
            double psRightAngle = findClosestAngle(getPowerShotAngle(VisionUtils.PowerShot.PS_RIGHT), robot.imu.getAngle());
            System.out.println("PSRIGHT: " + psRightAngle);
            robot.linearTurn(psRightAngle, 1);

            sleep(1000);

            // Turn to 2nd
            double psMiddleAngle = findClosestAngle(getPowerShotAngle(VisionUtils.PowerShot.PS_MIDDLE), robot.imu.getAngle());
            System.out.println("PSMIDDLE: " + psRightAngle);
            robot.linearTurn(psMiddleAngle, 1);

            sleep(1000);

            // Turn to 3rd
            double psLeftAngle = findClosestAngle(getPowerShotAngle(VisionUtils.PowerShot.PS_LEFT), robot.imu.getAngle());
            System.out.println("PSLEFT: " + psRightAngle);
            robot.linearTurn(psLeftAngle, 1);

            BREAKPOINT();

            robot.linearStrafe(215, 1600, ACCELERATION, 25, null);

            BREAKPOINT();

            robot.linearStrafe(-20, 2000, ACCELERATION, 200, null);

            BREAKPOINT();
        }
   }
}
