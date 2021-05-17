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
import org.firstinspires.ftc.teamcode.Navigation.Oracle;
import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.Vision.SanicPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.START_ANGLE;
import static org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_Intake.INTAKE_RPM_FORWARDS;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.getAngle;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setAngle;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setIntakePosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setUpdateTask;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setXPosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setYPosition;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.closestAngle;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PowerShot.PS_LEFT;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PowerShot.PS_MIDDLE;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PowerShot.PS_RIGHT;

@Disabled
@Autonomous(name="Mark 11", group="Autonomous Linear Opmode")
public class Mark11 extends LinearOpMode
{
    private SanicPipe ringFinder = new SanicPipe();
    private AimBotPipe aimBot = new AimBotPipe();
    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime time = new ElapsedTime();

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
            Oracle.frPosition = robot.fr.getCurrentPosition();
            Oracle.flPosition = robot.fl.getCurrentPosition();
            Oracle.brPosition = robot.br.getCurrentPosition();
            Oracle.blPosition = robot.bl.getCurrentPosition();

        });

        setUpVision();

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

    public void setUpVision(){
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
            Oracle.update();

            multTelemetry.addData("X", curO.x);
            multTelemetry.addData("Y", curO.y);
            multTelemetry.addData("A", curO.a);
            multTelemetry.addData("Oracle Angle", getAngle());
            multTelemetry.update();
        }
    }

    public void shootGoal(int rings, double waitSeconds){
        time.reset();
        robot.wings.mid();
        while (true) {
            robot.shooter.setRPM(aimBot.calcRPM());

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

    public void shootPowerShot(int rings, double waitSeconds, double rpm){
        time.reset();
        robot.wings.mid();
        while (true) {
            robot.shooter.setRPM(rpm);

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

            int rings = ringFinder.getRingCount();

            powerShotSequence();

            if (rings == 0) B();
            else if (rings == 1) B();
            else C();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void doPowerShot(VisionUtils.PowerShot powerShot){
        time.reset();

        // Should take 1.2 seconds
        while (time.seconds() < 0.6){

            // Update Oracle
            Oracle.update();

            // Set the RPM
            double RPM = aimBot.calcRPM() - 400;
            if (powerShot == PS_MIDDLE) RPM += 100;
            robot.shooter.setRPM(RPM);

            // Turn to PowerShot
            double psAngle = aimBot.getPowerShotAngle(powerShot, getAngle());
            robot.setDrivePowerPS(0, 0, psAngle, 1);

        }

        // Stop all movement
        robot.setDrivePower(0, 0, 0, 0);

        // Shoot
        double RPM = aimBot.calcRPM() - 400;
        if (powerShot == PS_MIDDLE) RPM -= 200;
        if (powerShot == PS_LEFT) RPM -= 100;
        shootPowerShot(1, 0.2, RPM);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void powerShotSequence(){

        // Rotating-Strafe to PowerShots
        robot.strafePowerRamp(170, 1150, 0.3, 178, 0.1, 0, () -> {
            robot.shooter.setRPM(3000); //3200
            robot.wings.mid();
        });


        // Roller Down for Camera
        robot.intake.rollerMid();
        sleep(200);


        // PowerShots
        doPowerShot(PS_MIDDLE);
        doPowerShot(PS_RIGHT);
        doPowerShot(PS_LEFT);
        robot.shooter.setPower(0);
    }


    public void A(){

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void B(){

        // Turn with back facing B
        time.reset();
        while (time.seconds() < 1.5){

            Oracle.update();

            double targetAngle = closestAngle(18, getAngle());
            double error = targetAngle - getAngle();
            double turn = robot.rotationPID.update(error) * -1;
            robot.setDrivePower(0, 0, turn * 0.85, 1);
        }
        robot.setAllPower(0);


        // Strafe backwards to B
        time.reset();
        while (time.seconds() < 0.8){
            robot.setDrivePower(-0.7, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Drop Wobble-Goal to B
        time.reset();
        while (time.seconds() < 1.5){

            robot.arm.out();
            if (time.seconds() > 1) robot.claw.open();
        }


        // Strafe Forwards a teensy bit
        time.reset();
        while (time.seconds() < 0.2){
            robot.setDrivePower(0.7, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Turn to face rings
        time.reset();
        while (time.seconds() < 1.5){

            Oracle.update();

            robot.arm.up();

            double targetAngle = closestAngle(-9, getAngle());
            double error = targetAngle - getAngle();
            double turn = robot.rotationPID.update(error) * -1;
            robot.setDrivePower(0, 0, turn * 0.85, 1);
        }
        robot.setAllPower(0);


        // Move forward, intaking rings
        robot.strafePowerRamp(-9, 2300, 0.1, -9, 0, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);
        });


        // Turn 140 with back facing wobble goal
        time.reset();
        while (time.seconds() < 1.5){

            Oracle.update();

            robot.arm.up();

            double targetAngle = closestAngle(145, getAngle());
            double error = targetAngle - getAngle();
            double turn = robot.rotationPID.update(error) * -1;
            robot.setDrivePower(0, 0, turn * 0.85, 1);
        }
        robot.setAllPower(0);


        // Turn off intake
        robot.intake.setIntakePower(0);


        // Start to put out arm
        time.reset();
        while (time.seconds() < 0.2){
            robot.arm.out();

        }


        // Strafe backwards to wobble goal
        time.reset();
        while (time.seconds() < 0.3){
            robot.arm.out();
            robot.setDrivePower(-0.3, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Grab second Wobble-Goal
        time.reset();
        while (time.seconds() < 2){

            robot.shooter.setRPM(aimBot.calcRPM());

            if (time.seconds() < 0.8) robot.arm.out();
            if (0.8 < time.seconds()) robot.claw.close();
            if (time.seconds() > 1.2) robot.arm.up();
        }


        // Turn to 180 for one sec and then orient to goal
        time.reset();
        while (time.seconds() < 1){

            Oracle.update();

            robot.arm.up();
            robot.shooter.setRPM(aimBot.calcRPM());

            double turn;
            if (time.seconds() < 1){
                double targetAngle = closestAngle(185, getAngle());
                double error = targetAngle - getAngle();
                turn = robot.rotationPID.update(error) * -1;
                turn *= 0.85;
                robot.setDrivePower(0, 0, turn, 1);

            }
            else {
                double goalAngle = aimBot.getGoalAngle(getAngle(), 0);
                robot.setDrivePowerGoal(0, 0, goalAngle, 1);
            }
        }
        robot.setAllPower(0);



        // Shoot top goal
        time.reset();
        while (time.seconds() < 1){
            robot.shooter.setRPM(aimBot.calcRPM());
            robot.shooter.feederState(true);
        }
        robot.shooter.setPower(0);


        // Strafe to B
        time.reset();
        robot.strafePowerRamp(165, 1200, 0.1, 0, 0, 0, () -> {

        });

        // Strafe Forwards to get out of wobble-range
        time.reset();
        while (time.seconds() < 0.4){
            robot.arm.out();
            robot.setDrivePower(0.5, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Drop 2nd Wobble-Goal to B
        time.reset();
        while (time.seconds() < 1.2){

            robot.arm.out();
            if (time.seconds() > 1) robot.claw.open();
        }


        // Strafe to Navigation
        robot.strafeStaticPower(0, 500, 0.3, 0, 0, 0, () -> {
        });


        // Turn to 180
        time.reset();
        while (time.seconds() < 1){

            Oracle.update();

            robot.arm.up();

            double targetAngle = closestAngle(180, getAngle());
            double error = targetAngle - getAngle();
            double turn = robot.rotationPID.update(error) * -1;
            robot.setDrivePower(0, 0, turn * 0.85, 1);
        }
        robot.setAllPower(0);


    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    public void C(){


        // Rotating-Strafe to Wobble-Goal to C
        time.reset();
        robot.strafePowerRamp(210, 1600, 0.3, 20, 0, 0,
                () -> {
            if (time.seconds() > 2) robot.arm.out();
        });


        // Open Claw
        time.reset();
        while (time.seconds() < 0.2){
            robot.claw.open();
        }


        // Drop Wobble Goal, & return to 2nd Wobble-Goal
        time.reset();
        robot.strafePowerRamp(-10, 2450, 0.1, 180, 0.45, 0,
                () -> {
                    robot.wings.up();
                    robot.intake.rollerUp();
                    robot.claw.open();
                    if (time.seconds() > 0.4 && time.seconds() < 0.7) robot.arm.up();
                    if (time.seconds() > 0.7) robot.arm.out();
                });


        // Grab Wobble-Goal
        time.reset();
        while (time.seconds() > 1 && time.seconds() < 2){
            robot.claw.close();
        }


        // Prepare to knock down rings
        time.reset();
        robot.strafePowerRamp(65, 1000, 0.2, 180, 0, 0,
            () -> {
                robot.claw.close();
                if (time.seconds() > 0.6) robot.arm.up();
            });

        robot.intake.rollerDown();



        // Knock over rings
        time.reset();
        robot.strafePowerRamp(180, 600, 0.1, 180, 0, 0, () -> {
            robot.intake.rollerDown();
            robot.wings.mid();

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
        });


        // Backtrack
        time.reset();
        while (time.seconds() < 0.4){
            robot.setDrivePower(-0.4, 0, 0, 1);
        }
        robot.setAllPower(0);



        // Move fast a bit and intake
        robot.strafeStaticPower(180, 100, 0.3, 185, 0, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        });


        // Move slow a bit and intake
        robot.strafeStaticPower(180, 400, 0.1, 185, 0, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        });


        // Move fast a bit and intake
        robot.strafeStaticPower(180, 200, 0.3, 185, 0, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        });

        // Just intake and shoot
        time.reset();
        while (time.seconds() < 2){
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        }
        robot.shooter.setPower(0);



        // Strafe to C to drop off Wobble-Goal-2
        time.reset();
        robot.strafePowerRamp(185, 1600, 0.6, 40, 0, 0,
            () -> {
                if (time.seconds() > 2) robot.arm.out();
                robot.intake.setIntakePower(0);
                robot.intake.rollerUp();
            });


        // Strafe to navigation
        time.reset();
        robot.strafePowerRamp(40, 1500, 0.6, 40, 0,0, () -> {
            robot.claw.open();
        });


        // Turn to face goal
        time.reset();
        robot.linearTurn(180, 0.1);
    }
}
