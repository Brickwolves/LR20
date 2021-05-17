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

@Autonomous(name="Mark 13", group="Autonomous Linear Opmode")
public class Mark13 extends LinearOpMode
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

        setUpVision();

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
        while (!BC.get(CROSS, DOWN) && opModeIsActive()){

            ButtonControls.update();
            Oracle.update();

            multTelemetry.addData("X", curO.x);
            multTelemetry.addData("Y", curO.y);
            multTelemetry.addData("A", curO.a);
            multTelemetry.addData("Oracle Angle", getAngle());
            multTelemetry.update();
        }
    }

    public void shoot(int rings, double waitSeconds){
        t.reset();
        robot.wings.mid();
        while (true) {
            robot.shooter.setRPM(aimBot.calcRPM());

            if (t.seconds() > waitSeconds) {
                if (robot.shooter.getFeederCount() < rings) robot.shooter.feederState(true);
                else break;
            }

            multTelemetry.addData("Position", robot.shooter.getPosition());
            multTelemetry.addData("RPM", robot.shooter.getRPM());
            multTelemetry.update();
        }
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

            goalSequence();
            rings = 0;

            if (rings == 0) A();
            else if (rings == 1) B();
            else C();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void shootGoal(){

        ElapsedTime t = new ElapsedTime();
        t.reset();

        // Should take 1.2 seconds
        while (t.seconds() < 1){

            // Update Oracle
            Oracle.update();

            // Set the RPM
            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);

            // Turn to PowerShot
            double goalAngle = aimBot.getGoalAngle(getAngle(), 0);
            robot.setDrivePowerGoal(0, 0, goalAngle, 1);

        }
        robot.setAllPower(0);
        shoot(3, 0.2);
        robot.shooter.setPower(0);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void goalSequence(){

        // Rotating-Strafe to PowerShots
        t.reset();
        robot.strafeStaticPower(170, 2000, 0.5, 0, 0.5, () -> {
            robot.shooter.setRPM(3000); //3200
            robot.wings.mid();
        });

        // Turn near goal
        robot.turnTime(178, 1, () -> {
            robot.shooter.setRPM(3000); //3200
            robot.intake.rollerMid();
        });


        // Top Goal
        shootGoal();

    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    public void A(){

        robot.turnTime(50, 1, null);

        // Place WG @ A
        t.reset();
        robot.strafeStaticPower(235, 1100, 0.4, 50, 0, () -> {
            robot.intake.rollerUp();
            if (t.seconds() > 1) robot.arm.out();
        });


        // Open claw
        t.reset();
        while (t.seconds() < 1){
            robot.claw.open();
        }


        // Move out a smidge
        robot.strafeStaticPower(50, 50, 0.3, 50, 0, () -> {
            robot.claw.open();
            robot.arm.up();
        });

        // Turn to face back wall
        robot.turnTime(180, 1, null);


        // PART 1: Strafe to 2nd WG
        t.reset();
        robot.strafeStaticPower(-45, 900, 0.5, 180, 0, () -> robot.arm.out());
        // PART 2: Strafe to 2nd WG
        robot.strafeStaticPower(0, 900, 0.3, 180, 0, () -> robot.arm.out());


        // Grab WG
        t.reset();
        while (t.seconds() < 1){
            robot.claw.close();
        }


        // Raise WG
        t.reset();
        while (t.seconds() < 1){
            robot.arm.up();
            robot.claw.close();
        }


        // Turn to A
        robot.turnTime(0, 1, () -> {
            robot.arm.up();
            robot.claw.close();
        });


        // Strafe to A
        robot.strafeStaticPower(170, 700, 0.5, 0, 0, () -> {
            robot.arm.up();
            robot.claw.close();
        });


        // Drop WG
        t.reset();
        while (t.seconds() < 1){
            robot.arm.out();
            if (t.seconds() > 0.5) robot.claw.open();
        }


        // Move away from WG
        robot.strafeStaticPower(0, 100, 0.3, 0, 0, null);


        // Arm Up WG
        t.reset();
        while (t.seconds() < 1){
            robot.arm.up();
        }


        // Strafe sidways to clear A Zone
        robot.strafeTime(90, 2, 0.4, 180, 1, null);


        // Strafe forward to navigation
        robot.strafeTime(180, 1, 0.4, 180, 0, null);

    }



    @RequiresApi(api = Build.VERSION_CODES.N)
    public void B(){

        /*

                13.21 VOLTS

         */


        // Turn with back facing B
        t.reset();
        while (t.seconds() < 1.5){

            Oracle.update();

            double targetAngle = closestAngle(18, getAngle());
            double error = targetAngle - getAngle();
            double turn = robot.rotationPID.update(error) * -1;
            robot.setDrivePower(0, 0, turn * 0.85, 1);
        }
        robot.setAllPower(0);


        // Strafe backwards to B
        t.reset();
        while (t.seconds() < 0.8){
            robot.setDrivePower(-0.7, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Drop Wobble-Goal to B
        t.reset();
        while (t.seconds() < 1.5){

            robot.arm.out();
            if (t.seconds() > 1) robot.claw.open();
        }


        // Strafe Forwards a teensy bit
        t.reset();
        while (t.seconds() < 0.2){
            robot.setDrivePower(0.7, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Turn to face rings
        t.reset();
        while (t.seconds() < 1.5){

            Oracle.update();

            robot.arm.up();

            double targetAngle = closestAngle(-9, getAngle());
            double error = targetAngle - getAngle();
            double turn = robot.rotationPID.update(error) * -1;
            robot.setDrivePower(0, 0, turn * 0.85, 1);
        }
        robot.setAllPower(0);


        // Move forward, intaking rings
        robot.strafePowerRamp2(-9, 2200, 0.1, -9, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);
        });


        // Turn 140 with back facing wobble goal
        t.reset();
        while (t.seconds() < 1.5){

            Oracle.update();

            robot.arm.up();

            double targetAngle = closestAngle(135, getAngle());
            double error = targetAngle - getAngle();
            double turn = robot.rotationPID.update(error) * -1;
            robot.setDrivePower(0, 0, turn * 0.85, 1);
        }
        robot.setAllPower(0);


        // Turn off intake
        robot.intake.setIntakePower(0);


        // Start to put out arm
        t.reset();
        while (t.seconds() < 0.2){
            robot.arm.out();

        }


        // Strafe backwards to wobble goal
        t.reset();
        while (t.seconds() < 0.3){
            robot.arm.out();
            robot.setDrivePower(-0.3, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Grab second Wobble-Goal
        t.reset();
        while (t.seconds() < 2){

            robot.shooter.setRPM(aimBot.calcRPM());

            if (t.seconds() < 0.8) robot.arm.out();
            if (0.8 < t.seconds()) robot.claw.close();
            if (t.seconds() > 1.2) robot.arm.up();
        }


        // Turn to 180 for one sec and then orient to goal
        t.reset();
        while (t.seconds() < 1){

            Oracle.update();

            robot.arm.up();
            robot.shooter.setRPM(aimBot.calcRPM());

            double turn;
            if (t.seconds() < 1){
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
        t.reset();
        while (t.seconds() < 1){
            robot.shooter.setRPM(aimBot.calcRPM());
            robot.shooter.feederState(true);
        }
        robot.shooter.setPower(0);


        // Strafe to B
        t.reset();
        robot.strafePowerRamp2(165, 1200, 0.1, 0, 0, () -> {

        });

        // Strafe Forwards to get out of wobble-range
        t.reset();
        while (t.seconds() < 0.4){
            robot.arm.out();
            robot.setDrivePower(0.5, 0, 0, 1);
        }
        robot.setAllPower(0);


        // Drop 2nd Wobble-Goal to B
        t.reset();
        while (t.seconds() < 1.2){

            robot.arm.out();
            if (t.seconds() > 1) robot.claw.open();
        }


        // Strafe to Navigation
        robot.strafeStaticPower(0, 500, 0.3, 0, 0, () -> {
        });


        // Turn to 180
        t.reset();
        while (t.seconds() < 1){

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
        t.reset();
        robot.strafeTime(210, 2.8, 0.5, 20, 0, () -> {
            if (t.seconds() > 2.4) robot.arm.out();
        });


        // Open Claw
        t.reset();
        while (t.seconds() < 0.2){
            robot.claw.open();
        }


        // Back up a tiny bit
        t.reset();
        robot.strafeTime(20, 0.5, 0.3, 20, 0, () -> {
            if (t.seconds() > 0.2) robot.arm.up();
        });


        // PART 1: Return to 2nd Wobble-Goal
        t.reset();
        robot.strafeTime(-7, 2, 0.7, 180, 0, () -> {
            robot.wings.up();
            robot.intake.rollerUp();
            robot.claw.open();
            if (t.seconds() > 1.8) robot.arm.out();
        });


        // PART 2: Return to 2nd Wobble-Goal
        t.reset();
        robot.strafeTime(-7, 0.8, 0.1, 180, 0, () -> {
        });


        // Grab Wobble-Goal
        t.reset();
        while (1 < t.seconds() && t.seconds() < 2){
            robot.claw.close();
        }

        // Prepare to knock down rings
        t.reset();
        robot.strafeTime(65, 2, 0.3, 180, 0,
            () -> {
                robot.claw.close();
                if (t.seconds() > 0.6) robot.arm.up();
            });

        robot.intake.rollerDown();


        // Knock over rings
        t.reset();
        robot.strafePowerRamp2(180, 600, 0.1, 180, 0, () -> {
            robot.intake.rollerDown();
            robot.wings.mid();

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
        });


        // Backtrack
        t.reset();
        while (t.seconds() < 0.4){
            robot.setDrivePower(-0.4, 0, 0, 1);
        }
        robot.setAllPower(0);



        // Move fast a bit and intake
        robot.strafeStaticPower(180, 100, 0.3, 185, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        });


        // Move slow a bit and intake
        robot.strafeStaticPower(180, 400, 0.1, 185, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        });


        // Move fast a bit and intake
        robot.strafeStaticPower(180, 200, 0.3, 185, 0, () -> {
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        });

        // Just intake and shoot
        t.reset();
        while (t.seconds() < 2){
            robot.intake.rollerMidH();
            robot.intake.setRPM(INTAKE_RPM_FORWARDS);

            double RPM = aimBot.calcRPM();
            robot.shooter.setRPM(RPM);
            robot.shooter.feederState(true);
        }
        robot.shooter.setPower(0);



        // Strafe to C to drop off Wobble-Goal-2
        t.reset();
        robot.strafePowerRamp2(185, 1600, 0.6, 40, 0,
            () -> {
                if (t.seconds() > 2) robot.arm.out();
                robot.intake.setIntakePower(0);
                robot.intake.rollerUp();
            });


        // Strafe to navigation
        t.reset();
        robot.strafePowerRamp2(40, 1500, 0.6, 40, 0,() -> {
            robot.claw.open();
        });


        // Turn to face goal
        t.reset();
        robot.linearTurn(180, 0.1);

    }
}
