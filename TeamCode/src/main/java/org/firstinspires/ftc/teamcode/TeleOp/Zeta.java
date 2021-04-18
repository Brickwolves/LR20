package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.DetectorTests.GoalFinder;
import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Vision.GoalFinderPipe;
import org.firstinspires.ftc.teamcode.Vision.PSFinderPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.round;
import static java.lang.StrictMath.sin;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Shooter.ps_rpm;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Shooter.goal_rpm;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.UP;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_DN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_L;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_R;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.RB2;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.SQUARE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.TOUCHPAD;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.findClosestAngle;
import static org.firstinspires.ftc.teamcode.TeleOp.Zeta.Target.GOAL;
import static org.firstinspires.ftc.teamcode.TeleOp.Zeta.Target.POWERSHOTS;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PS_LEFT_DIST;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PS_MIDDLE_DIST;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PS_RIGHT_DIST;


@TeleOp(name = "Zeta TeleOp", group="Linear TeleOp")
public class Zeta extends LinearOpMode {

    // Main Stuff
    private Mecanum robot;
    private ButtonControls BC1, BC2;
    private JoystickControls JC1;

    // Camera stuff
    private GoalFinderPipe goalFinder = new GoalFinderPipe();
    private Target aimTarget = GOAL;
    private VisionUtils.PowerShot powerShot = VisionUtils.PowerShot.PS_RIGHT;
    public enum Target {
        GOAL, POWERSHOTS
    }


    // Square button
    private boolean square_pressed = false;

    // PID Stuff
    private double  locked_direction;
    private boolean pid_on = false;
    private boolean last_cycle_pid_state = true;

    // Time Variables
    private double last_nanoseconds = 0.0;
    private double current_nanoseconds = 0.0;
    private ElapsedTime elapsedTime;

    // Hardware Positions
    private double current_angle = 0.0;
    private double last_angle = 0.0;

    private double current_intake_position = 0.0;
    private double last_intake_position = 0.0;

    private ElapsedTime intake_time = new ElapsedTime();

    private RingBuffer<Double> angle_ring_buffer = new RingBuffer<>(5, 0.0);
    private RingBuffer<Double> time_ring_buffer = new RingBuffer<>(5,  0.0);


    public void initialize() {
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC1 = new ButtonControls(gamepad1);
        BC2 = new ButtonControls(gamepad2);
        JC1 = new JoystickControls(gamepad1);
        startVision();

        multTelemetry.addLine("------USER 1----------------------------");
        multTelemetry.addData("Velocity Ranger", "[LB2]");
        multTelemetry.addData("Quick Turn", "[DPAD]");
        multTelemetry.addData("Power Shot", "[RB1 / RB2]");

        multTelemetry.addLine("");

        multTelemetry.addLine("------USER 2----------------------------");
        multTelemetry.addData("Claw", "[TRIANGLE]");
        multTelemetry.addData("Intake ON/OFF", "[RB1]");
        multTelemetry.addData("Intake Direction", "[LB1]");
        multTelemetry.addData("Bumper Toggle", "[DPAD DOWN]");
        multTelemetry.addData("Arm Out", "[DPAD RIGHT]");
        multTelemetry.addData("Arm Up", "[DPAD UP]");
        multTelemetry.addData("Arm In", "[DPAD LEFT]");
        multTelemetry.addData("Shooter ON/OFF", "[CIRCLE]");
        multTelemetry.addData("Shoot", "[RB2]");
        multTelemetry.addData("Switch Target", "[SQUARE]");


        multTelemetry.addData("Shutdown Keys", "[TOUCHPAD] simultaneously");
        multTelemetry.update();

    }

    public void shutdown(){
        multTelemetry.addData("Status", "Shutting Down");
        multTelemetry.update();
        robot.intake.shutdown();
        robot.arm.up();
    }

    public void startVision(){
        /*
        Set up camera, and pipeline
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        VisionUtils.webcam.setPipeline(goalFinder);
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
    }



    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        if (opModeIsActive()){
            elapsedTime = new ElapsedTime();
        }

        while (opModeIsActive()) {

            double velocity = 1;


             /*

             ----------- C A L C U L A T I O N S -----------

                                                          */

            // Calculate Angular Velocity
            current_nanoseconds = elapsedTime.nanoseconds();
            current_angle = robot.imu.getAngle();

            double delta_angle = current_angle - angle_ring_buffer.getValue(current_angle);
            double delta_time = current_nanoseconds - time_ring_buffer.getValue(current_nanoseconds);
            double current_angular_velocity = delta_angle / delta_time;




            last_nanoseconds = current_nanoseconds;
            last_angle       = current_angle;



            /*

             ----------- H A R D W A R E    F U N C T I O N A L I T Y -----------

                                                                               */

            ButtonControls.update();
            JoystickControls.update();



            //                  ARM
            if (BC2.get(DPAD_R, DOWN)) robot.arm.out();
            else if (BC2.get(DPAD_L, DOWN)) robot.arm.in();
            else if (BC2.get(DPAD_UP, TAP)) {
                if (robot.arm.getStatus() == Arm.STATUS.UP) robot.arm.up2();
                else robot.arm.up();
            }

            //                  CLAW
            if (BC2.get(TRIANGLE, TOGGLE)) robot.claw.close();
            else robot.claw.open();

            //                INTAKE CODE
            current_intake_position = robot.intake.getIntakePosition();
            double intake_velocity = (current_intake_position - last_intake_position) / (current_nanoseconds - last_nanoseconds);

            if (BC2.get(RB1, TOGGLE) && BC2.get(CIRCLE, UP)) {
                // CHECK INTAKE STALLING
                if (abs(robot.intake.getIntakePower()) == 1 && abs(intake_velocity) < 0.01 && intake_time.seconds() < 1) {
                    intake_time.reset();
                    robot.intake.setIntakePower(-1);
                }
                else {
                    if (BC2.get(LB1, TOGGLE)) robot.intake.setIntakePower(-1);
                    else robot.intake.setIntakePower(1);
                }
            }
            else robot.intake.setIntakePower(0);


            // INTAKE ARM
            if (BC2.get(DPAD_DN, TOGGLE)) robot.intake.armUp();
            else robot.intake.armDown();


            // Square toggles aiming at goal or powershots
            if (BC2.get(SQUARE, TOGGLE))    aimTarget = POWERSHOTS;
            else                            aimTarget = GOAL;

            // Get degree error and correct
            double rpm = goalFinder.getRPM();
            double errorToGoal = (abs(robot.imu.getModAngle()) - 180);
            double goalDegreeError;
            double powerShotFieldAngle;
            goalDegreeError = goalFinder.getGoalDegreeError();
            powerShotFieldAngle = goalFinder.getPSDegreeError(powerShot, robot.imu.getAngle());
            multTelemetry.addData("Goal Error", goalDegreeError);
            multTelemetry.addData("PowerShot Field Angle", powerShotFieldAngle);
            multTelemetry.addData("180 Error", errorToGoal);
            multTelemetry.addData("RPM", rpm);


            // Slow down the robot
            velocity = 0.5;

            // SHOOTER
            robot.shooter.feederState(BC2.get(RB2, DOWN));
            if (BC2.get(CIRCLE, TOGGLE)) {
                robot.intake.armDown();

                // Check if Goal is found, if not, set RPM to default, and orient nearby goal
                if (errorToGoal > 30) {
                    locked_direction = findClosestAngle(180, robot.imu.getAngle());
                    rpm = 3400;
                }
                else {
                    // Choose correct target
                    if (aimTarget == GOAL){
                        locked_direction = findClosestAngle(robot.imu.getAngle() + goalDegreeError, robot.imu.getAngle());
                        rpm = goalFinder.getRPM();
                    }
                    else {
                        locked_direction = findClosestAngle(powerShotFieldAngle, robot.imu.getAngle());
                        rpm = goalFinder.getRPM() - 300;
                    }
                }

                // Set the RPM
                robot.shooter.setRPM(rpm);
            }
            else robot.shooter.setPower(0);





            /*

             ----------- S T E E R I N G    F U N C T I O N A L I T Y -----------

                                                                                */

            //           ABSOLUTE CONTROL MODE          //
            JC1.setShifted(RIGHT, (robot.imu.getAngle() - 90) % 360);
            if (BC1.get(SQUARE, TAP) && !square_pressed) {
                robot.imu.setOffsetAngle(-((robot.imu.getAngle() + 180) % 360));
                robot.imu.resetDeltaAngle();
                square_pressed = true;
            }

            //              DRIVER VALUES               //
            double drive = JC1.get(RIGHT, INVERT_SHIFTED_Y);
            double strafe = JC1.get(RIGHT, SHIFTED_X);
            double turn = JC1.get(LEFT, X);

            //              VELOCITY RANGER             //
            if (BC1.get(LB2, DOWN)) velocity = Range.clip((1 - gamepad1.left_trigger), 0.5, 1);
            else if (BC1.get(RB2, DOWN)) velocity = Range.clip((1 - gamepad1.right_trigger), 0.2, 1);

            //              DPAD AUTO TURN              //
            if (BC1.get(DPAD_UP, DOWN)) locked_direction            = findClosestAngle(90, robot.imu.getAngle());
            else if (BC1.get(DPAD_R, DOWN)) locked_direction        = findClosestAngle(0, robot.imu.getAngle());
            else if (BC1.get(DPAD_L, DOWN)) locked_direction        = findClosestAngle(180, robot.imu.getAngle());
            else if (BC1.get(DPAD_DN, DOWN)) locked_direction       = findClosestAngle(270, robot.imu.getAngle());

            //            POWER SHOT INCREMENT          //
            if (BC1.get(RB1, TAP)){
                if (powerShot == VisionUtils.PowerShot.PS_LEFT){
                    powerShot = VisionUtils.PowerShot.PS_MIDDLE;
                }
                else if (powerShot == VisionUtils.PowerShot.PS_MIDDLE){
                    powerShot = VisionUtils.PowerShot.PS_RIGHT;
                }
                else if (powerShot == VisionUtils.PowerShot.PS_RIGHT){
                    powerShot = VisionUtils.PowerShot.PS_LEFT;
                }
            }
            else if (BC1.get(LB1, TAP)) {
                if (powerShot == VisionUtils.PowerShot.PS_LEFT){
                    powerShot = VisionUtils.PowerShot.PS_RIGHT;
                }
                else if (powerShot == VisionUtils.PowerShot.PS_MIDDLE){
                    powerShot = VisionUtils.PowerShot.PS_LEFT;
                }
                else if (powerShot == VisionUtils.PowerShot.PS_RIGHT){
                    powerShot = VisionUtils.PowerShot.PS_MIDDLE;
                }
            }
            powerShotFieldAngle = goalFinder.getPSDegreeError(powerShot, robot.imu.getAngle());
            if (BC1.get(RB1, TAP) || BC1.get(LB1, TAP)) locked_direction = findClosestAngle(powerShotFieldAngle, robot.imu.getAngle());



            /*

            ----------- P I D -----------

                                       */

            if (JC1.get(LEFT, X) != 0) pid_on = false;
            else if (current_angular_velocity == 0.0) pid_on = true;

            if (pid_on && !last_cycle_pid_state) locked_direction = robot.imu.getAngle();
            else if (pid_on) turn = robot.rotationPID.update(locked_direction - current_angle) * -1;

            last_cycle_pid_state = pid_on;



            /*

            ----------- S E T    P O W E R -----------

                                                    */

            // Rounded angle
            double rounded_locked = round(abs(locked_direction) % 90);
            if (rounded_locked == 0) turn *= 0.5;
            robot.setDrivePower(drive, strafe, turn, velocity);



            /*

             ----------- L O G G I N G -----------

                                                */

            multTelemetry.addLine("--DRIVER-------------------------------------");
            multTelemetry.addData("PowerShot", powerShot);
            multTelemetry.addData("Aim Target", aimTarget);
            multTelemetry.addData("Angle", robot.imu.getAngle());
            multTelemetry.addData("Locked Angle", locked_direction);
            multTelemetry.addData("Rounded Locked Angle", rounded_locked);


            multTelemetry.addLine("--HARDWARE-------------------------------------");
            multTelemetry.addData("Intake Forward", (BC2.get(LB1, TOGGLE)) ? "FORWARD" : "REVERSE");
            multTelemetry.addData("Shooter", (BC2.get(CIRCLE, TOGGLE)) ? "ON" : "OFF");
            multTelemetry.addData("RPM", robot.shooter.getRPM());


            multTelemetry.update();




            /*
             ----------- S H U T D O W N -----------
                                                  */

            if (BC1.get(TOUCHPAD, DOWN) || BC2.get(TOUCHPAD, DOWN)){
                shutdown();
                break;
            }
        }
    }
}


