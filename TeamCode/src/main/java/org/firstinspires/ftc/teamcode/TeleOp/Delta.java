package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Controls.Joystick;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.RingBuffer;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.rpm;

// Controller Imports
import static org.firstinspires.ftc.teamcode.Hardware.Controls.Controller.Input.*;



@TeleOp(name = "Delta TeleOp - Scrimmage", group="Linear TeleOp")
public class Delta extends LinearOpMode {

    private MecanumRobot robot;
    private Controller controller1, controller2;

    // Power Shot Angles
    private int     ps_increment = 0;
    private double  ps_delta_angle = 5;

    // PID Stuff
    private boolean pid_on = false;
    private boolean last_cycle_pid_state = true;
    private boolean shutdownReady = false;
    private boolean LB1_TOGGLE = false;

    private ElapsedTime elapsedTime, intake_time;

    private RingBuffer<Double> angle_ring_buffer = new RingBuffer<>(5, 0.0);
    private RingBuffer<Double> time_ring_buffer = new RingBuffer<>(5,  0.0);


    private double
        current_nanoseconds, last_nanoseconds,
        current_angle, last_angle, locked_direction,
        velocity, current_angular_velocity,
        current_intake_position, last_intake_position, intake_velocity;


    // JOYSTICKS
    private Joystick rightStick, leftStick;


    @RequiresApi(api = Build.VERSION_CODES.N)
    public void initialize() {
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        rightStick = new Joystick(gamepad1.right_stick_x, gamepad1.right_stick_y);
        leftStick = new Joystick(gamepad2.left_stick_x, gamepad2.left_stick_y);

        setUser1Controls();
        setUser2Controls();

        showInstructions();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void setUser1Controls(){

        controller1.setTapAction(TOUCHPAD, this::shutdown);
        controller1.setTapAction(TRIANGLE, this::cycleNextPowerShot);

        // ABSOLUTE CONTROL MODE
        controller1.setOnAction(SQUARE, () -> rightStick.setShift(robot.imu.getAngle() % 360));
        controller1.setOffAction(SQUARE, () -> rightStick.setShift(0));

        controller1.setTapAction(DPAD_UP, () -> { locked_direction = MecanumRobot.turnTarget(0, robot.imu.getAngle()); });
        controller1.setTapAction(DPAD_R, () -> { locked_direction = MecanumRobot.turnTarget(-90, robot.imu.getAngle()); });
        controller1.setTapAction(DPAD_L, () -> { locked_direction = MecanumRobot.turnTarget(90, robot.imu.getAngle()); });
        controller1.setTapAction(DPAD_DN, () -> { locked_direction = MecanumRobot.turnTarget(180, robot.imu.getAngle()); });
    }



    public void setUser2Controls(){
        controller2.setOnAction(X, () -> robot.claw.closeFull());
        controller2.setOffAction(X, () -> robot.claw.openFull());

        controller2.setTapAction(DPAD_UP, () -> robot.intake.armUp());
        controller2.setTapAction(DPAD_DN, () -> robot.intake.armDown());

        controller2.setTapAction(CIRCLE, () -> robot.shooter.setPower(rpm));
        controller2.setIdleAction(CIRCLE, () -> robot.shooter.setPower(0));
        controller2.setOnAction(CIRCLE, () -> robot.arm.down());

        controller2.setOnAction(TRIANGLE, () -> robot.arm.down());
        controller2.setOffAction(TRIANGLE, () -> robot.arm.up());
        controller2.setTapAction(L_BUMPER, () -> LB1_TOGGLE = !LB1_TOGGLE);
        controller2.setOnAction(R_BUMPER, () -> {
            if (!controller2.src.circle) {
                // CHECK INTAKE STALLING
                if (abs(robot.intake.getIntakePower()) == 1 && abs(intake_velocity) < 0.01 && intake_time.seconds() < 1) {
                    intake_time.reset();
                    robot.intake.setIntakePower(-1);
                } else {
                    if (LB1_TOGGLE) robot.intake.setIntakePower(-1);
                    else robot.intake.setIntakePower(1);
                }
            } else robot.intake.setIntakePower(0);
        });

    }



    public void showInstructions(){
        Utils.multTelemetry.addData("USER 1", "----------------------------------");
        Utils.multTelemetry.addData("Toggle ACM", "[R BUMPER]");
        Utils.multTelemetry.addData("Velocity Ranger", "[L TRIGGER]");
        Utils.multTelemetry.addData("Face Direction", "DPAD");
        Utils.multTelemetry.addData("Shoot", "[R TRIGGER]");
        Utils.multTelemetry.addData("Power Shot Increment", "[TRIANGLE]");

        Utils.multTelemetry.addData("", "");
        Utils.multTelemetry.addData("USER 2", "----------------------------------");
        Utils.multTelemetry.addData("Claw", "[CROSS]");
        Utils.multTelemetry.addData("Arm", "[TRIANGLE]");
        Utils.multTelemetry.addData("Intake ON/OFF", "[RB1]");
        Utils.multTelemetry.addData("Intake Direction", "[LB1]");
        Utils.multTelemetry.addData("Intake Arm Up", "[DPAD UP]");
        Utils.multTelemetry.addData("Intake Arm Down", "[DPAD DOWN]");
        Utils.multTelemetry.addData("Shooter ON/OFF", "[CIRCLE]");

        Utils.multTelemetry.addData("Shutdown Keys", "[TOUCHPAD] simultaneously");
        Utils.multTelemetry.update();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void cycleNextPowerShot(){
        if (ps_increment == 0)      locked_direction = MecanumRobot.turnTarget(-ps_delta_angle, robot.imu.getAngle());
        else if (ps_increment == 1) locked_direction = MecanumRobot.turnTarget(0, robot.imu.getAngle());
        else {
            locked_direction = MecanumRobot.turnTarget(ps_delta_angle, robot.imu.getAngle());
            ps_increment = -1;
        }
        ps_increment++;
    }


    public void log() {
        Utils.multTelemetry.addData("PID OFF", pid_on);

        Utils.multTelemetry.addData("Angle", current_angle);
        Utils.multTelemetry.addData("Locked Angle", locked_direction);
        Utils.multTelemetry.addData("PS Increment", ps_increment);

        Utils.multTelemetry.addData("Time", current_nanoseconds);
        Utils.multTelemetry.addData("Velocity Constant", velocity);
        Utils.multTelemetry.addData("Angular Velocity", current_angular_velocity);

        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        robot.arm.shutdown();
        robot.claw.shutdown();
        robot.intake.shutdown();

        shutdownReady = true;
    }

    public void updateVelocity(){
        current_nanoseconds = elapsedTime.nanoseconds();
        current_angle = robot.imu.getAngle();

        double delta_angle = current_angle - angle_ring_buffer.getValue(current_angle);
        double delta_time = current_nanoseconds - time_ring_buffer.getValue(current_nanoseconds);

        double current_angular_velocity = delta_angle / delta_time;

        last_nanoseconds = current_nanoseconds;
        last_angle = current_angle;
    }

    public void updateIntakeVelocity(){
        current_intake_position = robot.intake.getIntakePosition();
        intake_velocity = (current_intake_position - last_intake_position) / (current_nanoseconds - last_nanoseconds);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();
        waitForStart();
        elapsedTime = new ElapsedTime();
        intake_time = new ElapsedTime();


        while (opModeIsActive()) {

            updateVelocity();
            updateIntakeVelocity();


            Controller.update();
            robot.shooter.feederState(gamepad1.right_trigger > 0.75);


        /*

         ----------- S T E E R I N G    F U N C T I O N A L I T Y -----------

         */

            // Get Joysticks
            rightStick = new Joystick(gamepad1.right_stick_x, gamepad1.right_stick_y);
            leftStick = new Joystick(gamepad2.left_stick_x, gamepad2.left_stick_y);


            // DRIVER VALUES
            double drive = rightStick.getInvertedShiftedY();
            double strafe = rightStick.getShiftedX();
            double turn = leftStick.getX();

            // VELOCITY RANGER
            double velocity = Range.clip((1 - controller1.src.left_trigger), 0.5, 1);


            // Let robot strafe to a stop and then turn on PID
            if (leftStick.getX() != 0) pid_on = false;
            else if (current_angular_velocity == 0.0) pid_on = true;


            // When PID kicks back in, set the correct angle!
            if (pid_on && !last_cycle_pid_state) locked_direction = current_angle;
            else if (pid_on) turn = robot.rotationPID.update(locked_direction - current_angle) * -1;
            last_cycle_pid_state = pid_on;

            robot.setDrivePower(drive, strafe, turn, velocity);




            /*
                        -------- M I S C ---------
            */

            log();

            if (shutdownReady) {
                break;
            }
        }
    }
}


