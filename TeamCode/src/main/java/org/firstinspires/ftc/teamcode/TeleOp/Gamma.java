package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.RingBuffer;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.rpm;


@TeleOp(name = "Gamma TeleOp", group="Linear TeleOp")
public class Gamma extends LinearOpMode {

    // Main Stuff
    private MecanumRobot robot;
    private ControllerCollin controller1;
    private ControllerCollin controllerCollin;

    // Power Shot Angles
    private int     ps_increment = 2;
    private double  ps_delta_angle = 15;

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
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller1 = new ControllerCollin(gamepad1);
        controllerCollin = new ControllerCollin(gamepad2);



        /*
        Utils.multTelemetry.addData("USER 1", "----------------------------------");
        Utils.multTelemetry.addData("Toggle ACM", "[SQUARE]");
        Utils.multTelemetry.addData("Velocity Ranger", "[Left Trigger]");
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
         */

    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        robot.intake.shutdown();
        robot.arm.up();
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();
        waitForStart();
        elapsedTime = new ElapsedTime();


        while (opModeIsActive()) {

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
            controller1.updateToggles();
            controllerCollin.updateToggles();


            // ARM
            if (controllerCollin.src.dpad_right){
                robot.arm.out();
            }
            else if (controllerCollin.src.dpad_up) {
                robot.arm.up();
            }
            else if (controllerCollin.src.dpad_left) {
                robot.arm.in();
            }

            if (controllerCollin.src.triangle){
                robot.claw.open();
            }
            else if (controllerCollin.src.cross){
                robot.claw.close();
            }
            else if (controllerCollin.src.square){
                robot.claw.stop();
            }


            // INTAKE CODE
            current_intake_position = robot.intake.getIntakePosition();
            double intake_velocity = (current_intake_position - last_intake_position) / (current_nanoseconds - last_nanoseconds);

            if (controllerCollin.RB1_toggle && !controllerCollin.src.circle) {

                // CHECK INTAKE STALLING
                if (abs(robot.intake.getIntakePower()) == 1 && abs(intake_velocity) < 0.01 && intake_time.seconds() < 1) {
                    intake_time.reset();
                    robot.intake.setIntakePower(-1);
                }
                else {
                    if (controllerCollin.LB1_toggle) robot.intake.setIntakePower(-1);
                    else robot.intake.setIntakePower(1);
                }
            }
            else robot.intake.setIntakePower(0);



            // INTAKE ARM
            if (controllerCollin.DPADDWN_toggle && !controllerCollin.src.circle) robot.intake.armUp();
            else if (!controllerCollin.DPADDWN_toggle && !controllerCollin.src.circle) robot.intake.armDown();


            // SHOOTER
            robot.shooter.feederState(controller1.src.right_trigger > 0.75);
            if (controllerCollin.circle_toggle) {
                //robot.intake.armDown();
                //robot.shooter.setRPM(4500);
                robot.shooter.setRPM(rpm);
            }
            else {
                robot.shooter.setPower(0);
            }






        /*

         ----------- S T E E R I N G    F U N C T I O N A L I T Y -----------

         */

            // Get Thumbsticks
            ControllerCollin.Thumbstick rightThumbstick = controller1.getRightThumbstick();
            ControllerCollin.Thumbstick leftThumbstick = controller1.getLeftThumbstick();

            // ABSOLUTE CONTROL MODE
            if (controller1.square_toggle) rightThumbstick.setShift(robot.imu.getAngle() % 360);
            else rightThumbstick.setShift(0);

            // DRIVER VALUES
            double drive = rightThumbstick.getInvertedShiftedY();
            double strafe = rightThumbstick.getShiftedX();
            double turn = leftThumbstick.getX();

            // VELOCITY RANGER
            double velocity = Range.clip((1 - controller1.src.left_trigger), 0.5, 1);


            // DPAD Auto Turn
            if (controller1.src.dpad_up) locked_direction               = MecanumRobot.turnTarget(0, robot.imu.getAngle());
            else if (controller1.src.dpad_right) locked_direction       = MecanumRobot.turnTarget(-90, robot.imu.getAngle());
            else if (controller1.src.dpad_left) locked_direction        = MecanumRobot.turnTarget(90, robot.imu.getAngle());
            else if (controller1.src.dpad_down) locked_direction        = MecanumRobot.turnTarget(180, robot.imu.getAngle());




            // Power Shot increment
            if (controller1.RB1_tap){
                if (ps_increment == 2) ps_increment = 0;
                else ps_increment++;
            }
            else if (controller1.LB1_tap) {
                if (ps_increment == 0) ps_increment = 2;
                else ps_increment--;
            }
            if (controller1.RB1_tap || controller1.LB1_tap)
                if (ps_increment == 0)      locked_direction = MecanumRobot.turnTarget(-ps_delta_angle, robot.imu.getAngle());
                else if (ps_increment == 1) locked_direction = MecanumRobot.turnTarget(0, robot.imu.getAngle());
                else if (ps_increment == 2) locked_direction = MecanumRobot.turnTarget(ps_delta_angle, robot.imu.getAngle());


            /*

                    | |   ______       _____      ________
                    | |  |_   __ \    |_   _|    |_   ___ `.
                    | |    | |__) |     | |        | |   `. \
                    | |    |  ___/      | |        | |    | |
                    | |   _| |_        _| |_      _| |___.' /
                    | |  |_____|      |_____|    |________.'
                    | |

             */
            if (controller1.src.left_stick_x != 0) {
                pid_on = false;
            }
            else if (current_angular_velocity == 0.0 ) {
                pid_on = true;
            }

            if (pid_on && !last_cycle_pid_state) {
                locked_direction = robot.imu.getAngle();
            }
            else if (pid_on) {
                turn = robot.rotationPID.update(locked_direction - current_angle) * -1;
            }
            last_cycle_pid_state = pid_on;




            // LAST STEP
            robot.setDrivePower(drive, strafe, turn, velocity);


        /*

         ----------- L O G G I N G -----------

         */

        /*
            Utils.multTelemetry.addData("RPM", rpm);
            Utils.multTelemetry.addData("PID OFF", pid_on);

            Utils.multTelemetry.addData("ACM", controller1.right_stick_btn_toggle);
            Utils.multTelemetry.addData("Angle", robot.imu.getAngle());
            Utils.multTelemetry.addData("Locked Angle", locked_direction);
            Utils.multTelemetry.addData("PS Increment", ps_increment);

            Utils.multTelemetry.addData("Time", current_nanoseconds);
            Utils.multTelemetry.addData("Turn", turn);
            Utils.multTelemetry.addData("Velocity Constant", velocity);
            Utils.multTelemetry.addData("Angular Velocity", current_angular_velocity);

            Utils.multTelemetry.addData("", "");

            Utils.multTelemetry.addData("HARDWARE", "---------------------------------------");
            Utils.multTelemetry.addData("Arm", robot.arm.getPosition());
            Utils.multTelemetry.addData("Claw", controller2.circle_toggle);
            Utils.multTelemetry.addData("Intake ON?", controller2.RB1_toggle);
            Utils.multTelemetry.addData("Intake Forward", controller2.LB1_toggle);
            Utils.multTelemetry.addData("Intake Left Arm", robot.intake.getLeftServoPosition());
            Utils.multTelemetry.addData("Intake Right Arm", robot.intake.getRightServoPosition());
            Utils.multTelemetry.addData("Shooter ON?", controller2.circle_toggle);
        */


            Utils.multTelemetry.update();




        /*

         ----------- S H U T D O W N -----------

         */

        if ((controller1.src.touchpad) || (controllerCollin.src.touchpad)){
            shutdown();
            break;
        }


        }
    }
}


