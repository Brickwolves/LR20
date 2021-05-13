package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Shooter.goal_rpm;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.findClosestAngle;

@Disabled
@TeleOp(name = "Gamma TeleOp", group="Linear TeleOp")
public class Gamma extends LinearOpMode {

    // Main Stuff
    private Mecanum robot;
    private ControllerCollin controller1;
    private ControllerCollin controller2;

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
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        controller1 = new ControllerCollin(gamepad1);
        controller2 = new ControllerCollin(gamepad2);


        OpModeUtils.multTelemetry.addLine("------USER 1----------------------------");
        OpModeUtils.multTelemetry.addData("Velocity Ranger", "[LB2]");
        OpModeUtils.multTelemetry.addData("Face Direction", "DPAD");
        OpModeUtils.multTelemetry.addData("Power Shot Increment", "[TRIANGLE]");

        OpModeUtils.multTelemetry.addLine("");

        OpModeUtils.multTelemetry.addLine("------USER 2----------------------------");
        OpModeUtils.multTelemetry.addData("Claw", "[TRIANGLE]");
        OpModeUtils.multTelemetry.addData("Intake ON/OFF", "[RB1]");
        OpModeUtils.multTelemetry.addData("Intake Direction", "[LB1]");
        OpModeUtils.multTelemetry.addData("Bumper Toggle", "[DPAD DOWN]");
        OpModeUtils.multTelemetry.addData("Arm Out", "[DPAD RIGHT]");
        OpModeUtils.multTelemetry.addData("Arm Up", "[DPAD UP]");
        OpModeUtils.multTelemetry.addData("Arm In", "[DPAD LEFT]");
        OpModeUtils.multTelemetry.addData("Shooter ON/OFF", "[CIRCLE]");
        OpModeUtils.multTelemetry.addData("Shoot", "[RB2]");

        OpModeUtils.multTelemetry.addData("Shutdown Keys", "[TOUCHPAD] simultaneously");
        OpModeUtils.multTelemetry.update();

    }

    public void shutdown(){
        OpModeUtils.multTelemetry.addData("Status", "Shutting Down");
        OpModeUtils.multTelemetry.update();
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

            controller1.updateToggles();
            controller2.updateToggles();


            // ARM
            if (controller2.src.dpad_right) robot.arm.out();
            else if (controller2.src.dpad_up) robot.arm.up();
            else if (controller2.src.dpad_left) robot.arm.in();

            // CLAW
            if (controller2.triangle_toggle) robot.claw.open();
            else robot.claw.close();


            // INTAKE CODE
            current_intake_position = robot.intake.getIntakePosition();
            double intake_velocity = (current_intake_position - last_intake_position) / (current_nanoseconds - last_nanoseconds);

            if (controller2.RB1_toggle) {

                // CHECK INTAKE STALLING
                if (abs(robot.intake.getIntakePower()) == 1 && abs(intake_velocity) < 0.01 && intake_time.seconds() < 1) {
                    intake_time.reset();
                    robot.intake.setIntakePower(-1);
                }
                else {
                    if (controller2.LB1_toggle) robot.intake.setIntakePower(-1);
                    else robot.intake.setIntakePower(1);
                }
            }
            else robot.intake.setIntakePower(0);



            // INTAKE ARM
            if (controller2.DPADDWN_toggle) robot.intake.armUp();
            else if (!controller2.DPADDWN_toggle) robot.intake.armDown();


            // SHOOTER
            robot.shooter.feederState(controller2.src.right_trigger > 0.75);
            if (controller2.circle_toggle) {
                //robot.intake.armDown();
                //robot.shooter.setRPM(4500);
                robot.shooter.setRPM(goal_rpm);
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
            rightThumbstick.setShift(robot.imu.getAngle() % 360);

            // DRIVER VALUES
            double drive = rightThumbstick.getInvertedShiftedY();
            double strafe = rightThumbstick.getShiftedX();
            double turn = leftThumbstick.getX();

            // VELOCITY RANGER
            double velocity = Range.clip((1 - controller1.src.left_trigger), 0.5, 1);


            // DPAD Auto Turn
            if (controller1.src.dpad_up) locked_direction               = findClosestAngle(0, robot.imu.getAngle());
            else if (controller1.src.dpad_right) locked_direction       = findClosestAngle(-90, robot.imu.getAngle());
            else if (controller1.src.dpad_left) locked_direction        = findClosestAngle(90, robot.imu.getAngle());
            else if (controller1.src.dpad_down) locked_direction        = findClosestAngle(180, robot.imu.getAngle());


            if (controller1.src.circle) locked_direction = findClosestAngle(110, robot.imu.getAngle());

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
                if (ps_increment == 0)      locked_direction = findClosestAngle(-ps_delta_angle + 90, robot.imu.getAngle());
                else if (ps_increment == 1) locked_direction = findClosestAngle(0 + 90, robot.imu.getAngle());
                else if (ps_increment == 2) locked_direction = findClosestAngle(ps_delta_angle + 90, robot.imu.getAngle());


            /*

            ----------- P I D -----------

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

            OpModeUtils.multTelemetry.addLine("--DRIVER-------------------------------------");
            OpModeUtils.multTelemetry.addData("Angle", robot.imu.getAngle());
            OpModeUtils.multTelemetry.addData("Locked Angle", locked_direction);
            OpModeUtils.multTelemetry.addData("PS Increment", ps_increment);

            OpModeUtils.multTelemetry.addLine("--HARDWARE-------------------------------------");
            OpModeUtils.multTelemetry.addData("Intake Forward", controller2.LB1_toggle);
            OpModeUtils.multTelemetry.addData("Shooter ON?", controller2.circle_toggle);

            OpModeUtils.multTelemetry.update();




            /*
             ----------- S H U T D O W N -----------
                                                  */

            if ((controller1.src.touchpad) || (controller2.src.touchpad)){
                shutdown();
                break;
            }


        }
    }
}


