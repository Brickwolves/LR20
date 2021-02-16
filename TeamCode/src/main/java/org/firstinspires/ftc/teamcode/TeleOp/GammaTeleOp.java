package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO2_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO2_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO2_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO3_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO3_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO3_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO4_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO5_HOME;


@TeleOp(name = "Gamma TeleOp - Scrimmage", group="Linear TeleOp")
public class GammaTeleOp extends LinearOpMode {

    private MecanumRobot robot;
    private Controller controller1;
    private Controller controller2;
    private Servo Servo2;
    private String servo2_id = "servo_2";
    private Servo Servo3;
    private String servo3_id = "servo_3";
    private Servo Servo4;
    private String servo4_id = "servo_4";
    private Servo Servo5;
    private String servo5_id = "servo_5";
    private double Servo2_position = SERVO2_HOME;
    private double Servo3_position = SERVO3_HOME;
    private double Servo4_position = SERVO4_HOME;
    private double Servo5_position = SERVO5_HOME;
    private double locked_direction;



    public void initialize() {
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        Servo2 = Utils.hardwareMap.get(Servo.class, servo2_id);
        Servo2.setDirection(Servo.Direction.FORWARD);
        Servo2.setPosition(SERVO2_HOME);
        Servo3 = Utils.hardwareMap.get(Servo.class, servo3_id);
        Servo3.setDirection(Servo.Direction.FORWARD);
        Servo3.setPosition(SERVO3_HOME);
        Servo4 = Utils.hardwareMap.get(Servo.class, servo4_id);
        Servo4.setDirection(Servo.Direction.FORWARD);
        Servo4.setPosition(SERVO4_HOME);
        Servo5 = Utils.hardwareMap.get(Servo.class, servo5_id);
        Servo5.setDirection(Servo.Direction.FORWARD);
        Servo5.setPosition(SERVO5_HOME);


        Utils.multTelemetry.addData("Steering Controls", "--------");
        Utils.multTelemetry.addData("Toggle ACM", "[Right Stick Btn]");
        Utils.multTelemetry.addData("Velocity Ranger", "[Left Trigger]");
        Utils.multTelemetry.addData("Face Direction", "DPAD");

        Utils.multTelemetry.addData("Hardware Controls", "--------");
        Utils.multTelemetry.addData("Claw", "[Circle]");
        Utils.multTelemetry.addData("Arm", "[Triangle]");
        Utils.multTelemetry.addData("Shutdown Keys", "[RB] & [LB] simultaneously");
        Utils.multTelemetry.update();

    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        robot.arm.shutdown();
        robot.claw.shutdown();
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {



        /*

         ----------- H A R D W A R E    F U N C T I O N A L I T Y -----------

         */
            controller1.updateToggles();
            controller2.updateToggles();

            // Arm functionality
            if (controller2.triangle_toggle) robot.arm.down();
            else robot.arm.up();

            // Claw Functionality
            if (controller2.circle_toggle) robot.claw.closeFull();
            else robot.claw.openFull();

            // INTAKE CODE
            if (controller2.RB1_toggle) {
                if (controller2.LB1_toggle) robot.intake.setPower(-1);
                else robot.intake.setPower(1);
            }
            else robot.intake.setPower(0);
            robot.intake.update();

            // SHOOTER CODE
            /*if (controller2.LB1_toggle){
                robot.shooter.unlockFeeder();
                robot.shooter.resetFeeder();
            }
            else robot.shooter.lockFeeder();

            if (controller2.RB1_toggle){
                robot.shooter.feedRing();
            }
            else robot.shooter.resetFeeder();*/
            robot.shooter.feederState(controller2.src.circle);


            if (controller2.triangle_toggle){
                robot.shooter.setRPM(4500);
            }
            //Intake Arm
            if(controller2.src.dpad_up){
                Servo2_position = SERVO2_MIN;
                Servo3_position = SERVO3_MAX;
            }
            else if(controller2.src.dpad_down){
                Servo2_position= SERVO2_MAX;
                Servo3_position = SERVO3_MIN;
            }
            Servo2_position = Range.clip(Servo2_position, Dash_ServoDiagnostic.SERVO2_MIN, Dash_ServoDiagnostic.SERVO2_MAX);
            Servo2.setPosition(Servo2_position);

            Servo3_position = Range.clip(Servo3_position, Dash_ServoDiagnostic.SERVO3_MIN, Dash_ServoDiagnostic.SERVO3_MAX);
            Servo3.setPosition(Servo3_position);

        /*

         ----------- S T E E R I N G    F U N C T I O N A L I T Y -----------

         */

            // Get Thumbsticks
            Controller.Thumbstick rightThumbstick = controller1.getRightThumbstick();
            Controller.Thumbstick leftThumbstick = controller1.getLeftThumbstick();

            // ABSOLUTE CONTROL MODE
            if (controller1.right_stick_btn_toggle) rightThumbstick.setShift(robot.imu.getAngle() % 360);
            else rightThumbstick.setShift(0);

            // DRIVER VALUES
            double drive = rightThumbstick.getInvertedShiftedY();
            double strafe = rightThumbstick.getShiftedX();
            double turn = leftThumbstick.getX();

            // VELOCITY RANGER
            double velocity = Range.clip((1 - controller1.src.left_trigger), 0.3, 1);


            // DPAD Auto Turn
            if (controller1.DPADPress()){
                if (controller1.src.dpad_up) locked_direction               = MecanumRobot.turnTarget(0, robot.imu.getAngle());
                else if (controller1.src.dpad_right) locked_direction       = MecanumRobot.turnTarget(-90, robot.imu.getAngle());
                else if (controller1.src.dpad_left) locked_direction        = MecanumRobot.turnTarget(90, robot.imu.getAngle());
                else if (controller1.src.dpad_down) locked_direction        = MecanumRobot.turnTarget(180, robot.imu.getAngle());
            }


            // LOCKED DIRECTION MODE
            if (controller1.src.left_stick_x != 0) locked_direction = robot.imu.getAngle();
            else turn = robot.rotationPID.update(locked_direction - robot.imu.getAngle()) * -1;

            robot.setDrivePower(drive * velocity, strafe * velocity, turn * 0.5, 1);





        /*

         ----------- L O G G I N G -----------

         */
            Utils.multTelemetry.addData("IMU", robot.imu.getAngle());
            Utils.multTelemetry.addData("Locked Direction", locked_direction);

            Utils.multTelemetry.addData("ACM", controller1.right_stick_btn_toggle);
            Utils.multTelemetry.addData("Drive", drive);
            Utils.multTelemetry.addData("Strafe", strafe);
            Utils.multTelemetry.addData("Turn", turn);
            Utils.multTelemetry.addData("Velocity", velocity);

            Utils.multTelemetry.addData("Arm", robot.arm.getPosition());
            Utils.multTelemetry.addData("Claw", controller2.circle_toggle);
            Utils.multTelemetry.update();




        /*

         ----------- S H U T D O W N -----------

         */

        if ((controller1.src.right_bumper && controller1.src.left_bumper) || (controller2.src.right_bumper && controller2.src.left_bumper)){
            shutdown();
            break;
        }


        }
    }
}


