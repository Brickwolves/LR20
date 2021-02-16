package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;


@TeleOp(name = "Gamma TeleOp - Scrimmage", group="Linear TeleOp")
public class GammaTeleOp extends LinearOpMode {

    private MecanumRobot robot;
    private Controller controller1;
    private Controller controller2;

    private double locked_direction;


    public void initialize() {
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

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
            if(controller2.RB2_toggle) {
                robot.intake.setPower(-1);
            }
            else robot.intake.setPower(0);
            robot.intake.update();

            // SHOOTER CODE
            if (controller2.LB1_toggle){
                robot.shooter.unlockFeeder();
            }
            else robot.shooter.lockFeeder();

            if (controller2.RB1_toggle){
                robot.shooter.feedRing();
            }
            else robot.shooter.resetFeeder();

            if (controller2.triangle_toggle){
                robot.shooter.setRPM(3500);
            }

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


