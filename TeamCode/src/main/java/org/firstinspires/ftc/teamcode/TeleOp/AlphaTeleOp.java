package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;


@TeleOp(name = "Alpha TeleOp", group="Linear TeleOp")
public class AlphaTeleOp extends LinearOpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;


    private int buttonWaitSeconds = 200;

    // Toggle Modes
    private boolean absolute_control_mode = false;
    private boolean velocityToggle = false;
    private boolean locked_mode = false;
    private double locked_direction = 0;

    private boolean claw_toggle;


    public void initialize() {
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);

        Utils.multTelemetry.addData("Steering Controls", "--------");
        Utils.multTelemetry.addData("Toggle ACM", "[LB]");
        Utils.multTelemetry.addData("Toggle Velocity", "[RB]");
        Utils.multTelemetry.addData("Toggle Locked Direction", "[Square]");
        Utils.multTelemetry.addData("Face Direction", "DPAD");

        Utils.multTelemetry.addData("Hardware Controls", "--------");
        Utils.multTelemetry.addData("Toggle Claw", "[Circle]");
        Utils.multTelemetry.addData("Arm Up/Down", "[Triangle] / [Cross]");
        Utils.multTelemetry.addData("Shutdown Keys", "[RB] & [LB] simultaneously");
        Utils.multTelemetry.update();

    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        mecanumRobot.arm.shutdown();
        mecanumRobot.claw.shutdown();
        sleep(3000);
    }



    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {



        /*

         ----------- H A R D W A R E    F U N C T I O N A L I T Y -----------

         */

            controller.updateToggles();

            // If we press RB once, toggle velocity shift
            if (controller.RB1_tap) {
                velocityToggle = !velocityToggle;
                sleep(buttonWaitSeconds);
            }

            if (controller.LB1_tap) {
                absolute_control_mode = !absolute_control_mode;
                sleep(buttonWaitSeconds);
            }

            if (controller.square_tap){
                locked_mode = !locked_mode;
                if (locked_mode) locked_direction = mecanumRobot.imu.getAngle();
                sleep(buttonWaitSeconds);
            }

            if (controller.circle_tap){
                claw_toggle = !claw_toggle;
                sleep(buttonWaitSeconds);
            }


            // Arm functionality
            if (controller.src.triangle){
                mecanumRobot.arm.up();
            }
            if (controller.src.cross){
                mecanumRobot.arm.down();
            }


            // Claw Functionality
            if (claw_toggle) mecanumRobot.claw.openFull();
            else mecanumRobot.claw.closeFull();


            // Intake Functionality
            /*
            if (controller.src.triangle){
                mecanumRobot.intake.setPower(1);
            }
            if (controller.src.cross){
                mecanumRobot.intake.setPower(-1);
            }
            if (controller.src.right_stick_button){
                mecanumRobot.intake.setPower(0);
            }
            mecanumRobot.intake.update();

            */

        /*

         ----------- S T E E R I N G    F U N C T I O N A L I T Y -----------

         */

            // Get Thumbsticks
            Controller.Thumbstick rightThumbstick = controller.getRightThumbstick();
            Controller.Thumbstick leftThumbstick = controller.getLeftThumbstick();

            if (absolute_control_mode) rightThumbstick.setShift(mecanumRobot.imu.getAngle() % 360);
            else rightThumbstick.setShift(0);

            // Set Driver Values
            double drive = rightThumbstick.getInvertedShiftedY(); Utils.multTelemetry.addData("Drive1", rightThumbstick.getInvertedShiftedY());
            double strafe = rightThumbstick.getShiftedX();
            double turn = leftThumbstick.getX();
            double velocity = (velocityToggle) ? 0.2 : 1;



            if (locked_mode) turn = mecanumRobot.rotationPID.update(locked_direction - mecanumRobot.imu.getAngle()) * -1;


            // DPAD Auto Turn
            if (controller.DPADPress()){
                //if (controller.src.dpad_up) turn = mecanumRobot.turn2Direction(0, 1);
                //else if (controller.src.dpad_right) turn = mecanumRobot.turn2Direction(-90, 1);
                //else if (controller.src.dpad_left) turn = mecanumRobot.turn2Direction(90, 1);
                //else if (controller.src.dpad_down) turn = mecanumRobot.turn2Direction(180, 1);
            }
            mecanumRobot.setDrivePower(drive * velocity, strafe * velocity, turn * 0.5, 1);





        /*

         ----------- L O G G I N G -----------

         */
            Utils.multTelemetry.addData("Drive", drive);
            Utils.multTelemetry.addData("Strafe", strafe);
            Utils.multTelemetry.addData("Turn", turn);
            Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());

            Utils.multTelemetry.addData("Velocity Toggle", velocityToggle);
            Utils.multTelemetry.addData("ACM", absolute_control_mode);
            Utils.multTelemetry.addData("Locked Direction", locked_mode);

            Utils.multTelemetry.addData("Arm", mecanumRobot.arm.getPosition());
            Utils.multTelemetry.addData("Claw", claw_toggle);
            Utils.multTelemetry.update();




        /*

         ----------- S H U T D O W N -----------

         */

        if (controller.src.right_bumper && controller.src.left_bumper){
            shutdown();
            break;
        }


        }
    }
}


