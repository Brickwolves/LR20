package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Mecanum TeleOp", group="TeleOp")
public class MecanumTeleOp extends OpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;


    private int buttonWaitSeconds = 200;

    // Toggle Modes
    private boolean absolute_control_mode;
    private boolean velocityToggle;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);
    }



    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Loop Running");

        // Get Thumbsticks
        Controller.Thumbstick rightThumbstick = controller.getRightThumbstick();
        Controller.Thumbstick leftThumbstick = controller.getLeftThumbstick();



        // Update the toggles
        controller.updateToggles();



        // If we press RB once, toggle velocity shift
        if (controller.RBLastCycle) {
            velocityToggle = !velocityToggle;
            sleep(buttonWaitSeconds);
        }

        if (controller.LBLastCycle) {
            absolute_control_mode = !absolute_control_mode;
            sleep(buttonWaitSeconds);
        }


        // TOGGLE FUNCTIONALITY
        if (absolute_control_mode) rightThumbstick.setShift(mecanumRobot.imu.getAngle() % 360);
        else rightThumbstick.setShift(0);




        // Set Driver Values
        double drive = rightThumbstick.getInvertedShiftedY();
        double strafe = rightThumbstick.getShiftedX();
        double turn = leftThumbstick.getX();
        double velocity = (velocityToggle) ? 0.5 : 1;

        // DPAD Auto Turn
        if (controller.DPADPress()){
            if (controller.src.dpad_up) turn = mecanumRobot.turn2Direction(0, 1);
            else if (controller.src.dpad_right) turn = mecanumRobot.turn2Direction(-90, 1);
            else if (controller.src.dpad_left) turn = mecanumRobot.turn2Direction(90, 1);
            else if (controller.src.dpad_down) turn = mecanumRobot.turn2Direction(180, 1);
        }
        mecanumRobot.setDrivePower(drive, strafe, turn, velocity);



        /* TELEMETRY */
        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("IMU", mecanumRobot.imu.getAngle());
        telemetry.addData("Velocity Toggle", velocityToggle);
        telemetry.addData("ACM", absolute_control_mode);
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {}

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {}
}


