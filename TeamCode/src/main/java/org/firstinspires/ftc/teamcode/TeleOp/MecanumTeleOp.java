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


@TeleOp(name = "Mecanum Drive", group="TeleOp")
public class MecanumTeleOp extends OpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;
    private Servo servo_1;
    public final static double SERVO_1_HOME = .83;
    public final static double SERVO_1_MIN_RANGE = 0.5;
    public final static double SERVO_1_MAX_RANGE = .83;
    public double SERVO_1_SPEED = 0.1;
    double servo1Position = SERVO_1_HOME;
    private Servo servo_2;
    public final static double SERVO_2_HOME = .7;
    public final static double SERVO_2_MIN_RANGE = 0.3;
    public final static double SERVO_2_MAX_RANGE = 0.7;
    public double SERVO_2_SPEED = 0.1;
    double servo2Position = SERVO_2_HOME;
    private Servo servo_3;
    public final static double SERVO_3_HOME = 0.23;
    public final static double SERVO_3_MIN_RANGE = 0.23;
    public final static double SERVO_3_MAX_RANGE = 0.34;
    public double SERVO_3_SPEED = 0.1;
    double servo3Position = SERVO_3_HOME;
    private Servo servo_4;
    public final static double SERVO_4_HOME = 0.0;
    public final static double SERVO_4_MIN_RANGE = 0.0;
    public final static double SERVO_4_MAX_RANGE = 1.0;
    public double SERVO_4_SPEED = 1;
    double servo4Position = SERVO_4_HOME;







    // Toggle Variables
    private boolean absoluteControlMode;
    private boolean DPAD_Toggle;
    private boolean LBLastCycle;
    private boolean RBLastCycle;
    private boolean CrossLastCycle;
    private boolean SquareLastCycle;
    private boolean velocityToggle;
    private boolean wow;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);
        Utils.setOpMode(this);

        servo_1 = Utils.hardwareMap.get(Servo.class, "servo_1");
        servo_1.setDirection(Servo.Direction.FORWARD);
        servo_2 = Utils.hardwareMap.get(Servo.class, "servo_2");
        servo_3 = Utils.hardwareMap.get(Servo.class, "servo_3");
        servo_4 = Utils.hardwareMap.get(Servo.class, "servo_4");


        // Toggles
        absoluteControlMode = false;
        DPAD_Toggle = false;
        LBLastCycle = false;
        RBLastCycle = false;
    }



    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // Get Thumbsticks
        Controller.Thumbstick rightThumbstick = controller.getRightThumbstick();
        Controller.Thumbstick leftThumbstick = controller.getLeftThumbstick();


        // ButtonTapped() compares current and previous states to handle single presses of buttons.
        // If we press RB once, toggle velocity shift
        RBLastCycle = Utils.buttonTapped(controller.src.right_bumper, RBLastCycle);
        if (RBLastCycle) velocityToggle = !velocityToggle;

        LBLastCycle = Utils.buttonTapped(controller.src.left_bumper, LBLastCycle);
        if (LBLastCycle) absoluteControlMode = !absoluteControlMode;

        // Check Absolute Control Mode
        if (absoluteControlMode) rightThumbstick.setShift(mecanumRobot.imu.getAngle() % 360);
        else rightThumbstick.setShift(0);


        // Set Driver Values
        double drive = rightThumbstick.getInvertedShiftedY();
        double strafe = rightThumbstick.getShiftedX();
        double turn = leftThumbstick.getX();
        double velocity = (velocityToggle) ? 0.5 : 1;

        // DPAD Auto Turn
        if (controller.DPADPress()){
            if (controller.src.dpad_up) mecanumRobot.turn(MecanumRobot.Direction.NORTH, 1);
            else if (controller.src.dpad_right) mecanumRobot.turn(MecanumRobot.Direction.EAST, 1);
            else if (controller.src.dpad_left) mecanumRobot.turn(MecanumRobot.Direction.WEST, 1);
            else if (controller.src.dpad_down) mecanumRobot.turn(MecanumRobot.Direction.SOUTH, 1);
        }

        else mecanumRobot.setDrivePower(drive, strafe, turn, velocity);

        /* touchpad?*/
        if (controller.src.touchpad){

        }
        if(gamepad1.cross){
            servo4Position += SERVO_4_SPEED;
        }
        if(gamepad1.square){
            servo4Position -= SERVO_4_SPEED;
        }
        if(gamepad1.triangle){
            servo2Position = .3;
            servo2Position = Range.clip(servo2Position, SERVO_2_MIN_RANGE, SERVO_2_MAX_RANGE);
            servo_2.setPosition(servo2Position);
            sleep(100);
            servo2Position = .7;
        }


        servo1Position = Range.clip(servo1Position, SERVO_1_MIN_RANGE, SERVO_1_MAX_RANGE);
        servo_1.setPosition(servo1Position);
        servo2Position = Range.clip(servo2Position, SERVO_2_MIN_RANGE, SERVO_2_MAX_RANGE);
        servo_2.setPosition(servo2Position);
        servo3Position = Range.clip(servo3Position, SERVO_3_MIN_RANGE, SERVO_3_MAX_RANGE);
        servo_3.setPosition(servo3Position);
        servo4Position = Range.clip(servo4Position, SERVO_4_MIN_RANGE, SERVO_4_MAX_RANGE);
        servo_4.setPosition(servo4Position);



        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("wow", wow);
        telemetry.addData("Servo1Position", servo_1.getPosition());
        telemetry.addData("Servo2Position", servo_2.getPosition());
        telemetry.addData("Servo3Position", servo_3.getPosition());
        telemetry.addData("Servo4Position", servo_4.getPosition());
        this.log();
    }



    /**
     * Logs out Telemetry Data
     */
    public void log(){
        telemetry.addData("IMU", mecanumRobot.imu.getAngle());
        //telemetry.addData("RGB", String.format("(%d, %d, %d)", mecanumRobot.colorSensor.red(), mecanumRobot.colorSensor.green(), mecanumRobot.colorSensor.blue()));
        telemetry.addData("Velocity Toggle", velocityToggle);
        telemetry.addData("ACM", absoluteControlMode);
        telemetry.addData("Error", mecanumRobot.imu.getStartAngle() - mecanumRobot.imu.getAngle());
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


