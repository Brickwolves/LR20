package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Diagnostic Mecanum Drive", group="TeleOp")
public class DiagnosticTeleOp extends OpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;
    private Servo servo_ring_lock, servo_ring_pusher, servo_claw, servo_arm;

    public final static double SERVO_RING_LOCK_HOME = .83;
    public final static double SERVO_RING_LOCK_MIN_RANGE = 0.5;
    public final static double SERVO_RING_LOCK_MAX_RANGE = .83;
    public double SERVO_RING_LOCK_SPEED = 0.1;
    double servo_ring_lock_position = SERVO_RING_LOCK_HOME;

    public final static double SERVO_RING_PUSHER_HOME = .7;
    public final static double SERVO_RING_PUSHER_MIN_RANGE = 0.3;
    public final static double SERVO_RING_PUSHER_MAX_RANGE = 0.7;
    public double SERVO_RING_PUSHER_SPEED = 0.1;
    double servo_ring_pusher_position = SERVO_RING_PUSHER_HOME;

    public final static double SERVO_CLAW_HOME = 0.23;
    public final static double SERVO_CLAW_MIN_RANGE = 0.23;
    public final static double SERVO_CLAW_MAX_RANGE = 0.34;
    public double SERVO_CLAW_SPEED = 0.1;

    public final static double SERVO_ARM_HOME = 0.0;
    public final static double SERVO_ARM_MIN_RANGE = 0.0;
    public final static double SERVO_ARM_MAX_RANGE = 1.0;
    public double SERVO_ARM_SPEED = 1;
    double servo_arm_position = SERVO_ARM_HOME;


    // Toggle Variables
    private boolean     velocityToggle,     absolute_control_mode,  claw_toggle,         DPAD_Toggle;
    private boolean     RBLastCycle,        LBLastCycle,            CircleLastCycle,    CrossLastCycle, SquareLastCycle;
    private boolean wow;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);
        Utils.setOpMode(this);

        servo_ring_lock = Utils.hardwareMap.get(Servo.class, "servo_1");
        servo_ring_pusher = Utils.hardwareMap.get(Servo.class, "servo_2");
        servo_claw = Utils.hardwareMap.get(Servo.class, "servo_3");
        servo_arm = Utils.hardwareMap.get(Servo.class, "servo_4");

        servo_ring_lock.setDirection(Servo.Direction.FORWARD);
        servo_ring_pusher.setDirection(Servo.Direction.FORWARD);
        servo_claw.setDirection(Servo.Direction.FORWARD);
        servo_arm.setDirection(Servo.Direction.FORWARD);


        // Toggles
        absolute_control_mode = false;
        DPAD_Toggle = false;
        LBLastCycle = false;
        RBLastCycle = false;
        CrossLastCycle = false;
        claw_toggle = false;
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
        if (LBLastCycle) absolute_control_mode = !absolute_control_mode;

        CircleLastCycle = Utils.buttonTapped(controller.src.cross, CircleLastCycle);
        if (CircleLastCycle) claw_toggle = !claw_toggle;

        // Check claw_toggle
        if (claw_toggle) servo_arm.setPosition(SERVO_ARM_MAX_RANGE);
        else servo_arm.setPosition(SERVO_ARM_MIN_RANGE);

        // Check Absolute Control Mode
        if (absolute_control_mode) rightThumbstick.setShift(mecanumRobot.imu.getAngle() % 360);
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


        /*
            Cross, the principal symbol of the Christian religion,
            recalling the Crucifixion of Jesus Christ and the redeeming
            benefits of his Passion and death. The cross is thus a sign
            both of Christ himself and of the faith of Christians.

            Therefore the cross makes arm ascend to heaven
            boop beep boop
         */
        if(gamepad1.cross){
            servo_arm_position += SERVO_ARM_SPEED;
        }
        /*
            To know the true meaning of what it means to be a square, one must go back to its origins.
            Square is a transliteration of what is originally an Irish term (dating back to the 1500s) —
            ’S cóir é (pron. s’cór æ) — It is fair (play); It is honest; therefore moral.
            It’s Irish opposite is ’S cam é — It is a trick; It is a fraud; therefore immoral.
            In other words, scam is the antonym of square. It was brought with the Irish to America.

            In this sense, square is used in terms like fair and square, square shooter and square dealer
            (a person who will speak or deal with you truthfully and honestly),
            square deal (a deal one can trust to be honest and fair for all involved),
            and square meal (an unpretentious, balanced, and honestly nourishing meal).
            The fact that some sources attempt to associate the latter with square plates
            used in the Royal Navy instead of acknowledging the Irish, is utterly preposterous and laughable.

            And that's why square make arm go boop beep boop down, to level the odds and keep things fair and square
         */
        if(gamepad1.square){
            servo_arm_position -= SERVO_ARM_SPEED;
        }
        if(gamepad1.triangle){
            servo_ring_pusher_position = .3;
            servo_ring_pusher_position = Range.clip(servo_ring_pusher_position, SERVO_RING_PUSHER_MIN_RANGE, SERVO_RING_PUSHER_MAX_RANGE);
            servo_ring_pusher.setPosition(servo_ring_pusher_position);
            sleep(100);
            servo_ring_pusher_position = .7;
        }


        servo_ring_lock_position = Range.clip(servo_ring_lock_position, SERVO_RING_LOCK_MIN_RANGE, SERVO_RING_LOCK_MAX_RANGE);
        servo_ring_lock.setPosition(servo_ring_lock_position);
        servo_ring_pusher_position = Range.clip(servo_ring_pusher_position, SERVO_RING_PUSHER_MIN_RANGE, SERVO_RING_PUSHER_MAX_RANGE);
        servo_ring_pusher.setPosition(servo_ring_pusher_position);
        servo_arm_position = Range.clip(servo_arm_position, SERVO_ARM_MIN_RANGE, SERVO_ARM_MAX_RANGE);
        servo_arm.setPosition(servo_arm_position);



        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("wow", wow);
        telemetry.addData("Servo1Position", servo_ring_lock.getPosition());
        telemetry.addData("Servo2Position", servo_ring_pusher.getPosition());
        telemetry.addData("Servo3Position", servo_claw.getPosition());
        telemetry.addData("Servo4Position", servo_arm.getPosition());
        this.log();
    }



    /**
     * Logs out Telemetry Data
     */
    public void log(){
        telemetry.addData("IMU", mecanumRobot.imu.getAngle());
        //telemetry.addData("RGB", String.format("(%d, %d, %d)", mecanumRobot.colorSensor.red(), mecanumRobot.colorSensor.green(), mecanumRobot.colorSensor.blue()));
        telemetry.addData("Velocity Toggle", velocityToggle);
        telemetry.addData("ACM", absolute_control_mode);
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


