package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumClawRobot;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Diagnostic Claw TeleOp", group="TeleOp")
public class DiagnosticClawTeleOp extends OpMode {

    private MecanumRobot mecanumClawRobot;
    private Controller controller;
    private Servo servo_ring_lock, servo_ring_pusher, servo_claw, servo_arm;
/*
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
*/
    public final static double SERVO_ARM_HOME = 0.0;
    public final static double SERVO_ARM_MIN_RANGE = 0.0;
    public final static double SERVO_ARM_MAX_RANGE = 0.35;
    public double SERVO_ARM_SPEED = 0.75;
    double servo_arm_position = SERVO_ARM_HOME;


    // Toggle Variables
    private boolean velocityToggle;
    private boolean absolute_control_mode;
    private boolean claw_toggle;
    private boolean DPAD_Toggle;
    private boolean RBLastCycle;
    private boolean LBLastCycle;
    private boolean CircleLastCycle;
    private boolean CrossLastCycle;
    private boolean SquareLastCycle;
    private boolean wow;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        Utils.setOpMode(this);
        mecanumClawRobot = new MecanumRobot();
        controller = new Controller(gamepad1);

        servo_arm = Utils.hardwareMap.get(Servo.class, "servo_4");
        servo_claw = Utils.hardwareMap.get(Servo.class, "servo_3");

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
        if (RBLastCycle) {
            velocityToggle = !velocityToggle;
            sleep(200);
        }

        LBLastCycle = Utils.buttonTapped(controller.src.left_bumper, LBLastCycle);
        if (LBLastCycle) {
            absolute_control_mode = !absolute_control_mode;
            sleep(200);
        }

        CircleLastCycle = Utils.buttonTapped(controller.src.circle, CircleLastCycle);
        if (CircleLastCycle) {
            claw_toggle = !claw_toggle;
            sleep(200);
        }


        // Check claw_toggle
        if (claw_toggle) mecanumClawRobot.claw.openFull();
        else mecanumClawRobot.claw.closeFull();

        // Check Absolute Control Mode
        if (absolute_control_mode) rightThumbstick.setShift(mecanumClawRobot.imu.getAngle() % 360);
        else rightThumbstick.setShift(0);


        // Set Driver Values
        double drive = rightThumbstick.getInvertedShiftedY();
        double strafe = rightThumbstick.getShiftedX();
        double turn = leftThumbstick.getX();
        double velocity = (velocityToggle) ? 0.5 : 1;

        // DPAD Auto Turn
        if (controller.DPADPress()){
            if (controller.src.dpad_up) mecanumClawRobot.turn(MecanumRobot.Direction.NORTH, 1);
            else if (controller.src.dpad_right) mecanumClawRobot.turn(MecanumRobot.Direction.EAST, 1);
            else if (controller.src.dpad_left) mecanumClawRobot.turn(MecanumRobot.Direction.WEST, 1);
            else if (controller.src.dpad_down) mecanumClawRobot.turn(MecanumRobot.Direction.SOUTH, 1);
        }
        else mecanumClawRobot.setDrivePower(drive, strafe, turn, velocity);


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


        servo_arm_position = Range.clip(servo_arm_position, SERVO_ARM_MIN_RANGE, SERVO_ARM_MAX_RANGE);
        servo_arm.setPosition(servo_arm_position);




        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("wow", wow);
        telemetry.addData("Servo3Position", servo_claw.getPosition());
        telemetry.addData("Servo4Position", servo_arm.getPosition());
        this.log();
    }



    /**
     * Logs out Telemetry Data
     */
    public void log(){
        telemetry.addData("IMU", mecanumClawRobot.imu.getAngle());
        //telemetry.addData("RGB", String.format("(%d, %d, %d)", mecanumRobot.colorSensor.red(), mecanumRobot.colorSensor.green(), mecanumRobot.colorSensor.blue()));
        telemetry.addData("Velocity Toggle", velocityToggle);
        telemetry.addData("ACM", absolute_control_mode);
        telemetry.addData("Error", mecanumClawRobot.imu.getStartAngle() - mecanumClawRobot.imu.getAngle());
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


