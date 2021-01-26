package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumClawRobot;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static android.os.SystemClock.sleep;


@TeleOp(name = "ServoDiagnostic TeleOp", group="Linear TeleOp")
public class ServoDiagnosticTeleOp extends LinearOpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;

    private Servo servo;
    private String servo_id = "servo_0";

    private double SERVO_HOME = DashConstants.servo_home;
    private double SERVO_MIN_RANGE = DashConstants.servo_min;
    private double SERVO_MAX_RANGE = DashConstants.servo_max;
    private double SERVO_SPEED = 0.75;
    private double servo_position = SERVO_HOME;


    public void initialize() {
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);


        servo = Utils.hardwareMap.get(Servo.class, servo_id);
        servo.setDirection(Servo.Direction.FORWARD);
        // SERVO_ARM: IF WE DON'T SET THE POSITION, SERVO STARTS AT 0.23 (halfway) instead of 0.0 (down)
        servo.setPosition(SERVO_HOME);

        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        servo.setPosition(SERVO_HOME);
        sleep(3000);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            SERVO_MAX_RANGE = DashConstants.servo_max;
            SERVO_MIN_RANGE = DashConstants.servo_min;
            SERVO_HOME = DashConstants.servo_home;

            if (controller.src.triangle) {
                servo_position += SERVO_SPEED;
                Utils.multTelemetry.addData("Servo Status", "Forward");
            }
            if (controller.src.cross) {
                servo_position -= SERVO_SPEED;
                Utils.multTelemetry.addData("Servo Status", "Backward");
            }

            servo_position = Range.clip(servo_position, SERVO_MIN_RANGE, SERVO_MAX_RANGE);
            servo.setPosition(servo_position);

            Utils.multTelemetry.addData("Servo ID", servo_id);
            Utils.multTelemetry.addData("Servo Position", servo.getPosition());
            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.right_bumper && controller.src.left_bumper){
                shutdown();
                break;
            }
        }

    }
}


