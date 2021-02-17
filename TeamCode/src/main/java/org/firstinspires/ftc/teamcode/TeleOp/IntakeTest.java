package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LEFT_ARM_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LEFT_ARM_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LEFT_ARM_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.RIGHT_ARM_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.RIGHT_ARM_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.RIGHT_ARM_SERVO_MIN;


@TeleOp(name = "IntakeTest", group="Linear TeleOp")
public class IntakeTest extends LinearOpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;

    private Servo Servo2;
    private String servo2_id = "servo_2";

    private double Servo2_position = LEFT_ARM_SERVO_HOME;

    private Servo Servo3;
    private String servo3_id = "servo_3";

    private double Servo3_position = RIGHT_ARM_SERVO_HOME;




    public void initialize() {
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);


        Servo2 = Utils.hardwareMap.get(Servo.class, servo2_id);
        Servo2.setDirection(Servo.Direction.FORWARD);
        // SERVO_ARM: IF WE DON'T SET THE POSITION, SERVO STARTS AT 0.23 (halfway) instead of 0.0 (down)
        Servo2.setPosition(LEFT_ARM_SERVO_HOME);
        Servo3 = Utils.hardwareMap.get(Servo.class, servo3_id);
        Servo3.setDirection(Servo.Direction.FORWARD);
        // SERVO_ARM: IF WE DON'T SET THE POSITION, SERVO STARTS AT 0.23 (halfway) instead of 0.0 (down)
        Servo3.setPosition(RIGHT_ARM_SERVO_HOME);

        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        Servo2.setPosition(LEFT_ARM_SERVO_HOME);
        Servo3.setPosition(RIGHT_ARM_SERVO_HOME);

        sleep(3000);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            if(controller.src.dpad_up){
                Servo2_position = LEFT_ARM_SERVO_MIN;
                Servo3_position = RIGHT_ARM_SERVO_MAX;
            }
            else if(controller.src.dpad_down){
                Servo2_position= LEFT_ARM_SERVO_MAX;
                Servo3_position = RIGHT_ARM_SERVO_MIN;
            }


            Servo2_position = Range.clip(Servo2_position, Dash_ServoDiagnostic.LEFT_ARM_SERVO_MIN, Dash_ServoDiagnostic.LEFT_ARM_SERVO_MAX);
            Servo2.setPosition(Servo2_position);

            Utils.multTelemetry.addData("Servo ID", servo2_id);
            Utils.multTelemetry.addData("Servo Position", Servo2.getPosition());

            Servo3_position = Range.clip(Servo3_position, Dash_ServoDiagnostic.RIGHT_ARM_SERVO_MIN, Dash_ServoDiagnostic.RIGHT_ARM_SERVO_MAX);
            Servo3.setPosition(Servo3_position);

            Utils.multTelemetry.addData("Servo ID", servo3_id);
            Utils.multTelemetry.addData("Servo Position", Servo3.getPosition());

            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.right_bumper && controller.src.left_bumper) {
                shutdown();
                break;
            }
            }
        }
}


