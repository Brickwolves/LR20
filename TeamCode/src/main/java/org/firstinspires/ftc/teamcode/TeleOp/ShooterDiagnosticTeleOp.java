package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LOCK_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LOCK_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LOCK_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SHOOT_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SHOOT_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SHOOT_SERVO_MIN;

@TeleOp(name = "ShooterDiagnostic TeleOp", group="Linear TeleOp")
public class ShooterDiagnosticTeleOp extends LinearOpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;

    private Servo shoot_servo, lock_servo;
    private String status_shoot_servo = "HOME";
    private String status_lock_servo = "HOME";


    public void initialize() {
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);


        shoot_servo = Utils.hardwareMap.get(Servo.class, "servo_4");
        shoot_servo.setDirection(Servo.Direction.FORWARD);
        shoot_servo.setPosition(SHOOT_SERVO_HOME);

        lock_servo = Utils.hardwareMap.get(Servo.class, "servo_5");
        lock_servo.setDirection(Servo.Direction.FORWARD);
        lock_servo.setPosition(LOCK_SERVO_HOME);


        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        shoot_servo.setPosition(SHOOT_SERVO_HOME);
        lock_servo.setPosition(LOCK_SERVO_HOME);
        sleep(3000);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            controller.updateToggles();

            if (controller.LB1_toggle) {
                shoot_servo.setPosition(SHOOT_SERVO_MIN);
                status_shoot_servo = "MIN";
            }
            else {
                shoot_servo.setPosition(SHOOT_SERVO_MAX);
                status_shoot_servo = "MAX";
            }



            if (controller.RB1_toggle) {
                lock_servo.setPosition(LOCK_SERVO_MIN);
                status_lock_servo = "MIN";
            }
            else {
                lock_servo.setPosition(LOCK_SERVO_MAX);
                status_lock_servo = "MAX";
            }

            Utils.multTelemetry.addData("Shoot Position", shoot_servo.getPosition());
            Utils.multTelemetry.addData("Shoot Status", status_shoot_servo);
            Utils.multTelemetry.addData("LB1_Toggle", controller.LB1_toggle);

            Utils.multTelemetry.addData("", "");

            Utils.multTelemetry.addData("Lock Position", lock_servo.getPosition());
            Utils.multTelemetry.addData("Lock Status", status_lock_servo);
            Utils.multTelemetry.addData("RB1_Toggle", controller.RB1_toggle);

            Utils.multTelemetry.addData("", "");

            Utils.multTelemetry.addData("RTrig", controller.src.right_trigger > 0.75);

            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.right_bumper && controller.src.left_bumper){
                shutdown();
                break;
            }
        }

    }
}


