package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.CRSERVO_POWER;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_MIN;


@TeleOp(name = "CRServoDiag TeleOp", group="Linear TeleOp")
public class CRServoDiag extends LinearOpMode {

    private Controller2 controller;

    private CRServo servo;
    private String servo_id = "claw";
    private String status = "HOME";


    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller2(gamepad1);


        servo = Utils.hardwareMap.get(CRServo.class, servo_id);
        servo.setDirection(CRServo.Direction.FORWARD);
        servo.setPower(0);

        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        servo.setPower(0);
        sleep(3000);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        double power = 0.0;
        while (opModeIsActive()) {

            if (controller.src.dpad_up) {
                power = CRSERVO_POWER;
                status = "OPENING";
            }
            if (controller.src.dpad_down) {
                power = CRSERVO_POWER * -1;
                status = "CLOSING";
            }
            if (controller.src.square) {
                power = 0;
                status = "STOPPED";
            }
            servo.setPower(power);

            Utils.multTelemetry.addData("Servo ID", servo_id);
            Utils.multTelemetry.addData("Servo Status", status);
            Utils.multTelemetry.addData("Servo Power", servo.getPower());
            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.touchpad){
                shutdown();
                break;
            }
        }

    }
}


