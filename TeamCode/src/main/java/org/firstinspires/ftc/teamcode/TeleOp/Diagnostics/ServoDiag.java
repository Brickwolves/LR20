package org.firstinspires.ftc.teamcode.TeleOp.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.*;


@TeleOp(name = "ServoDiag TeleOp", group="Linear TeleOp")
public class ServoDiag extends LinearOpMode {

    private Controller2 controller;

    private Servo servo;
    private String servo_id = SERVO_ID;
    private String status = "HOME";


    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller2(gamepad1);


        servo = Utils.hardwareMap.get(Servo.class, servo_id);
        servo.setDirection(Servo.Direction.FORWARD);
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

            if (controller.src.dpad_up) {
                servo.setPosition(SERVO_MAX);
                status = "MAX";
            }
            if (controller.src.dpad_down) {
                servo.setPosition(SERVO_MIN);
                status = "MIN";
            }

            Utils.multTelemetry.addData("Servo ID", servo_id);
            Utils.multTelemetry.addData("Servo Status", status);
            Utils.multTelemetry.addData("Servo Position", servo.getPosition());
            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.touchpad){
                shutdown();
                break;
            }
        }

    }
}


