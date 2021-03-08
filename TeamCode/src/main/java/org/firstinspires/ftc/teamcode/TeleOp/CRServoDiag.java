package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;
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


    private ElapsedTime time;
    private double millis_to_open = 1000;
    private double millis_to_close = 1000;

    private enum STATE {
        IDLE,
        OPEN,
        CLOSE
    }
    private STATE current_state;

    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller2(gamepad1);


        servo = Utils.hardwareMap.get(CRServo.class, servo_id);
        servo.setDirection(CRServo.Direction.FORWARD);
        servo.setPower(0);

        time = new ElapsedTime();
        current_state = STATE.IDLE;

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


    public void clawMachine(){

        switch (current_state) {

            case IDLE:
                servo.setPower(0);
                time.reset();

                if (controller.src.dpad_up) current_state = STATE.OPEN;
                else if (controller.src.dpad_down) current_state = STATE.CLOSE;

                break;

            case OPEN:
                if (controller.src.square) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < millis_to_open) servo.setPower(0.5);
                else current_state = STATE.IDLE;
                break;

            case CLOSE:
                if (controller.src.square) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < millis_to_close) servo.setPower(-0.5);
                else current_state = STATE.IDLE;
                break;
        }

    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        double power = 0.0;
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {

            clawMachine();

            /*
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
            */
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


