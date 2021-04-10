package org.firstinspires.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_ServoDiagnostic.*;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_DN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.TOUCHPAD;


@TeleOp(name = "ServoDiag TeleOp", group="Linear TeleOp")
public class ServoDiag extends LinearOpMode {

    private ButtonControls BC;

    private Servo servo;
    private String servo_id = SERVO_ID;
    private String status = "HOME";


    public void initialize() {
        OpModeUtils.setOpMode(this);
        BC = new ButtonControls(gamepad1);


        servo = OpModeUtils.hardwareMap.get(Servo.class, servo_id);
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(SERVO_HOME);

        OpModeUtils.multTelemetry.addData("Status", "Initialized");
        OpModeUtils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        OpModeUtils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        OpModeUtils.multTelemetry.update();
    }

    public void shutdown(){
        OpModeUtils.multTelemetry.addData("Status", "Shutting Down");
        OpModeUtils.multTelemetry.update();
        servo.setPosition(SERVO_HOME);
        sleep(3000);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                servo.setPosition(SERVO_MAX);
                status = "MAX";
            }
            if (gamepad1.dpad_down) {
                servo.setPosition(SERVO_MIN);
                status = "MIN";
            }

            OpModeUtils.multTelemetry.addData("Servo ID", servo_id);
            OpModeUtils.multTelemetry.addData("Servo Status", status);
            OpModeUtils.multTelemetry.addData("Servo Position", servo.getPosition());
            OpModeUtils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (BC.get(TOUCHPAD, DOWN)){
                shutdown();
                break;
            }
        }

    }
}


