package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;


@TeleOp(name = "IntakeDiagnostic TeleOp", group="Linear TeleOp")
public class IntakeDiagnosticTeleOp extends LinearOpMode {

    private Controller controller;
    private Intake intake;
    private String motor_id = "intake";
    private DcMotor motor;


    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller(gamepad1);
        //motor = Utils.hardwareMap.get(DcMotor.class, motor_id);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setDirection(DcMotor.Direction.FORWARD);
        intake = new Intake(motor_id);


        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.addData("Forward", "Press [Triangle]");
        Utils.multTelemetry.addData("Backward", "Press [X]");
        Utils.multTelemetry.addData("Stop", "Press [Right Stick Button]");
        Utils.multTelemetry.update();
    }

    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            if (controller.src.triangle) {
                intake.setPower(1);
                Utils.multTelemetry.addData("Servo Status", "Forward");
            }
            if (controller.src.cross) {
                intake.setPower(-1);
                Utils.multTelemetry.addData("Servo Status", "Backward");
            }
            if (controller.src.right_stick_button) {
                intake.setPower(0);
                Utils.multTelemetry.addData("Servo Status", "Stopped");
            }
            intake.update();
            Utils.multTelemetry.addData("Motor ID", motor_id);
            Utils.multTelemetry.update();

        }
    }
}


