package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

@Autonomous(name="MarkV", group="Autonomous Linear Opmode")
public class MarkV extends LinearOpMode {

    private Controller2 controller;
    private MecanumRobot robot;
    private DcMotor encoder;

    private double claw_pos;

    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller2(gamepad1);

        robot = new MecanumRobot();
        encoder = Utils.hardwareMap.get(DcMotor.class, "claw_encoder");
    }

    @Override
    public void runOpMode() {
        initialize();

        Utils.multTelemetry.addLine("Waiting for start");
        Utils.multTelemetry.update();
        waitForStart();

        while (opModeIsActive()){
            claw_pos        = robot.claw.getClawPosition();
            Utils.multTelemetry.addData("Claw Pos", claw_pos);


            robot.claw.clawMachine(controller.src.dpad_up, controller.src.dpad_down, controller.src.square, encoder);


            Utils.multTelemetry.update();
        }
    }
}
