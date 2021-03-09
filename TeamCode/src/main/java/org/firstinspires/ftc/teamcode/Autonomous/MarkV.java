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
    private DcMotor claw_encoder;
    private Claw claw;

    private double robot_claw_pos, encoder_pos, claw_pos;

    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller2(gamepad1);

        robot = new MecanumRobot();
        claw_encoder = Utils.hardwareMap.get(DcMotor.class, "claw_encoder");
        claw = new Claw("claw", "claw_encoder", Claw.MODE.GLOBAL);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        Utils.multTelemetry.addLine("Waiting for start");
        Utils.multTelemetry.update();
        waitForStart();

        while (opModeIsActive()){
            robot_claw_pos  = robot.claw.getClawPosition();
            encoder_pos     = claw_encoder.getCurrentPosition();
            claw_pos        = claw.getClawPosition();
            Utils.multTelemetry.addData("Rob Claw Pos", robot_claw_pos);
            Utils.multTelemetry.addData("Encoder Pos", encoder_pos);
            Utils.multTelemetry.addData("Claw Pos", claw_pos);

            Utils.multTelemetry.update();
        }
    }
}
