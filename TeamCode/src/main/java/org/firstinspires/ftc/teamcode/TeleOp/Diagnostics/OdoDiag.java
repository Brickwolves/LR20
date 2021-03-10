package org.firstinspires.ftc.teamcode.TeleOp.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.Utils;


@TeleOp(name = "OdTest", group="Linear TeleOp")
public class OdoDiag extends LinearOpMode {

    private ControllerCollin controller;
    private DcMotor od;
    private String od_id = "arm_encoder";
    private double position;

    public void initialize() {
        Utils.setOpMode(this);
        controller = new ControllerCollin(gamepad1);

        od = Utils.hardwareMap.get(DcMotor.class, od_id);

        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.update();
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            position = od.getCurrentPosition();

            Utils.multTelemetry.addData("Encoder ID", od_id);
            Utils.multTelemetry.addData("Position", position);
            Utils.multTelemetry.update();
        }
    }
}


