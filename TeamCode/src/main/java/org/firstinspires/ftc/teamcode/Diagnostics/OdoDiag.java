package org.firstinspires.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;


@Disabled
@TeleOp(name = "OdTest", group="Linear TeleOp")
public class OdoDiag extends LinearOpMode {

    private ControllerCollin controller;
    private DcMotor od;
    private String od_id = "arm_encoder";
    private double position;

    public void initialize() {
        OpModeUtils.setOpMode(this);
        controller = new ControllerCollin(gamepad1);

        od = OpModeUtils.hardwareMap.get(DcMotor.class, od_id);

        OpModeUtils.multTelemetry.addData("Status", "Initialized");
        OpModeUtils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        OpModeUtils.multTelemetry.update();
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            position = od.getCurrentPosition();

            OpModeUtils.multTelemetry.addData("Encoder ID", od_id);
            OpModeUtils.multTelemetry.addData("Position", position);
            OpModeUtils.multTelemetry.update();
        }
    }
}


