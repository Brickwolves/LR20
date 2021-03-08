package org.firstinspires.ftc.teamcode.TeleOp.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_ID;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_MIN;


@TeleOp(name = "OdTest", group="Linear TeleOp")
public class OdoDiag extends LinearOpMode {

    private Controller2 controller;
    private DcMotor od;
    private String od_id = "arm_encoder";
    private double position;

    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller2(gamepad1);

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


