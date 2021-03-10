package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

@Autonomous(name="OdoDiag", group="Autonomous Linear Opmode")
public class OdoDiag extends LinearOpMode {

    private DcMotor od;
    private ControllerCollin controller;
    private double start_pos, pos;

    public void breakpoint(){
        while (true){
            Utils.multTelemetry.addData("Status", "Holding");
            Utils.multTelemetry.update();
            if (controller.src.cross) break;
        }
        Utils.multTelemetry.addData("Status", "Continuing");
        Utils.multTelemetry.update();
    }

    public void initialize(){
        Utils.setOpMode(this);
        controller = new ControllerCollin(gamepad1);
        od = Utils.hardwareMap.get(DcMotor.class, "claw_encoder");
        start_pos = od.getCurrentPosition();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Utils.multTelemetry.addLine("Waiting for start");
        Utils.multTelemetry.update();
        waitForStart();

        while (opModeIsActive()){
            pos = od.getCurrentPosition();
            Utils.multTelemetry.addData("Position", pos);
            Utils.multTelemetry.update();
        }
    }
}
