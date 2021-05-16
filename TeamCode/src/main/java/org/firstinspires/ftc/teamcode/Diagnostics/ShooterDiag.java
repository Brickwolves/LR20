package org.firstinspires.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_Shooter.goal_rpm;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

@Disabled
@TeleOp(name = "Shooter Diagnostic TeleOp", group="Linear TeleOp")
public class ShooterDiag extends LinearOpMode {

    private Mecanum robot;
    private ControllerCollin controller;

    public void initialize() {
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        controller = new ControllerCollin(gamepad1);
    }

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {
            controller.updateToggles();

            robot.shooter.feederState(controller.src.right_trigger > 0.75);
            if (controller.circle_toggle) {
                robot.intake.rollerMid();
                robot.shooter.setRPM(goal_rpm);
            }
            else {
                robot.shooter.setPower(0);
            }



            multTelemetry.addData("RPM", robot.shooter.getRPM());
            multTelemetry.addData("Power", robot.shooter.getPower());
            multTelemetry.addData("Position", robot.shooter.getPosition());
            multTelemetry.addData("Imu", robot.imu.getAngle());

            multTelemetry.update();

        }
    }
}


