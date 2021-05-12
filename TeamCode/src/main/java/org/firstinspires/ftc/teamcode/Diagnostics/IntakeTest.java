package org.firstinspires.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Intake.INTAKE_RPM_FORWARDS;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Intake.INTAKE_RPM_REVERSE;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Shooter.goal_rpm;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

@TeleOp(name = "IntakeTest TeleOp", group="Linear TeleOp")
public class IntakeTest extends LinearOpMode {

    private Mecanum robot;
    private ButtonControls BC;

    public void initialize() {
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);
    }

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {
            ButtonControls.update();

            robot.intake.armDown();

            //                INTAKE CODE
            if (BC.get(RB1, TOGGLE)) {
                if (BC.get(LB1, DOWN)) robot.intake.setRPM(INTAKE_RPM_REVERSE);
                else robot.intake.setRPM(INTAKE_RPM_FORWARDS);
            }
            else robot.intake.setIntakePower(0);



            multTelemetry.addData("RPM", robot.intake.getRPM());
            multTelemetry.addData("Position", robot.intake.getIntakePosition());
            multTelemetry.update();

        }
    }
}


