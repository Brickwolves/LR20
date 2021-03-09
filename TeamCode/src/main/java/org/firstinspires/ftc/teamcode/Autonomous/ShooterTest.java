package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.ps_rpm;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.rings;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.rpm;

@Autonomous(name="ShooterTest", group="Autonomous Linear Opmode")
public class ShooterTest extends LinearOpMode {

    private MecanumRobot robot;
    private Controller2 controller;
    private ElapsedTime time;

    public void initialize(){
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller = new Controller2(gamepad1);
        time = new ElapsedTime();
    }

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        if (opModeIsActive()){

            robot.strafe(0, 40, 90, 0.1, null);
            robot.strafe(-90, 16, 90, 0.1, null);

            time.reset();
            while (true) {
                robot.intake.armDown();
                robot.shooter.setRPM(rpm);

                if (time.seconds() > 4) {
                    if (robot.shooter.feederCount() < 3) robot.shooter.feederState(true);
                    else break;
                }

                Utils.multTelemetry.addData("Position", robot.shooter.getPosition());
                Utils.multTelemetry.update();
            }
        }

        /*
        while (opModeIsActive()){
            controller.updateToggles();

            robot.shooter.feederState(controller.src.right_trigger > 0.75);
            if (controller.circle_toggle) {
                robot.intake.armDown();
                robot.shooter.setRPM(ps_rpm);
            }
            else {
                robot.shooter.setPower(0);
            }

            Utils.multTelemetry.addData("Position", robot.shooter.getPosition());
            Utils.multTelemetry.update();
        }
         */
    }
}
