package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Controller.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static java.lang.Math.abs;


@TeleOp(name = "Weird TeleOp - Scrimmage", group="Linear TeleOp")
public class WeirdTeleOp extends LinearOpMode {

    // Main Stuff
    private MecanumRobot robot;
    private Controller controller1;

    private double  locked_direction = 0;


    public void initialize() {
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller1 = new Controller(gamepad1);
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        robot.arm.shutdown();
        robot.claw.shutdown();
        robot.intake.shutdown();
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();
        waitForStart();


        while (opModeIsActive()) {

            controller1.updateToggles();

            double current_angle = robot.imu.getAngle();
            double turn = robot.rotationPID.update(locked_direction - current_angle) * -1;

            // DPAD Auto Turn
            if (gamepad1.dpad_up) locked_direction               = MecanumRobot.turnTarget(0, current_angle);
            else if (gamepad1.dpad_right) locked_direction       = MecanumRobot.turnTarget(-90, current_angle);
            else if (gamepad1.dpad_left) locked_direction        = MecanumRobot.turnTarget(90, current_angle);
            else if (gamepad1.dpad_down) locked_direction        = MecanumRobot.turnTarget(180, current_angle);

            robot.setDrivePower(0, 0, turn, 1);


            Utils.multTelemetry.addData("Current Angle", current_angle);
            Utils.multTelemetry.addData("Locked Angle", locked_direction);
            Utils.multTelemetry.addData("Turn", turn);
            Utils.multTelemetry.update();
        }
    }
}


