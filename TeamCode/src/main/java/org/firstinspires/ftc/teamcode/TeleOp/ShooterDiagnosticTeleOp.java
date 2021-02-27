package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.rpm;

//@Disabled
@TeleOp(name = "Shooter Diagnostic TeleOp", group="Linear TeleOp")
public class ShooterDiagnosticTeleOp extends LinearOpMode {

    private MecanumRobot robot;
    private Controller2 controller;

    private String shooter1_id = "spinny_1";
    private String shooter2_id = "spinny_2";

    private ElapsedTime time;


    public void initialize() {
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        controller = new Controller2(gamepad1);


        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        sleep(3000);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();
        time = new ElapsedTime();

        while (opModeIsActive()) {
            controller.updateToggles();

            time.reset();
            double delta_time = time.milliseconds();

            robot.shooter.feederState(controller.src.right_trigger > 0.75);
            if (controller.circle_toggle) {
                robot.intake.armDown();

                //robot.shooter.setPower(shooter_power);
                robot.shooter.setRPM(rpm);
            }
            else {
                robot.shooter.setPower(0);
               // robot.shooter.setRPM(0);
            }







            Utils.multTelemetry.addData("RPM", robot.shooter.getRPM());
            Utils.multTelemetry.addData("Power", robot.shooter.getPower());
            Utils.multTelemetry.addData("Position", robot.shooter.getPosition());
            Utils.multTelemetry.addData("Imu", robot.imu.getAngle());

            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.touchpad){
                shutdown();
                break;
            }
        }

    }
}


