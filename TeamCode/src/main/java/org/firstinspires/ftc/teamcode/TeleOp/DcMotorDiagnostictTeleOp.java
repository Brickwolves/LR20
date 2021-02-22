package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LOCK_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LOCK_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LOCK_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SHOOT_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SHOOT_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SHOOT_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.p1;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter.p2;

@TeleOp(name = "DcMotor Diagnostic TeleOp", group="Linear TeleOp")
public class DcMotorDiagnostictTeleOp extends LinearOpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;

    private DcMotor shooter1, shooter2;
    private String shooter1_id = "spinny_1";
    private String shooter2_id = "spinny_2";


    private double shooter1_current_position = 0.0;
    private double shooter2_current_position = 0.0;
    private double shooter1_last_position = 0.0;
    private double shooter2_last_position = 0.0;

    private ElapsedTime time;


    public void initialize() {
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
        controller = new Controller(gamepad1);


        shooter1 = Utils.hardwareMap.get(DcMotor.class, shooter1_id);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2 = Utils.hardwareMap.get(DcMotor.class, shooter2_id);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

    public double getRPM(DcMotor shooter, double delta_time, double last_pos, double cur_pos){
        return (cur_pos - last_pos) / delta_time;
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
            shooter1_current_position = shooter1.getCurrentPosition();
            shooter2_current_position = shooter2.getCurrentPosition();


            shooter1.setPower(p1);
            shooter2.setPower(p2);



            double shooter1_RPM = getRPM(shooter1, delta_time, shooter1_last_position, shooter1_current_position);
            double shooter2_RPM = getRPM(shooter1, delta_time, shooter2_last_position, shooter2_current_position);



            shooter1_last_position = shooter1_current_position;
            shooter2_last_position = shooter2_current_position;


            /*
            shooter.feederState(controller.src.right_trigger > 0.75);
            if (controller1.circle_toggle) {
                robot.intake.armDown();
                //robot.shooter.setRPM(4500);
                robot.shooter.setPower(0.7);
            }
            else {
                robot.shooter.setPower(0);
                robot.shooter.setRPM(0);
            }

             */






            Utils.multTelemetry.addData("Shooter1 Position", shooter1.getCurrentPosition());
            Utils.multTelemetry.addData("Shooter2 Position", shooter2.getCurrentPosition());

            Utils.multTelemetry.addData("Shooter1 Power", shooter1.getPower());
            Utils.multTelemetry.addData("Shooter2 Power", shooter2.getPower());

            Utils.multTelemetry.addData("Shooter1 RPMillis", shooter1_RPM);
            Utils.multTelemetry.addData("Shooter2 RPMillis", shooter2_RPM);

            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.right_bumper && controller.src.left_bumper){
                shutdown();
                break;
            }
        }

    }
}


