package org.firstinspires.ftc.teamcode.TeleOp.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Shooter.power;

@Disabled
@TeleOp(name = "DcMotor Diag TeleOp", group="Linear TeleOp")
public class DcMotorDiag extends LinearOpMode {

    private ControllerCollin controller;

    private DcMotor motor;
    private String motor_id = "front_right_motor";


    private double motor_current_position = 0.0;
    private double motor_last_position = 0.0;
    private double motor_RPM = 0.0;

    private ElapsedTime time;


    public void initialize() {
        Utils.setOpMode(this);
        controller = new ControllerCollin(gamepad1);


        motor = Utils.hardwareMap.get(DcMotor.class, motor_id);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            motor_current_position = motor.getCurrentPosition();

            motor_RPM = getRPM(motor, delta_time, motor_last_position, motor_current_position);

            motor_last_position = motor_current_position;


            motor.setPower(power);




            Utils.multTelemetry.addData("Motor Position", motor.getCurrentPosition());
            Utils.multTelemetry.addData("Motor Power", motor.getPower());

            Utils.multTelemetry.addData("Motor RPMillis", motor_RPM);

            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.right_bumper && controller.src.left_bumper){
                shutdown();
                break;
            }
        }

    }
}


