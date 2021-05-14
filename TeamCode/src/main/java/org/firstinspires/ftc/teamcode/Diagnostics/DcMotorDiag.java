package org.firstinspires.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_Shooter.power;

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
        OpModeUtils.setOpMode(this);
        controller = new ControllerCollin(gamepad1);


        motor = OpModeUtils.hardwareMap.get(DcMotor.class, motor_id);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        OpModeUtils.multTelemetry.addData("Status", "Initialized");
        OpModeUtils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        OpModeUtils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        OpModeUtils.multTelemetry.update();
    }

    public void shutdown(){
        OpModeUtils.multTelemetry.addData("Status", "Shutting Down");
        OpModeUtils.multTelemetry.update();
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




            OpModeUtils.multTelemetry.addData("Motor Position", motor.getCurrentPosition());
            OpModeUtils.multTelemetry.addData("Motor Power", motor.getPower());

            OpModeUtils.multTelemetry.addData("Motor RPMillis", motor_RPM);

            OpModeUtils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.right_bumper && controller.src.left_bumper){
                shutdown();
                break;
            }
        }

    }
}


