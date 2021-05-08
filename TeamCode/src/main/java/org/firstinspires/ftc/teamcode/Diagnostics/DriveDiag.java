package org.firstinspires.ftc.teamcode.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

@Disabled
@TeleOp(name = "Drive Diag TeleOp", group="Linear TeleOp")
public class DriveDiag extends LinearOpMode {

    private ControllerCollin controller;
    private ElapsedTime time;

    private DcMotor fr, fl, br, bl;


    public void initialize(){
        OpModeUtils.setOpMode(this);
        controller = new ControllerCollin(gamepad1);

        fr = OpModeUtils.hardwareMap.get(DcMotor.class, "front_right_motor");
        fl = OpModeUtils.hardwareMap.get(DcMotor.class, "front_left_motor");
        br = OpModeUtils.hardwareMap.get(DcMotor.class, "back_right_motor");
        bl = OpModeUtils.hardwareMap.get(DcMotor.class, "back_left_motor");
        resetMotors();

        OpModeUtils.multTelemetry.addData("Status", "Initialized");
        OpModeUtils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        OpModeUtils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        OpModeUtils.multTelemetry.update();
    }

    public void resetMotors(){
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutdown(){
        OpModeUtils.multTelemetry.addData("Status", "Shutting Down");
        OpModeUtils.multTelemetry.update();
        sleep(3000);
    }

    public double getPosition(){
        return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4.0;
    }

    public void setDrivePower(double drive, double strafe, double turn, double velocity) {
        fr.setPower((drive - strafe - turn) * velocity);
        fl.setPower((drive + strafe + turn) * velocity);
        br.setPower((drive + strafe - turn) * velocity);
        bl.setPower((drive - strafe + turn) * velocity);
    }

    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        while (opModeIsActive()) {
            controller.updateToggles();


            // Get Thumbsticks
            ControllerCollin.Thumbstick rightThumbstick = controller.getRightThumbstick();
            ControllerCollin.Thumbstick leftThumbstick = controller.getLeftThumbstick();

            // DRIVER VALUES
            double drive = -controller.src.right_stick_y;
            double strafe = controller.src.right_stick_x;
            double turn = leftThumbstick.getX();

            setDrivePower(drive, strafe, turn, 1);

            OpModeUtils.multTelemetry.addData("Position", getPosition());
            OpModeUtils.multTelemetry.addData("Drive", drive);
            OpModeUtils.multTelemetry.addData("Strafe", strafe);
            OpModeUtils.multTelemetry.addData("Turn", turn);
            OpModeUtils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.right_bumper && controller.src.left_bumper){
                shutdown();
                break;
            }
        }

    }
}


