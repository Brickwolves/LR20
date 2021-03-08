package org.firstinspires.ftc.teamcode.TeleOp.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Controls.Controller2;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CRSERVO_ID;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CRSERVO_POWER;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.MOE;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.STATE_MACHINE_ON;


@TeleOp(name = "ClawDiag", group="Linear TeleOp")
public class ClawDiag extends LinearOpMode {

    private Controller2 controller;

    private Arm arm;
    private DcMotor claw_encoder;
    private CRServo crServo;
    private String status = "IDLE";

    private ElapsedTime time;
    private double start_position = 0;

    private enum STATE {
        IDLE,
        OPEN,
        CLOSE
    }
    private STATE current_state;

    public void initialize() {
        Utils.setOpMode(this);
        controller = new Controller2(gamepad1);

        arm = new Arm("arm");

        claw_encoder = Utils.hardwareMap.get(DcMotor.class, "claw_encoder");
        start_position = claw_encoder.getCurrentPosition();

        crServo = Utils.hardwareMap.get(CRServo.class, CRSERVO_ID);
        crServo.setDirection(CRServo.Direction.FORWARD);

        time = new ElapsedTime();
        current_state = STATE.IDLE;

        Utils.multTelemetry.addData("Status", "Initialized");
        Utils.multTelemetry.addData("Start Keys", "Press [>] to begin");
        Utils.multTelemetry.addData("Shutdown Keys", "Press [RB] & [LB] simultaneously");
        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        crServo.setPower(0);
        arm.up();
        sleep(2000);
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        double power = 0.0;

        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {

            arm.out();


            double claw_position = claw_encoder.getCurrentPosition();
            double delta2Open = Math.abs(claw_position - OPEN_POSITION);
            double delta2Close = Math.abs(claw_position - CLOSE_POSITION);

            if (controller.src.dpad_up) {

                if (delta2Open > MOE){
                    power = CRSERVO_POWER;
                    status = "OPENING";
                }
                else status = "CANNOT OPEN ANYMORE";

            } else if (controller.src.dpad_down) {

                if (delta2Close > MOE){
                    power = -CRSERVO_POWER;
                    status = "CLOSING";
                }
                else status = "CANNOT CLOSE ANYMORE";

            } else if (controller.src.square) {
                power = 0;
                status = "STOPPED";
            }
            crServo.setPower(power);

            Utils.multTelemetry.addData("Encoder Position", claw_encoder.getCurrentPosition());
            Utils.multTelemetry.addData("Delta2Open", delta2Open);
            Utils.multTelemetry.addData("Delta2Close", delta2Close);


            Utils.multTelemetry.addData("CServo ID", CRSERVO_ID);
            Utils.multTelemetry.addData("Servo Status", status);
            Utils.multTelemetry.addData("Servo Power", crServo.getPower());
            Utils.multTelemetry.update();


            // S H U T D O W N     S E Q U E N C E

            if (controller.src.touchpad) {
                shutdown();
                break;
            }
        }
    }
}


