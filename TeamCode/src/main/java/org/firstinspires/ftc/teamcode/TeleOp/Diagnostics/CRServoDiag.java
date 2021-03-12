package org.firstinspires.ftc.teamcode.TeleOp.Diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.STATE_MACHINE_ON;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CRSERVO_ID;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CRSERVO_POWER;

@Disabled
@TeleOp(name = "CRServoDiag TeleOp", group="Linear TeleOp")
public class CRServoDiag extends LinearOpMode {

    private ControllerCollin controller;

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
        controller = new ControllerCollin(gamepad1);

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
        Utils.multTelemetry.addData("MODE", claw_encoder.getMode());
        Utils.multTelemetry.update();
    }

    public void shutdown(){
        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        crServo.setPower(0);
    }


    public void clawMachine(){

        switch (current_state) {

            case IDLE:
                status = "IDLE";
                crServo.setPower(0);
                time.reset();

                if (controller.src.dpad_up) current_state = STATE.OPEN;
                else if (controller.src.dpad_down) current_state = STATE.CLOSE;
                Utils.multTelemetry.addData("Power", 0);


                break;

            case OPEN:
                status = "OPENING";
                if (controller.src.square) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < Dash_CRServoDiag.millis_to_open) crServo.setPower(CRSERVO_POWER);
                else current_state = STATE.IDLE;

                Utils.multTelemetry.addData("Power", CRSERVO_POWER);
                break;

            case CLOSE:
                status = "CLOSING";
                if (controller.src.square) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < Dash_CRServoDiag.millis_to_close) crServo.setPower(-CRSERVO_POWER);
                else current_state = STATE.IDLE;

                Utils.multTelemetry.addData("Power", -CRSERVO_POWER);
                break;
        }
    }


    @Override
    public void runOpMode() {


        initialize();
        waitForStart();

        boolean lastDPADUP = false;
        boolean lastDPADDOWN = false;
        boolean lastSQUARE = false;
        double power = 0.0;
        double duration = 0;
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {

            //clawMachine();
            arm.out();

            if (STATE_MACHINE_ON) clawMachine();

            else {
                if (controller.src.dpad_up) {
                    if (!lastDPADUP) time.reset();
                    power = CRSERVO_POWER;
                    status = "OPENING";
                }
                if (controller.src.dpad_down) {
                    if (!lastDPADDOWN) time.reset();
                    power = -CRSERVO_POWER;
                    status = "CLOSING";
                }
                if (controller.src.square) {
                    if (!lastSQUARE) duration = time.milliseconds();
                    power = 0;
                    status = "STOPPED";
                }
                crServo.setPower(power);
            }


            lastDPADUP = controller.src.dpad_up;
            lastDPADDOWN = controller.src.dpad_down;
            lastSQUARE = controller.src.square;





            Utils.multTelemetry.addData("Duration", duration);
            Utils.multTelemetry.addData("Encoder Position", claw_encoder.getCurrentPosition());
            Utils.multTelemetry.addData("Encoder Delta", start_position - claw_encoder.getCurrentPosition());

            Utils.multTelemetry.addData("CServo ID", CRSERVO_ID);
            Utils.multTelemetry.addData("Servo Status", status);
            Utils.multTelemetry.addData("Servo Power", crServo.getPower());
            Utils.multTelemetry.update();



            // S H U T D O W N     S E Q U E N C E

            if (controller.src.touchpad){
                shutdown();
                break;
            }
        }
    }
}


