package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CRSERVO_POWER;

public class Gripper {

    private Servo arm;
    private CRServo claw;

    private String arm_id, claw_id;

    private final static double SERVO_ARM_HOME = 0.9;
    private final static double SERVO_ARM_MIN_RANGE = 0.0;
    private final static double SERVO_ARM_MAX_RANGE = 0.9;

    private String status = "IDLE";
    private ElapsedTime time;
    private STATE current_state;


    private enum STATE {
        IDLE,
        OPEN,
        CLOSE
    }

    public Gripper(String claw_id, String arm_id) {
        initArm();
        initClaw();
    }

    public void initArm(){
        arm = Utils.hardwareMap.get(Servo.class, arm_id);
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(SERVO_ARM_HOME);
    }

    public void initClaw(){
        claw = Utils.hardwareMap.get(CRServo.class, claw_id);
        claw.setDirection(CRServo.Direction.FORWARD);

        time = new ElapsedTime();
        current_state = STATE.IDLE;
    }

    public void clawMachine(boolean OPEN_BUTTON, boolean CLOSE_BUTTON, boolean STOP_BUTTON){

        switch (current_state) {

            case IDLE:
                status = "IDLE";
                claw.setPower(0);
                time.reset();

                if (OPEN_BUTTON) current_state = STATE.OPEN;
                else if (CLOSE_BUTTON) current_state = STATE.CLOSE;

                //Utils.multTelemetry.addData("Status", status);
                //Utils.multTelemetry.addData("Power", 0);
                break;

            case OPEN:
                status = "OPENING";
                if (STOP_BUTTON) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < Dash_CRServoDiag.millis_to_open) claw.setPower(CRSERVO_POWER);
                else current_state = STATE.IDLE;

                //Utils.multTelemetry.addData("Power", CRSERVO_POWER);
                //Utils.multTelemetry.addData("Status", status);
                break;

            case CLOSE:
                status = "CLOSING";
                if (STOP_BUTTON) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < Dash_CRServoDiag.millis_to_close) claw.setPower(-CRSERVO_POWER);
                else current_state = STATE.IDLE;

                //Utils.multTelemetry.addData("Power", -CRSERVO_POWER);
                //Utils.multTelemetry.addData("Status", status);
                break;
        }
    }

    public void setClawPower(double power){ claw.setPower(power); }


    public void armUp() { arm.setPosition(SERVO_ARM_MAX_RANGE); }
    public void armDown() { arm.setPosition(SERVO_ARM_MIN_RANGE); }
    public double getArmPosition(){ return arm.getPosition(); }

    public void setArmPosition(double position){
        double servo_arm_position = Range.clip(position, SERVO_ARM_MIN_RANGE, SERVO_ARM_MAX_RANGE);
        arm.setPosition(servo_arm_position);
    }

    public void shutdown(){
        arm.setPosition(SERVO_ARM_HOME);
        claw.setPower(0);
    }

}
