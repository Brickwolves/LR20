package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Utils;


import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CRSERVO_POWER;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.MOE;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.RELATIVE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.RELATIVE_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.GLOBAL_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.GLOBAL_CLOSED_POSITION;


public class Claw {

    private CRServo crServo;
    private DcMotor encoder;

    private String status = "IDLE";

    private double claw_position, delta2Open, delta2Close, power;

    private double CLOSED_POSITION;
    private double OPEN_POSITION;

    private ElapsedTime time;


    private enum STATE {
        STOPPED,
        OPENING,
        CLOSING,
        OPEN,
        CLOSED
    }

    private MODE mode;

    public enum MODE {
        GLOBAL,
        RELATIVE
    }

    private STATE current_state = STATE.STOPPED;

    public Claw(String crServo_id, String encoder_id, MODE mode){

        crServo = Utils.hardwareMap.get(CRServo.class, crServo_id);
        crServo.setDirection(CRServo.Direction.FORWARD);
        crServo.setPower(0);

        encoder = Utils.hardwareMap.get(DcMotor.class, encoder_id);
        this.mode = mode;
        if (mode == MODE.RELATIVE) {
            resetEncoder();
            CLOSED_POSITION = RELATIVE_CLOSED_POSITION;
            OPEN_POSITION = RELATIVE_OPEN_POSITION;
        }
        else {
            CLOSED_POSITION = GLOBAL_CLOSED_POSITION;
            OPEN_POSITION = GLOBAL_OPEN_POSITION;
        }

        time = new ElapsedTime();
    }

    public void setMode(MODE mode){
        this.mode = mode;
        if (mode == MODE.RELATIVE) {
            resetEncoder();
            CLOSED_POSITION = RELATIVE_CLOSED_POSITION;
            OPEN_POSITION = RELATIVE_OPEN_POSITION;
        }
        else {
            CLOSED_POSITION = GLOBAL_CLOSED_POSITION;
            OPEN_POSITION = GLOBAL_OPEN_POSITION;
        }
    }

    public void resetEncoder(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void clawMachine(boolean OPEN_BUTTON, boolean CLOSE_BUTTON, boolean STOP_BUTTON){

        switch (current_state) {

            case STOPPED:
                status = "STOPPED";
                claw_position = encoder.getCurrentPosition();

                crServo.setPower(0);

                if (OPEN_BUTTON) current_state = STATE.OPENING;
                else if (CLOSE_BUTTON) current_state = STATE.CLOSING;

                break;

            case OPEN:
                status = "OPEN";
                claw_position = encoder.getCurrentPosition();

                crServo.setPower(0);
                if (CLOSE_BUTTON) current_state = STATE.CLOSING;
                break;

            case CLOSED:
                status = "CLOSED";
                claw_position = encoder.getCurrentPosition();

                crServo.setPower(0);
                if (OPEN_BUTTON) current_state = STATE.OPENING;
                break;

            case OPENING:
                status = "OPENING";
                claw_position = encoder.getCurrentPosition();

                delta2Open = claw_position - OPEN_POSITION;
                if (Math.abs(delta2Open) > MOE) crServo.setPower(CRSERVO_POWER);
                else {
                    current_state = STATE.OPEN;
                    break;
                }

                if (STOP_BUTTON) current_state = STATE.STOPPED;
                else if (CLOSE_BUTTON) current_state = STATE.CLOSING;
                break;


            case CLOSING:
                status = "CLOSING";
                claw_position = encoder.getCurrentPosition();

                delta2Close = Math.abs(claw_position - CLOSED_POSITION);
                if (delta2Close > MOE) crServo.setPower(-CRSERVO_POWER);
                else {
                    current_state = STATE.CLOSED;
                    break;
                }

                if (STOP_BUTTON) current_state = STATE.STOPPED;
                else if (OPEN_BUTTON) current_state = STATE.OPENING;
                break;
        }
    }

    public DcMotor.RunMode getEncoderMode(){
        return encoder.getMode();
    }

    public double getPower(){
        return crServo.getPower();
    }

    public double getClawPosition(){
        return claw_position;
    }

    public double getDelta2Open(){
        return delta2Open;
    }

    public double getDelta2Close(){
        return delta2Close;
    }

    public String getStatus(){
        return status;
    }

    public void shutdown(){
        crServo.setPower(0);
    }

    public void setPower(double power){ crServo.setPower(power); }
}
