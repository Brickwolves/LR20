package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.CRSERVO_POWER;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.MOE;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_CRServoDiag.OPEN_POSITION;

public class Claw2 {

    private DcMotor encoder;
    private CRServo crServo;
    private String status = "IDLE";

    private double power;
    private double delta2Open;
    private double delta2Close;
    private double claw_position;


    private ElapsedTime time;


    public Claw2(String crServo_id, String encoder_id){

        crServo = Utils.hardwareMap.get(CRServo.class, crServo_id);
        crServo.setDirection(CRServo.Direction.FORWARD);
        crServo.setPower(0);

        encoder = Utils.hardwareMap.get(DcMotor.class, encoder_id);

        time = new ElapsedTime();
    }

    public void open() {
        power = 0.0;
        claw_position = encoder.getCurrentPosition();
        delta2Open = Math.abs(claw_position - OPEN_POSITION);
        delta2Close = Math.abs(claw_position - CLOSE_POSITION);
        if (delta2Open > MOE){
            power = CRSERVO_POWER;
            status = "OPENING";
        }
        else {
            status = "CANNOT OPEN ANYMORE";
            power = 0.0;
        }

        crServo.setPower(power);
    }

    public void close() {
        power = 0.0;
        claw_position = encoder.getCurrentPosition();
        delta2Open = Math.abs(claw_position - OPEN_POSITION);
        delta2Close = Math.abs(claw_position - CLOSE_POSITION);
        if (delta2Close > MOE){
            power = -CRSERVO_POWER;
            status = "CLOSING";
        }
        else {
            status = "CANNOT CLOSE ANYMORE";
            power = 0.0;
        }

        crServo.setPower(power);
    }

    public void checkLimit(){
        claw_position = encoder.getCurrentPosition();

        delta2Open = Math.abs(claw_position - OPEN_POSITION);
        delta2Close = Math.abs(claw_position - CLOSE_POSITION);

        if (delta2Open > MOE) {
            power = 0;
            status = "CANNOT OPEN ANYMORE";
        }
        if (delta2Close > MOE){
            power = 0;
            status = "CANNOT CLOSE ANYMORE";
        }
        crServo.setPower(power);
    }

    public void stop() {
        crServo.setPower(0);
    }

    public String getStatus(){
        return status;
    }

    public double getPower(){
        return power;
    }

    public double getDelta2Open(){
        return delta2Open;
    }

    public double getDelta2Close(){
        return delta2Close;
    }

    public void shutdown(){
        open();
    }

    public void setPower(double power){ crServo.setPower(power); }
}
