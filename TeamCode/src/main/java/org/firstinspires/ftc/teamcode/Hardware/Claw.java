package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Claw {

    private CRServo servo;

    public Claw(String id){

        servo = Utils.hardwareMap.get(CRServo.class, id);
        servo.setDirection(CRServo.Direction.FORWARD);
        servo.setPower(0);
    }

    public void openFull(){
        servo.setPower(1);
    }
    public void closeFull(){
        servo.setPower(-1);
    }

    public void shutdown(){
        servo.setPower(0);
    }

    public void setPower(double power){ servo.setPower(power); }
}
