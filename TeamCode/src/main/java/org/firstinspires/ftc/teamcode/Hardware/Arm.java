package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static org.firstinspires.ftc.teamcode.Hardware.Arm.STATUS.*;

public class Arm {

    private Servo servo;

    private final static double SERVO_ARM_HOME = 0.9;
    private final static double OUT_POS = 0.0;
    private final static double UP2_POS = 0.59;
    private final static double UP_POS = 0.7;
    private final static double IN_POS = 0.9;
    private STATUS status = IN;

    public enum STATUS {
        OUT,
        UP,
        UP2,
        IN
    }

    public Arm(String id) {
        servo = OpModeUtils.hardwareMap.get(Servo.class, id);
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(IN_POS);
    }

    public void setArmPosition(double position){
        double servo_arm_position = Range.clip(position, OUT_POS, IN_POS);
        servo.setPosition(servo_arm_position);
    }


    public void in() {
        servo.setPosition(IN_POS);
        status = IN;
    }

    public void up(){
        servo.setPosition(UP_POS);
        status = UP;
    }
    public void up2() {
        servo.setPosition(UP2_POS);
        status = UP2;
    }

    public void out() {
        servo.setPosition(OUT_POS);
        status = OUT;
    }

    public void shutdown(){
        servo.setPosition(SERVO_ARM_HOME);
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

    public double getPosition(){
        return servo.getPosition();
    }

    public STATUS getStatus() { return status; }
}
