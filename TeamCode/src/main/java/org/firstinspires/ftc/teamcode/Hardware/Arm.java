package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Arm {

    private Servo servo;

    private final static double SERVO_ARM_HOME = 0.9;
    private final static double OUT_POS = 0.0;
    private final static double UP_POS = 0.68;
    private final static double IN_POS = 0.9;

    public Arm(String id) {
        servo = Utils.hardwareMap.get(Servo.class, id);
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(SERVO_ARM_HOME);
    }

    public void setArmPosition(double position){
        double servo_arm_position = Range.clip(position, OUT_POS, IN_POS);
        servo.setPosition(servo_arm_position);
    }


    public void in() {
        servo.setPosition(IN_POS);
    }

    public void up(){
        servo.setPosition(UP_POS);
    }

    public void out() {
        servo.setPosition(OUT_POS);
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
}
