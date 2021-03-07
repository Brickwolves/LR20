package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Arm {

    private Servo servo;

    private final static double SERVO_ARM_HOME = 0.9;
    private final static double SERVO_ARM_MIN_RANGE = 0.0;
    private final static double SERVO_ARM_MAX_RANGE = 0.9;

    public Arm(String id) {
        servo = Utils.hardwareMap.get(Servo.class, id);
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(SERVO_ARM_HOME);
    }

    public void setArmPosition(double position){
        double servo_arm_position = Range.clip(position, SERVO_ARM_MIN_RANGE, SERVO_ARM_MAX_RANGE);
        servo.setPosition(servo_arm_position);
    }

    public void up() {
        servo.setPosition(SERVO_ARM_MAX_RANGE);
    }

    public void down() {
        servo.setPosition(SERVO_ARM_MIN_RANGE);
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
