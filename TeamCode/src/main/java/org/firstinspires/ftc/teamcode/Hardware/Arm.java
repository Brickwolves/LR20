package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Arm {

    private Servo servo;

    private final static double SERVO_ARM_HOME = 0.0;
    private final static double SERVO_ARM_MIN_RANGE = 0.0;
    private final static double SERVO_ARM_MAX_RANGE = 0.35;
    private double SERVO_ARM_SPEED = 0.75;
    private double servo_arm_position = SERVO_ARM_HOME;

    public Arm(String id) {

        try {
            servo = Utils.hardwareMap.get(Servo.class, id);
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(SERVO_ARM_HOME);
        } catch (Exception e) {
            throw new Error("Cannot find servo with id: " + id + "\n. Could not initialize arm.");
        }

    }

    public void up() {
        try {
            servo_arm_position += SERVO_ARM_SPEED;
            servo_arm_position = Range.clip(servo_arm_position, SERVO_ARM_MIN_RANGE, SERVO_ARM_MAX_RANGE);
            servo.setPosition(servo_arm_position);
        } catch (Exception e) {
            throw new Error("Could not set position of non-existant arm.");
        }
    }

    public void down() {
        try {
            servo_arm_position -= SERVO_ARM_SPEED;
            servo_arm_position = Range.clip(servo_arm_position, SERVO_ARM_MIN_RANGE, SERVO_ARM_MAX_RANGE);
            servo.setPosition(servo_arm_position);
        } catch (Exception e) {
            throw new Error("Could not set position of non-existant arm.");
        }

    }

    public void shutdown(){
        try {
            servo.setPosition(SERVO_ARM_HOME);
        } catch (Exception e) {
            throw new Error("Could not set position of non-existant arm.");
        }
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

    public double getPosition(){
        return servo.getPosition();
    }

    public void setSpeed(double speed) {
        SERVO_ARM_SPEED = speed;
    }
}
