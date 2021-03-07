package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Gripper {

    private Servo arm;
    private CRServo claw;

    private String arm_id, claw_id;

    private final static double SERVO_ARM_HOME = 0.9;
    private final static double SERVO_ARM_MIN_RANGE = 0.0;
    private final static double SERVO_ARM_MAX_RANGE = 0.9;


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
        claw.setPower(0);
    }


    public void openClaw(){
        claw.setPower(1);
    }
    public void closeClaw(){
        claw.setPower(-1);
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
