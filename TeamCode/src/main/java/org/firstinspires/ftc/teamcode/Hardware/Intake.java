package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_LSERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_LSERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_LSERVO_MID;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_LSERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_RSERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_RSERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_RSERVO_MID;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.INTAKE_ARM_RSERVO_MIN;

public class Intake {


    private double currentPower;

    private String intake_motor_id;
    private String left_servo_id;
    private String right_servo_id;

    private DcMotor intake_motor;
    private Servo left_arm_servo;
    private Servo right_arm_servo;
    private STATUS status;
    public enum STATUS {
        UP,
        MID,
        DOWN
    }

    public Intake(String intake_motor_id, String left_servo_id, String right_servo_id){

        this.intake_motor_id = intake_motor_id;
        this.left_servo_id = left_servo_id;
        this.right_servo_id = right_servo_id;

        intake_motor = Utils.hardwareMap.get(DcMotor.class, intake_motor_id);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);

        left_arm_servo = Utils.hardwareMap.get(Servo.class, left_servo_id);
        left_arm_servo.setDirection(Servo.Direction.FORWARD);
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_HOME);

        right_arm_servo = Utils.hardwareMap.get(Servo.class, right_servo_id);
        right_arm_servo.setDirection(Servo.Direction.FORWARD);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_HOME);

    }

    public void setIntakePower(double power){
        intake_motor.setPower(power);
    }
    public void armUp(){
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_MIN);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_MAX);

        if (left_arm_servo.getPosition() == INTAKE_ARM_LSERVO_MIN && right_arm_servo.getPosition() == INTAKE_ARM_RSERVO_MAX){
            status = STATUS.UP;
        }
    }

    public void armMid(){
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_MID);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_MID);
        if (left_arm_servo.getPosition() == INTAKE_ARM_LSERVO_MID && right_arm_servo.getPosition() == INTAKE_ARM_RSERVO_MID){
            status = STATUS.MID;
        }
    }

    public void armDown(){
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_MAX);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_MIN);

        if (left_arm_servo.getPosition() == INTAKE_ARM_LSERVO_MAX && right_arm_servo.getPosition() == INTAKE_ARM_RSERVO_MIN){
            status = STATUS.DOWN;
        }
    }

    public STATUS getStatus() { return status; }
    public double getLeftServoPosition(){
        return left_arm_servo.getPosition();
    }
    public double getRightServoPosition(){
        return right_arm_servo.getPosition();
    }
    public double getIntakePosition(){
        return intake_motor.getCurrentPosition();
    }
    public double getIntakePower(){
        return intake_motor.getPower();
    }
    public void shutdown(){
        armUp();
        setIntakePower(0);
    }

}
