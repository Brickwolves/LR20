package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LEFT_ARM_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LEFT_ARM_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LEFT_ARM_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.RIGHT_ARM_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.RIGHT_ARM_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.RIGHT_ARM_SERVO_MIN;

public class Intake {


    private double currentPower;

    private String intake_motor_id;
    private String left_servo_id;
    private String right_servo_id;

    private DcMotor intake_motor;
    private Servo left_arm_servo;
    private Servo right_arm_servo;

    public Intake(String intake_motor_id, String left_servo_id, String right_servo_id){


        this.intake_motor_id = intake_motor_id;
        this.left_servo_id = left_servo_id;
        this.right_servo_id = right_servo_id;

        intake_motor = Utils.hardwareMap.get(DcMotor.class, intake_motor_id);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);

        left_arm_servo = Utils.hardwareMap.get(Servo.class, left_servo_id);
        left_arm_servo.setDirection(Servo.Direction.FORWARD);
        left_arm_servo.setPosition(LEFT_ARM_SERVO_HOME);

        right_arm_servo = Utils.hardwareMap.get(Servo.class, right_servo_id);
        right_arm_servo.setDirection(Servo.Direction.FORWARD);
        right_arm_servo.setPosition(RIGHT_ARM_SERVO_HOME);


    }

    public void setIntakePower(double power){
        intake_motor.setPower(power);
    }
    public void armUp(){
        left_arm_servo.setPosition(LEFT_ARM_SERVO_MIN);
        right_arm_servo.setPosition(RIGHT_ARM_SERVO_MAX);
    }
    public void armDown(){
        left_arm_servo.setPosition(LEFT_ARM_SERVO_MAX);
        right_arm_servo.setPosition(RIGHT_ARM_SERVO_MIN);
    }


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

}
