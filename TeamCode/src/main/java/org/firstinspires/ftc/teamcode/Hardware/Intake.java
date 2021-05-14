package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_Intake;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.INTAKE_ARM_LSERVO_MAX;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.INTAKE_ARM_LSERVO_MID;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.INTAKE_ARM_LSERVO_MIN;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.INTAKE_ARM_RSERVO_MAX;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.INTAKE_ARM_RSERVO_MID;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.INTAKE_ARM_RSERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

public class Intake {


    private double intakeRPM;
    private double intakePower;

    private String intake_motor_id;
    private String left_servo_id;
    private String right_servo_id;
    private PID intakePID = new PID(Dash_Intake.p, Dash_Intake.i, Dash_Intake.d, 0.85, 0, false);
    private RingBuffer<Double> positionBuffer   = new RingBuffer<>(5,  0.0);
    private RingBuffer<Double> timeBuffer       = new RingBuffer<>(5,  0.0);
    private ElapsedTime time = new ElapsedTime();

    private static final double TICKS_PER_ROTATION = 103.8;

    // Uses a 1620RPM DcMotor w/ 28ticks/rotation
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

        intake_motor = hardwareMap.get(DcMotor.class, intake_motor_id);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);

        left_arm_servo = hardwareMap.get(Servo.class, left_servo_id);
        left_arm_servo.setDirection(Servo.Direction.FORWARD);
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_MID);

        right_arm_servo = hardwareMap.get(Servo.class, right_servo_id);
        right_arm_servo.setDirection(Servo.Direction.FORWARD);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_MID);

        rollerUp();

    }

    public void setIntakePower(double power){
        intake_motor.setPower(power);
    }
    public void rollerUp(){
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_MIN);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_MAX);

        if (left_arm_servo.getPosition() == INTAKE_ARM_LSERVO_MIN && right_arm_servo.getPosition() == INTAKE_ARM_RSERVO_MAX){
            status = STATUS.UP;
        }
    }

    public void rollerMid(){
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_MID);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_MID);

        if (left_arm_servo.getPosition() == INTAKE_ARM_LSERVO_MID && right_arm_servo.getPosition() == INTAKE_ARM_RSERVO_MID){
            status = STATUS.MID;
        }
    }

    public void rollerDown(){
        left_arm_servo.setPosition(INTAKE_ARM_LSERVO_MAX);
        right_arm_servo.setPosition(INTAKE_ARM_RSERVO_MIN);
        if (left_arm_servo.getPosition() == INTAKE_ARM_LSERVO_MAX && right_arm_servo.getPosition() == INTAKE_ARM_RSERVO_MIN){
            status = STATUS.DOWN;
        }
    }

    public double updateRPM() {

        double currentPosition = intake_motor.getCurrentPosition();
        double currentTime = time.milliseconds();

        double deltaMillis = currentTime - timeBuffer.updateCurWith(currentTime);
        double deltaMinutes = deltaMillis / 60000.0;

        double deltaTickRotations = currentPosition - positionBuffer.updateCurWith(currentPosition);
        double deltaRotations = deltaTickRotations / TICKS_PER_ROTATION;

        intakeRPM = abs(deltaRotations / deltaMinutes);

        multTelemetry.addData("intakeRPM", intakeRPM);
        multTelemetry.addData("deltaRotations", deltaRotations);
        multTelemetry.addData("deltaMillis", deltaMillis);

        return intakeRPM;
    }

    public double getRPM(){
        return intakeRPM;
    }

    public void setRPM(double targetRPM){
        intakePID.setF(targetRPM / 1180);
        intakePower = intakePID.update( targetRPM + updateRPM());

        intakePower = clip(intakePower, 0.0, 1.0);
        intake_motor.setPower(intakePower);
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
        rollerUp();
        setIntakePower(0);
    }

}
