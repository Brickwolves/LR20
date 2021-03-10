package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Utils;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.CLAW_LSERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.CLAW_LSERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.CLAW_LSERVO_MAX;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.CLAW_RSERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.CLAW_RSERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.CLAW_RSERVO_MAX;


// LEFT PORT 1,  MAX CLOSED, MIN OPEN
// RIGHT PORT 2, MIN CLOSED, MAX OPEN


public class Claw {

    private ElapsedTime time;
    private String status = "IDLE";
    private Servo left_servo, right_servo;

    public Claw(String left_servo_id, String right_servo_id){

        left_servo = Utils.hardwareMap.get(Servo.class, left_servo_id);
        left_servo.setDirection(Servo.Direction.FORWARD);

        right_servo = Utils.hardwareMap.get(Servo.class, right_servo_id);
        right_servo.setDirection(Servo.Direction.FORWARD);

        resetHome();

        time = new ElapsedTime();
    }

    public double getLeftServoPosition(){ return left_servo.getPosition(); }
    public double getRightServoPosition(){ return right_servo.getPosition(); }
    public void setLeftServoPosition(double position) { left_servo.setPosition(position);}
    public void setRightServoPosition(double position) { right_servo.setPosition(position);}

    public void open() {
        status = "OPENING";
        left_servo.setPosition(CLAW_LSERVO_MIN);
        right_servo.setPosition(CLAW_RSERVO_MAX);
    }

    public void close() {
        status = "CLOSING";
        left_servo.setPosition(CLAW_LSERVO_MAX);
        right_servo.setPosition(CLAW_RSERVO_MIN);
    }

    public void stop() {
        status = "STOPPING";
        left_servo.setPosition(left_servo.getPosition());
        right_servo.setPosition(right_servo.getPosition());
    }

    public void resetHome(){
        left_servo.setPosition(CLAW_LSERVO_HOME);
        right_servo.setPosition(CLAW_RSERVO_HOME);
    }

    public String getStatus(){ return status; }
}
