package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.L_WING_MID;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.L_WING_OUT;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.L_WING_UP;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.R_WING_MID;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.R_WING_OUT;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Servos.R_WING_UP;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;


// LEFT,  MAX OUT, MIN UP
// RIGHT, MIN UP, MAX OUT
// LEFT & RIGHT relative to someone looking at back of robot


public class Wings {

    private ElapsedTime time;
    private String status = "IDLE";
    private Servo left_servo, right_servo;

    public Wings(String left_servo_id, String right_servo_id){

        left_servo = hardwareMap.get(Servo.class, left_servo_id);
        left_servo.setDirection(Servo.Direction.FORWARD);

        right_servo = hardwareMap.get(Servo.class, right_servo_id);
        right_servo.setDirection(Servo.Direction.FORWARD);

        resetHome();

        time = new ElapsedTime();
    }

    public double getLeftServoPosition(){ return left_servo.getPosition(); }
    public double getRightServoPosition(){ return right_servo.getPosition(); }
    public void setLeftServoPosition(double position) { left_servo.setPosition(position);}
    public void setRightServoPosition(double position) { right_servo.setPosition(position);}

    public void out() {
        status = "OUT";
        left_servo.setPosition(L_WING_OUT);
        right_servo.setPosition(R_WING_OUT);
    }

    public void mid() {
        status = "MID";
        left_servo.setPosition(L_WING_MID);
        right_servo.setPosition(R_WING_MID);
    }

    public void up() {
        status = "UP";
        left_servo.setPosition(L_WING_UP);
        right_servo.setPosition(R_WING_UP);
    }

    public void resetHome(){
        status = "HOME";
        up();
    }

    public String getStatus(){ return status; }
}
