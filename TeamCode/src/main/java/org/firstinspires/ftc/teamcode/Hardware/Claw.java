package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Claw {

    private Servo servo;
    private final static double SERVO_CLAW_HOME = 0.23;
    private final static double SERVO_CLAW_MIN_RANGE = 0.23;
    private final static double SERVO_CLAW_MAX_RANGE = 0.38;
    private double SERVO_CLAW_SPEED = 0.1;

    public Claw(String id){
        servo = Utils.hardwareMap.get(Servo.class, id);
    }

    public void openFull(){
        servo.setPosition(SERVO_CLAW_MAX_RANGE);
    }
    public void closeFull(){
        servo.setPosition(SERVO_CLAW_MIN_RANGE);
    }
    public void setSpeed(double speed){ SERVO_CLAW_SPEED = speed; }
}
