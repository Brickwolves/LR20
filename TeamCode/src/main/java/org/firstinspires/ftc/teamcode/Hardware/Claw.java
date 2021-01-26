package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Claw {

    private Servo servo;
    private final static double SERVO_CLAW_HOME = 0.3;
    private final static double SERVO_CLAW_MIN_RANGE = 0.3;
    private final static double SERVO_CLAW_MAX_RANGE = 0.65;
    private double SERVO_CLAW_SPEED = 0.1;

    public Claw(String id){

        try {
            servo = Utils.hardwareMap.get(Servo.class, id);
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(SERVO_CLAW_HOME);
        }
        catch (Exception e){
            //throw new Error("Cannot find servo with id: " + id + "\n. Could not initialize claw.");
            Utils.telemetry.addData("Status","Could not set position of non-existant claw.");
            Utils.telemetry.update();
        }

    }

    public void openFull(){
        try {
            servo.setPosition(SERVO_CLAW_MAX_RANGE);
        }
        catch (Exception e){
            //throw new Error("Could not set position of non-existant claw");
            Utils.telemetry.addData("Status","Could not set position of non-existant claw.");
            Utils.telemetry.update();
        }
    }
    public void closeFull(){
        try {
            servo.setPosition(SERVO_CLAW_MIN_RANGE);
        }
        catch (Exception e){
            //throw new Error("Could not set position of non-existant claw");
            Utils.telemetry.addData("Status","Could not set position of non-existant claw.");
            Utils.telemetry.update();
        }

    }


    public void shutdown(){
        try {
            servo.setPosition(SERVO_CLAW_HOME);
        } catch (Exception e) {
            throw new Error("Could not set position of non-existant arm.");
        }
    }

    public void setSpeed(double speed){ SERVO_CLAW_SPEED = speed; }
}
