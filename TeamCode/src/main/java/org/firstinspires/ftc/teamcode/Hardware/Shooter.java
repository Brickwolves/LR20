package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Shooter;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.LOCK_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.LOCK_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.LOCK_SERVO_MIN;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.SHOOT_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.SHOOT_SERVO_MAX;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Servos.SHOOT_SERVO_MIN;

public class Shooter {

    private DcMotor shooter1;
    private DcMotor shooter2;
    private Servo shoot_servo;
    private Servo lock_servo;
    //public PID shooterPID = new PID(.0002, 0.000008, 0.000009, 0, false);
    public PID shooterPID = new PID(Dash_Shooter.p, Dash_Shooter.i, Dash_Shooter.d, 0, false);

    private static final double TICKS_PER_ROTATION = 28;

    private static final double SHOOT_POSITION =    SHOOT_SERVO_MIN;
    private static final double RESET_POSITION =    SHOOT_SERVO_MAX;

    private static final double LOCK_POSITION =     LOCK_SERVO_MAX;
    private static final double UNLOCK_POSITION =   LOCK_SERVO_MIN;

    private static final int TOP_GOAL = 3250;
    private static final int POWER_SHOT = 3000;

    private static final double FEED_TIME = .23;
    private static final double RESET_TIME = .2;
    private static final double LOCK_TIME = .8;
    private static final double UNLOCK_TIME = .1;

    private double shooterPower = 0.0;
    private boolean isFeederLocked;
    private double shooterRPM;
    private int feedCount = 0;

    RingBuffer<Long> timeRing = new RingBuffer<Long>(5, (long)0);
    RingBuffer<Long> positionRing = new RingBuffer<Long>(5, (long)0);
    private FeederState currentFeederState = FeederState.STATE_IDLE;


    public ElapsedTime feederTime = new ElapsedTime();

    public Shooter(String shooter1_id, String shooter2_id, String shooter_id, String lock_id) {


        shooter1 = Utils.hardwareMap.get(DcMotor.class, shooter1_id);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter2 = Utils.hardwareMap.get(DcMotor.class, shooter2_id);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shoot_servo = Utils.hardwareMap.get(Servo.class, shooter_id);
        shoot_servo.setDirection(Servo.Direction.FORWARD);
        shoot_servo.setPosition(SHOOT_SERVO_HOME);

        lock_servo = Utils.hardwareMap.get(Servo.class, lock_id);
        lock_servo.setDirection(Servo.Direction.FORWARD);
        lock_servo.setPosition(LOCK_SERVO_HOME);
    }

    public void shootRing(){
        shoot_servo.setPosition(SHOOT_POSITION);
    }

    public void resetShooter(){
        shoot_servo.setPosition(RESET_POSITION);
    }

    public void lockFeeder(){
        lock_servo.setPosition(LOCK_POSITION);
        isFeederLocked = true;
    }

    public boolean isFeederLocked() { return shoot_servo.getPosition() == LOCK_POSITION; }

    public void unlockFeeder(){
        lock_servo.setPosition(UNLOCK_POSITION);
        isFeederLocked = false;
    }

    public double getFeederCount(){ return feedCount; }
    public void setFeederCount(int feedCount){ this.feedCount = feedCount; }

    public void feederState(boolean trigger){

        switch (currentFeederState) {

            case STATE_IDLE:

                if (trigger && getPower() >= .1)        setState(FeederState.STATE_FEED);

                if (feederTime.seconds() > LOCK_TIME)   lockFeeder();
                else                                    unlockFeeder();

                resetShooter();
                break;


            case STATE_FEED:

                if (isFeederLocked) {                                                                                   // If feeder is locked then check if...
                    if (feederTime.seconds() > UNLOCK_TIME + FEED_TIME) setState(FeederState.STATE_RESET);              // If we've had enough time to unlock and shoot a ring? Reset to shoot another
                    if (feederTime.seconds() > UNLOCK_TIME) shootRing();                                                // If we've had time to unlock, we can shoot
                }
                else {                                                                                                  // If we are unlocked and good to go
                    if (feederTime.seconds() > FEED_TIME) setState(FeederState.STATE_RESET);                            // If we've shot we should reset
                    shootRing();                                                                                        // Otherwise Shoot!!!
                }
                unlockFeeder();                                                                                         // End the state by unlocking
                break;

            case STATE_RESET:
                if (feederTime.seconds() > RESET_TIME) {
                    setState(FeederState.STATE_IDLE);
                    feedCount++;
                    break;
                }
                resetShooter();
                unlockFeeder();
                break;
        }
    }


    public void setPower(double power){
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public double getPower(){
        return (shooter1.getPower() + shooter2.getPower()) / 2;
    }

    public void resetEncoders(){
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public long getPosition(){
        return (shooter1.getCurrentPosition() + shooter2.getCurrentPosition()) /  2;
    }

    public double updateRPM(){

        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;

        long currentPosition = getPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;

        shooterRPM = Math.abs(deltaRotations / deltaMinutes);

        return shooterRPM;
    }

    public double getRPM(){
        return shooterRPM;
    }

    public void setRPM(int targetRPM){
        shooterPower = shooterPID.update( targetRPM + 1500 - updateRPM());

        shooterPower = Range.clip(shooterPower,0.0, 1.0);
        setPower(shooterPower);
    }


    public void topGoal(){ setRPM(TOP_GOAL); }

    public void powerShot(){ setRPM(POWER_SHOT); }

    public void shooterOff(){ setPower(0.0); }



    private void setState(FeederState newState) { currentFeederState = newState; feederTime.reset(); }


    private enum FeederState {
        STATE_IDLE,
        STATE_RESET,
        STATE_FEED
    }


    public double getShooterServoPosition(){
        return shoot_servo.getPosition();
    }
    public double getLockServoPosition(){
        return lock_servo.getPosition();
    }

}
