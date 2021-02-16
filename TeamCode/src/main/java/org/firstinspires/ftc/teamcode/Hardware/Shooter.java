package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.RingBuffer;

public class Shooter {

    private DcMotor shooterOne;
    private DcMotor shooterTwo;
    private Servo feeder;
    private Servo feederLock;
    public PID shooterPID = new PID(.0002, 0.000008, 0.000009, 0, false);

    private static final double TICKS_PER_ROTATION = 28;
    private static final double RING_FEED = 0.9; //feeder max
    private static final double RESET = 0.65; //feeder min
    private static final double FEEDER_LOCK = 0.5; //lock max
    private static final double FEEDER_UNLOCK = 0.3; //lock min

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

    RingBuffer<Long> timeRing = new RingBuffer<Long>(20, (long)0);
    RingBuffer<Long> positionRing = new RingBuffer<Long>(20, (long)0);
    private FeederState currentFeederState = FeederState.STATE_IDLE;


    public ElapsedTime feederTime = new ElapsedTime();

    public Shooter(DcMotor shooterOne, DcMotor shooterTwo, Servo feeder, Servo feederLock) {

        shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.feeder = feeder;
        this.feederLock = feederLock;
    }

    public void feedRing(){
        feeder.setPosition(RING_FEED);
    }

    public void resetFeeder(){
        feeder.setPosition(RESET);
    }

    public void lockFeeder(){
        feederLock.setPosition(FEEDER_LOCK);
    }

    public boolean isFeederLocked() { return feeder.getPosition() == FEEDER_LOCK; }

    public void unlockFeeder(){
        feederLock.setPosition(FEEDER_UNLOCK);
    }

    public double feederCount(){ return feedCount; }

    public void feederState(boolean trigger){
        switch (currentFeederState) {

            case STATE_IDLE:
                if(trigger && getPower() >= .1){ newState(FeederState.STATE_FEED); }
                if(feederTime.seconds() > LOCK_TIME){ lockFeeder(); isFeederLocked = false; }
                else{ unlockFeeder(); isFeederLocked = false; }
                resetFeeder();
                break;

            case STATE_FEED:
                if (isFeederLocked) {
                    if (feederTime.seconds() > FEED_TIME + UNLOCK_TIME) { newState(FeederState.STATE_RESET); }
                    if (feederTime.seconds() > UNLOCK_TIME) { feedRing(); }
                }else{
                    if (feederTime.seconds() > FEED_TIME) { newState(FeederState.STATE_RESET); }
                    feedRing();
                }
                unlockFeeder();
                break;

            case STATE_RESET:
                if (feederTime.seconds() > RESET_TIME) { newState(FeederState.STATE_IDLE); feedCount++; break; }
                resetFeeder();
                unlockFeeder();
                break;
        }
    }


    public void setPower(double power){
        shooterOne.setPower(power);
        shooterTwo.setPower(power);
    }

    public double getPower(){
        return (shooterOne.getPower() + shooterTwo.getPower()) /2;
    }

    public void resetEncoders(){
        shooterOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public long getPosition(){
        return (shooterOne.getCurrentPosition() + shooterTwo.getCurrentPosition()) /  2;
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
        shooterPower = shooterPID.update( targetRPM - updateRPM());

        shooterPower = Range.clip(shooterPower,0.0, 1.0);
        setPower(shooterPower);
    }


    public void topGoal(){ setRPM(TOP_GOAL); }

    public void powerShot(){ setRPM(POWER_SHOT); }

    public void shooterOff(){ setPower(0.0); }



    private void newState(FeederState newState) { currentFeederState = newState; feederTime.reset(); }


    private enum FeederState {
        STATE_IDLE,
        STATE_RESET,
        STATE_FEED
    }


}
