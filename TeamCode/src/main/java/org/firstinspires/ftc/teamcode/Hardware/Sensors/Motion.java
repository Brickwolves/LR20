package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;

import java.util.HashMap;
import java.util.Map;

import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.teamcode.Hardware.Sensors.Motion.Motor.BL;
import static org.firstinspires.ftc.teamcode.Hardware.Sensors.Motion.Motor.BR;
import static org.firstinspires.ftc.teamcode.Hardware.Sensors.Motion.Motor.FL;
import static org.firstinspires.ftc.teamcode.Hardware.Sensors.Motion.Motor.FR;
import static org.firstinspires.ftc.teamcode.Hardware.Sensors.Motion.MotorData.RPM;

public class Motion {

    private static IMU imu;

    private static ElapsedTime time = new ElapsedTime();
    private static RingBuffer<Double> timeBuffer           = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> angleBuffer          = new RingBuffer<>(5, 0.0);
    private static RingBuffer<Double> frBuffer             = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> flBuffer             = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> brBuffer             = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> blBuffer             = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> xPosBuffer           = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> yPosBuffer           = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> intakeBuffer         = new RingBuffer<>(5,  0.0);

    private static Map<Motor, Double> motorRPMs = new HashMap<Motor, Double>();


    public enum Motor {
        FR, FL, BR, BL
    }
    public enum MotorData {
        POSITION, RPM
    }

    public Motion(){
        imu = new IMU("imu");

        // Motor RPMS
        motorRPMs.put(FR, 0.0);
        motorRPMs.put(FL, 0.0);
        motorRPMs.put(BR, 0.0);
        motorRPMs.put(BL, 0.0);
    }

    public double getMotorData(Motor motor, MotorData motorData){
        double data = 0;

        switch (motor){
            case FR:
                data = (motorData == RPM) ? 0 : frBuffer.getValue();
                break;

            case FL:
                data = (motorData == RPM) ? 0 : flBuffer.getValue();
                break;

            case BR:
                data = (motorData == RPM) ? 0 : brBuffer.getValue();
                break;

            case BL:
                data = (motorData == RPM) ? 0 : blBuffer.getValue();
                break;
        }
        return data;
    }

    /*
    public double robotVelocityComponent(double angle){
        double relYVelocity = getYComp(motorRPMs.get("FR"), motorRPMs.get("FL"), motorRPMs.get("BR"), motorRPMs.get("BL"));
        double relXVelocity = (motorRPMs.get("FR") - motorRPMs.get("FL") - motorRPMs.get("BR") + motorRPMs.get("BL")) / 4;
        //double strafe = getXComp(motorRPMs.get("FR"), motorRPMs.get("FL"), motorRPMs.get("BR"), motorRPMs.get("BL"));


        double velocityAngle;

        double speed = hypot(relXVelocity, relYVelocity);
        if (speed == 0) velocityAngle = 0;
        else velocityAngle = - toDegrees(atan2(relYVelocity, relXVelocity)) + 180;

        angle -= velocityAngle;

        return toDegrees(cos(angle)) * speed;
    }
     */

    /*
    public static void update(){

        // Retrieve Deltas
        double deltaMillis                  = timeBuffer.getValue(time.milliseconds());
        double deltaMinutes                 = deltaMillis / 60000.0;
        double deltaAngle                   = angleBuffer.getValue(imu.getAngle());

        double deltaIntakeRotations         = intakeBuffer.getValue((double) intake.getIntakePosition()) / 537.7;

        double deltaFRRotations             = frBuffer.getValue((double) fr.getCurrentPosition()) / 537.7;
        double deltaFLRotations             = flBuffer.getValue((double) fl.getCurrentPosition()) / 537.7;
        double deltaBRRotations             = brBuffer.getValue((double) br.getCurrentPosition()) / 537.7;
        double deltaBLRotations             = blBuffer.getValue((double) bl.getCurrentPosition()) / 537.7;

        double deltaX                       = xPosBuffer.getValue(getXComp());
        double deltaY                       = yPosBuffer.getValue(getYComp());

        // Retrieve RPMs
        double frRPM = deltaFRRotations / deltaMinutes;
        double flRPM = deltaFLRotations / deltaMinutes;
        double brRPM = deltaBRRotations / deltaMinutes;
        double blRPM = deltaBLRotations / deltaMinutes;

        // Retrieve Velocities
        double angularVelocity  = deltaAngle / deltaMinutes;
        double xVelocity        = deltaX / deltaMinutes;
        double yVelocity        = deltaY / deltaMinutes;


        // Update HashMaps
        motorRPMs.put("FR", frRPM);
        motorRPMs.put("FL", flRPM);
        motorRPMs.put("BR", brRPM);
        motorRPMs.put("BL", blRPM);
    }

     */
}
