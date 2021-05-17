package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;
import org.firstinspires.ftc.teamcode.Utilities.Task;

public class Oracle {

    public static ElapsedTime time = new ElapsedTime();
    private static RingBuffer<Double> timeBuffer           = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> angleBuffer          = new RingBuffer<>(5, 0.0);
    private static RingBuffer<Double> xPosBuffer           = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> yPosBuffer           = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> intakeBuffer         = new RingBuffer<>(5,  0.0);

    private static RingBuffer<Double> frBuffer              = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> flBuffer              = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> brBuffer              = new RingBuffer<>(5,  0.0);
    private static RingBuffer<Double> blBuffer              = new RingBuffer<>(5,  0.0);


    public static Task updateTask;
    private static double currentXPosition, currentYPosition, currentAngle, currentIntakePosition;
    public static double frPosition, flPosition, brPosition, blPosition;
    private static double aVelocity, xVelocity, yVelocity, intakeRPM, frRPM, flRPM, brRPM, blRPM;

    public static double getAngularVelocity() { return  aVelocity; }
    public static double getXVelocity() { return  xVelocity; }
    public static double getYVelocity() { return  yVelocity; }
    public static double getXPosition() { return currentXPosition; }
    public static double getYPosition() { return currentYPosition; }
    public static double getAngle() { return currentAngle; }
    public static double getIntakePosition() { return currentIntakePosition; }

    public static void setUpdateTask(Task updateTask) { Oracle.updateTask = updateTask; }
    public static void setAngle(double currentAngle) { Oracle.currentAngle = currentAngle; }
    public static void setXPosition(double currentXPosition) { Oracle.currentXPosition = currentXPosition; }
    public static void setYPosition(double currentYPosition) { Oracle.currentYPosition = currentYPosition; }
    public static void setIntakePosition(double currentIntakePosition) { Oracle.currentIntakePosition = currentIntakePosition; }
    public static void setFRPosition(double frPosition) { Oracle.frPosition = frPosition; }
    public static void setFLPosition(double flPosition) { Oracle.flPosition = flPosition; }
    public static void setBRPosition(double brPosition) { Oracle.brPosition = brPosition; }
    public static void setBLPosition(double blPosition) { Oracle.blPosition = blPosition; }


    public static void setAVelocity(double aVelocity) { Oracle.aVelocity = aVelocity; }
    public static void setXVelocity(double xVelocity) { Oracle.xVelocity = xVelocity; }
    public static void setYVelocity(double yVelocity) { Oracle.yVelocity = yVelocity; }


    public static void update(){

        updateTask.execute();

        // Retrieve Deltas
        double deltaMillis                  = time.milliseconds() - timeBuffer.updateCurWith(time.milliseconds());
        double deltaMinutes                 = deltaMillis / 60000.0;
        double deltaAngle                   = currentAngle - angleBuffer.updateCurWith(currentAngle);
        double deltaX                       = currentXPosition - xPosBuffer.updateCurWith(currentXPosition);
        double deltaY                       = currentYPosition - yPosBuffer.updateCurWith(currentYPosition);
        double deltaIntakeRotations         = currentYPosition - intakeBuffer.updateCurWith(currentIntakePosition);

        double deltaFR                       = (frPosition - frBuffer.updateCurWith(frPosition)) / 537.7;
        double deltaFL                       = (flPosition - flBuffer.updateCurWith(flPosition)) / 537.7;
        double deltaBR                       = (brPosition - brBuffer.updateCurWith(brPosition)) / 537.7;
        double deltaBL                       = (blPosition - blBuffer.updateCurWith(blPosition)) / 537.7;


        // Retrieve Velocities
        aVelocity        = deltaAngle / deltaMinutes;
        frRPM            = deltaFR / deltaMinutes;
        flRPM            = deltaFL / deltaMinutes;
        brRPM            = deltaBR / deltaMinutes;
        blRPM            = deltaBL / deltaMinutes;

        xVelocity = -(frRPM - flRPM - brRPM + blRPM) / 4.0;
        yVelocity = (frRPM + flRPM + brRPM + blRPM) / 4.0;

    }
}
