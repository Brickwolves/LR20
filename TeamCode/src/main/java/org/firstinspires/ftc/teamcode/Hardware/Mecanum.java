package org.firstinspires.ftc.teamcode.Hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.Utils;


import static android.os.SystemClock.sleep;
import static java.lang.Math.floorMod;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.*;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.convertInches2Ticks;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.map;


public class Mecanum implements Robot {

   private DcMotor fr, fl, br, bl;
   //public TouchSensor touchSensor;
   //public ColorSensorImpl colorSensor;
   //public ColorSensor colorSensorBase;
   public IMU imu;
   public Arm arm;
   public Claw claw;
   public Intake intake;
   public Shooter shooter;

   private double turn;

   private double initAngle;
   private double currentPosition;

   public PID rotationPID = new PID(Dash_Movement.p, Dash_Movement.i, Dash_Movement.d, 100, true);

   public Mecanum(){
      initRobot();
   }

   public void initRobot() {
      Utils.telemetry.addData("Status", "Initialized");

      // Init Motors
      fr = Utils.hardwareMap.get(DcMotor.class, "front_right_motor");
      fl = Utils.hardwareMap.get(DcMotor.class, "front_left_motor");
      br = Utils.hardwareMap.get(DcMotor.class, "back_right_motor");
      bl = Utils.hardwareMap.get(DcMotor.class, "back_left_motor");
      resetMotors();

      // Sensors
      //touchSensor = Utils.hardwareMap.get(TouchSensor.class, "touch_sensor");
      //colorSensorBase = Utils.hardwareMap.get(ColorSensor.class, "color_sensor");
      //colorSensor = new ColorSensorImpl(colorSensorBase);
      imu = new IMU("imu");
      //gripper = new Gripper("claw", "arm");
      claw = new Claw("left_claw", "right_claw");
      arm = new Arm("arm");
      intake = new Intake("intake", "servo_0", "servo_1");
      shooter = new Shooter("spinny_1", "spinny_2", "servo_4", "servo_5");

      initAngle = imu.getAngle();
   }

   /**
    * (Re)Init Motors
    */
   public void resetMotors(){
      fr.setDirection(DcMotorSimple.Direction.FORWARD);
      fl.setDirection(DcMotorSimple.Direction.REVERSE);
      br.setDirection(DcMotorSimple.Direction.FORWARD);
      bl.setDirection(DcMotorSimple.Direction.REVERSE);

      fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }



   /**
    * @return average encoder position
    */
   public double getPosition(){
      return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4.0;
   }

   /**
    * @param power
    */
   @Override
   public void setAllPower(double power){
      fl.setPower(power);
      fr.setPower(power);
      bl.setPower(power);
      br.setPower(power);
   }

   /**
    * @param drive
    * @param strafe
    * @param turn
    */
   public void setDrivePower(double drive, double strafe, double turn, double velocity) {
      fr.setPower((drive - strafe - turn) * velocity);
      fl.setPower((drive + strafe + turn) * velocity);
      br.setPower((drive + strafe - turn) * velocity);
      bl.setPower((drive - strafe + turn) * velocity);
   }


   @RequiresApi(api = Build.VERSION_CODES.N)
   public double closestRelativeAngle(double targetAngle){
      double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + imu.getAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
      return StrictMath.abs(simpleTargetDelta) >= StrictMath.abs(alternateTargetDelta) ? 0 - simpleTargetDelta : 0 - alternateTargetDelta;
   }

   /**
    * @param angle
    * @param inches
    */
   @RequiresApi(api = Build.VERSION_CODES.N)
   public void strafe(double angle, double inches, double directionFacingAngle, double acceleration, SyncTask task){

      ElapsedTime time = new ElapsedTime();
      System.out.println(angle + " " + inches);
      double ticks = convertInches2Ticks(inches);


      angle = closestRelativeAngle(angle);
      directionFacingAngle = findClosestAngle(directionFacingAngle, imu.getAngle());


      resetMotors();                                              // Reset Motor Encoder

      double radians = (angle + 90) * (Math.PI / 180);             // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
      double yFactor = Math.sin(radians);                         // Unit Circle Y
      double xFactor = Math.cos(radians);                         // Unit Circle X
      double yTicks = Math.abs(yFactor * ticks);
      double xTicks = Math.abs(xFactor * ticks);
      double distance = Math.max(yTicks, xTicks);


      // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
      double normalizeToPower = 1 / Math.max(Math.abs(xFactor), Math.abs(yFactor));
      double drive = normalizeToPower * yFactor;                 // Fill out power to a max of 1
      double strafe = normalizeToPower * xFactor;                // Fill out power to a max of 1
      double turn = 0;


      currentPosition = getPosition();
      while (getPosition() < distance && Utils.isActive()){

         // Execute task synchronously
         if (task != null) task.execute();

         // Power ramping
         double power = Utils.powerRamp(currentPosition, distance, acceleration);

         // PID Controller
         double error = directionFacingAngle - imu.getAngle();
         turn = rotationPID.update(error) * -1;
         //turn = error * DashConstants.learning_rate * -1;
         setDrivePower(drive * power, strafe * power, turn, 1);

         // Log and get new position
         currentPosition = getPosition();


         Utils.telemetry.addData("Error", error);
         Utils.telemetry.update();
      }
      setAllPower(0);
   }

   @RequiresApi(api = Build.VERSION_CODES.N)
   public static double findClosestAngle(double targetAngle, double currentAngle){
      double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
      return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
   }

   @RequiresApi(api = Build.VERSION_CODES.N)
   public void turn(double target_angle, double MOE) {

      double startTime = System.currentTimeMillis();
      double turn_direction, pid_return, power, powerRampPosition;


      //            Calc Power Ramping and PID Values           //
      double current_angle = imu.getAngle();
      double startAngle = current_angle;
      double actual_target_angle = findClosestAngle(target_angle, current_angle);
      double startDeltaAngle = Math.abs(actual_target_angle - current_angle);
      double error = actual_target_angle - current_angle;



      while ((Math.abs(error) > MOE) && Utils.isActive()) {

         //              PID                     //
         error = actual_target_angle - current_angle;
         pid_return = rotationPID.update(error) * -1;
         turn_direction = (pid_return > 0) ? 1 : -1;


         //              Power Ramping            //
         powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
         power = Utils.powerRamp(powerRampPosition, startDeltaAngle, 0.1);


         //turn = (turn > 0) ? Range.clip(turn, 0.1, 1) : Range.clip(turn, -1, -0.1);


         //        Check timeout             //
         //double elapsedTime = Math.abs(System.currentTimeMillis() - startTime);
         //if (elapsedTime > 3000) break;


         //        Set Power                 //
         setDrivePower(0, 0, turn_direction, power);
         current_angle = imu.getAngle();



         //          Logging                 //
         Utils.multTelemetry.addData("Error", error);
         Utils.multTelemetry.addData("Turn", turn_direction);
         Utils.multTelemetry.addData("Power", power);
         Utils.multTelemetry.addData("IMU", current_angle);
         Utils.multTelemetry.addData("Finished", (Math.abs(error) <= MOE));
         Utils.multTelemetry.update();
      }
      setAllPower(0);
   }



   @RequiresApi(api = Build.VERSION_CODES.N)
   public void turn(double target_angle, double MOE, double acceleration, SyncTask task) {

      double startTime = System.currentTimeMillis();
      double turn_direction, pid_return, power, powerRampPosition;


      //            Calc Power Ramping and PID Values           //
      double current_angle = imu.getAngle();
      double startAngle = current_angle;
      double actual_target_angle = findClosestAngle(target_angle, current_angle);
      double startDeltaAngle = Math.abs(actual_target_angle - current_angle);
      double error = actual_target_angle - current_angle;



      while ((Math.abs(error) > MOE) && Utils.isActive()) {

         if (task != null) task.execute();


         //              PID                     //
         error = actual_target_angle - current_angle;
         pid_return = rotationPID.update(error) * -1;
         turn_direction = (pid_return > 0) ? 1 : -1;


         //              Power Ramping            //
         powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
         power = Utils.powerRamp(powerRampPosition, startDeltaAngle, acceleration);


         //        Set Power                 //
         setDrivePower(0, 0, turn_direction, power);
         current_angle = imu.getAngle();



         //          Logging                 //
         Utils.multTelemetry.addData("Error", error);
         Utils.multTelemetry.addData("Turn", turn_direction);
         Utils.multTelemetry.addData("Power", power);
         Utils.multTelemetry.addData("IMU", current_angle);
         Utils.multTelemetry.addData("Finished", (Math.abs(error) <= MOE));
         Utils.multTelemetry.update();
      }
      setAllPower(0);
   }



   @RequiresApi(api = Build.VERSION_CODES.N)
   public double giveTurn(double target_angle, double MOE, SyncTask task, double acceleration) {

      //            Calc Power Ramping and PID Values           //
      double turn_direction, pid_return, power, powerRampPosition;
      double current_angle = imu.getAngle();
      double startAngle = current_angle;
      double actual_target_angle = findClosestAngle(target_angle, current_angle);
      double startDeltaAngle = Math.abs(actual_target_angle - current_angle);
      double error = actual_target_angle - current_angle;


      if ((Math.abs(error) > MOE) && Utils.isActive()) {

         if (task != null) task.execute();

         //                   PID                 //
         error = actual_target_angle - current_angle;
         pid_return = rotationPID.update(error) * -1;
         turn_direction = (pid_return > 0) ? 1 : -1;

         //              Power Ramping            //
         powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
         power = Utils.powerRamp(powerRampPosition, startDeltaAngle, acceleration);

         //             Set Power                 //
         return turn_direction * power;
      }
      else return 0.0;
   }



   public enum PSState {
      LEFT, CENTER, RIGHT, TURNING_CENTER, TURNING_LEFT, END
   }
   public PSState current_ps_state = RIGHT;
   public ElapsedTime time = new ElapsedTime();

   @RequiresApi(api = Build.VERSION_CODES.N)
   public void turnPowerShot(double MOE, SyncTask syncTask) {

      double current_angle = imu.getAngle();
      double turn_direction = 0;
      double power = 0;
      double actual_target_angle, error, pid_return;


      //          STATE MACHINE           //
      switch (current_ps_state){

         case RIGHT:
            if (shooter.getFeederCount() < 1) shooter.feederState(true);
            else current_ps_state = TURNING_CENTER;
            Utils.multTelemetry.addData("Status", "Shooting RIGHT");
            break;


         case TURNING_CENTER:
            actual_target_angle = findClosestAngle(88, current_angle);
            error = actual_target_angle - current_angle;
            turn_direction = (rotationPID.update(error) * -1 > 0) ? 1 : -1;
            power = 0.2;

            //    DRIVE IF WE HAVEN'T REACHED TARGET     //
            if (Math.abs(error) > MOE) power = 0.2;
            else {
               current_ps_state = CENTER;
               shooter.setFeederCount(0);
            }

            Utils.multTelemetry.addData("Finished Turning CENTER", (Math.abs(error) <= MOE));
            break;


         case CENTER:
            power = 0;
            if (shooter.getFeederCount() < 1) shooter.feederState(true);
            else current_ps_state = TURNING_LEFT;
            Utils.multTelemetry.addData("Status", "Shooting CENTER");
            break;


         case TURNING_LEFT:

            actual_target_angle = findClosestAngle(92, current_angle);
            error = actual_target_angle - current_angle;
            pid_return = rotationPID.update(error) * -1;
            turn_direction = (pid_return > 0) ? 1 : -1;
            power = 0.2;

            //    DRIVE IF WE HAVEN'T REACHED TARGET     //
            if (Math.abs(error) > MOE) power = 0.2;
            else {
               shooter.setFeederCount(0);
               current_ps_state = CENTER;
            }

            Utils.multTelemetry.addData("Finished Turning LEFT", (Math.abs(error) <= MOE));
            break;

         case LEFT:
            power = 0;
            if (shooter.getFeederCount() < 1) shooter.feederState(true);
            else current_ps_state = END;

            Utils.multTelemetry.addData("Status", "Shooting LEFT");
            break;
      }

      setDrivePower(0, 0, turn_direction, power);
   }

   public void turnPowerRamp(double targetAngle, double MOE) {
         System.out.println("Turning to " + targetAngle + " degrees");
         double power;
         double startAngle = imu.getAngle();
         double currentAngle = imu.getAngle();
         double deltaAngle = Math.abs(targetAngle - currentAngle);
         double position = getPosition();

         // Retrieve angle and MOE
         double upperBound = targetAngle + MOE;
         double lowerBound = targetAngle - MOE;
         while ((lowerBound >= currentAngle || currentAngle >= upperBound) && Utils.isActive()) {

            // Power Ramping based off a logistic piecewise
            double currentDeltaAngle = targetAngle - currentAngle;
            double anglePosition = deltaAngle - currentDeltaAngle + 0.01; // Added the 0.01 so that it doesn't get stuck at 0
            double relativePosition = map(currentAngle, startAngle, targetAngle, 0, deltaAngle);
            double direction = -Math.signum(currentDeltaAngle);
            // RelativePosition must be calculated to match domain restrictions of powerRamp
            // We want to map our currentAngle relative to a range of [0, and distance it needs to travel]

            // Modeling a piece wise of power as a function of distance
            power = Utils.powerRamp(relativePosition, deltaAngle, 0.05);

            // Handle clockwise (+) and counterclockwise (-) motion
            setDrivePower(0, 0, direction, power);

            currentAngle = imu.getAngle();

            Utils.telemetry.addData("IMU", imu.getAngle());
            Utils.telemetry.addData("Direction", direction);
            Utils.telemetry.addData("Relative Position", relativePosition);
            Utils.telemetry.addData("Delta Angle", deltaAngle);
            Utils.telemetry.addData("Power", power);

            Utils.telemetry.addData("Lower", lowerBound);
            Utils.telemetry.addData("Upper", upperBound);
            Utils.telemetry.update();
         }

         // Stop power
         setAllPower(0);

         sleep(100);
      }

      public double getTurn(){
         return turn;
      }
}