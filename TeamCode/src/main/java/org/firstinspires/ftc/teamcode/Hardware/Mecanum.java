package org.firstinspires.ftc.teamcode.Hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Navigation.Odometry;
import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Navigation.Point;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;


import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.floorMod;
import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.abs;
import static java.lang.StrictMath.atan2;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.max;
import static java.lang.StrictMath.min;
import static java.lang.StrictMath.pow;
import static java.lang.StrictMath.sin;
import static java.lang.StrictMath.sqrt;
import static java.lang.StrictMath.toDegrees;
import static java.lang.StrictMath.toRadians;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.*;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.convertInches2Ticks;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.isActive;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.map;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;


public class Mecanum implements Robot {

   private DcMotor fr, fl, br, bl;

   public IMU imu;
   public Odometry odom;
   public Arm arm;
   public Claw claw;
   public Intake intake;
   public Shooter shooter;

   public PID rotationPID = new PID(Dash_Movement.p, Dash_Movement.i, Dash_Movement.d, 0, 100, true);

   public Mecanum(){
      initRobot();
   }

   public void initRobot() {
      multTelemetry.addData("Status", "Initialized");

      // Init Motors
      fr = hardwareMap.get(DcMotor.class, "front_right_motor");
      fl = hardwareMap.get(DcMotor.class, "front_left_motor");
      br = hardwareMap.get(DcMotor.class, "back_right_motor");
      bl = hardwareMap.get(DcMotor.class, "back_left_motor");
      resetMotors();

      // Sensors
      //touchSensor = Utils.hardwareMap.get(TouchSensor.class, "touch_sensor");
      //colorSensorBase = Utils.hardwareMap.get(ColorSensor.class, "color_sensor");
      //colorSensor = new ColorSensorImpl(colorSensorBase);
      imu = new IMU("imu");
      odom = new Odometry(0, 0, imu.getAngle());
      claw = new Claw("eservo_2", "eservo_1");
      arm = new Arm("eservo_0");
      intake = new Intake("intake", "cservo_2", "cservo_4");
      shooter = new Shooter("spinny_1", "spinny_2", "cservo_1", "cservo_0");
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

   /**
    * @param targetAngle
    * @return
    */
   @RequiresApi(api = Build.VERSION_CODES.N)
   public double closestRelativeAngle(double targetAngle){
      double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + imu.getAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
      return StrictMath.abs(simpleTargetDelta) >= StrictMath.abs(alternateTargetDelta) ? 0 - simpleTargetDelta : 0 - alternateTargetDelta;
   }

   /**
    * @param targetAngle
    * @param currentAngle
    * @return
    */
   //@RequiresApi(api = Build.VERSION_CODES.N)
   @RequiresApi(api = Build.VERSION_CODES.N)
   public static double findClosestAngle(double targetAngle, double currentAngle){
      double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
      return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
   }


   /**
    * @param position
    * @param distance
    * @param acceleration
    * @return
    */
   public static double powerRamp(double position, double distance, double acceleration, double maxVelocity){
      /*
       *  The piece wise function has domain restriction [0, inf] and range restriction [0, 1]
       *  Simply returns a proportional constant
       */


      // Constant to map power to [0, 1]
      double normFactor = maxVelocity / Math.sqrt(acceleration * distance);

      // Modeling a piece wise of power as a function of distance
      double p1       = normFactor * Math.sqrt(acceleration * position);
      double p2       = maxVelocity;
      double p3       = normFactor * (Math.sqrt(acceleration * (distance - position)));
      double power    = Math.min(Math.min(p1, p2), p3) + 0.1;
      power           = clip(power, 0.1, 1);

      return power;
   }

   private double powerRamp(double percentageDistance, double a){

      double p1 = sqrt(a * percentageDistance);
      double p2 = 1;
      double p3 = sqrt(a * (1 - percentageDistance));

      double p = min(min(p1, p2), p3);
      p = clip(p, 0.1, 1);

      return p;
   }

   public static Point shift(double x, double y, double shiftAngle){
      double shiftedX = (x * Math.sin(toRadians(shiftAngle))) + (y * cos(toRadians(shiftAngle)));
      double shiftedY = (x * Math.cos(toRadians(shiftAngle))) - (y * sin(toRadians(shiftAngle)));
      return new Point(shiftedX, shiftedY);
   }

   public static Point unShift(double x, double y, double shiftAngle){
      double r = toRadians(shiftAngle);
      double unShiftedY = ((x * cos(r)) - (y * sin(r)))   /  (pow(cos(r), 2) + pow(sin(r), 2));
      double unShiftedX = (x - (unShiftedY * cos(r))) / sin(r);
      return new Point(unShiftedX, unShiftedY);
   }

   public double getRComp(){
      double r1 = 0.5 * (bl.getCurrentPosition() - fr.getCurrentPosition());
      double r2 = 0.5 * (fl.getCurrentPosition() - br.getCurrentPosition());
      return (r1 + r2) / 2.0;
   }

   public double getXComp(){
      double x1 = 0.5 * (fl.getCurrentPosition() + br.getCurrentPosition() - (2 * getYComp()));
      double x2 = -0.5 * (fr.getCurrentPosition() + bl.getCurrentPosition() - (2 * getYComp()));
      return (x1 + x2) / 2.0;
   }

   public double getYComp(){
      return (fr.getCurrentPosition() + fl.getCurrentPosition() + br.getCurrentPosition() + bl.getCurrentPosition()) / 4.0;
   }

   public double eurekaSub(double x){
      return x - toRadians(10) * sin(4 * x);
   }

   public double correctTargetRadians(double x){
      double c1 = 7.8631064977;
      double c2 = 8;
      return x + toRadians(c2) * sin(4 * x - 0.2);
   }

   public enum Units {
      TICKS, INCHES, FEET
   }

   public void linearStrafe(Point dest, double acceleration, SyncTask task){

      // Initialize starter variables
      resetMotors();
      //double ticks = convertInches2Ticks(inches);

      // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
      dest.y *= -1;
      Orientation startO = odom.getOrientation();
      Orientation curO = new Orientation(startO.x, startO.y, startO.a);
      double distX = dest.x - startO.x;
      double distY = dest.y - startO.y;
      double distC = sqrt(pow(distX, 2) + pow(distY, 2));

      //double targetRadians = eurekaPlus(atan2(distY, distX));
      double targetRadians = atan2(distY, distX);

      // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
      // The yPower and xPower should maintain the same ratio with each other
      double power;
      double maxPower = 1;
      double px0 = cos(targetRadians);                  // Fill out power to a max of 1
      double py0 = sin(targetRadians);                  // Fill out power to a max of 1
      double normalizeToPower = maxPower / max(abs(px0), abs(py0));
      px0 *= normalizeToPower;
      py0 *= normalizeToPower;
      double pr0 = 0;

      double curC = 0;
      while (curC < distC && isActive()){

         // Execute task synchronously
         if (task != null) task.execute();

         // Power ramping
         power = powerRamp(curC, distC, acceleration, maxPower);

         // PID CONTROLLER
         pr0 = clip(rotationPID.update(startO.a - imu.getAngle()) * -1, -1, 1);

         // SHIFT POWER
         Point shiftedPowers = shift(px0, py0, curO.a % 360);

         // Un-shift X and Y distances traveled
         Point relPos = unShift(getXComp(), getYComp(), curO.a % 360);
         curO.x = relPos.x + startO.x;
         curO.y = relPos.y + startO.y;
         curO.a = imu.getAngle();
         curC = sqrt(pow(relPos.x, 2) + pow(relPos.y, 2));

         // SET POWER
         setDrivePower(shiftedPowers.y, shiftedPowers.x, pr0, power);

         // LOGGING
         System.out.println("atan2(y, x): " + toDegrees(atan2(curO.y, curO.x)));
         multTelemetry.addData("Power", power);
         multTelemetry.addData("curC", curC);
         multTelemetry.addData("distC", distC);
         multTelemetry.update();
      }
      odom.update(curO);
      setAllPower(0);
   }

   /**
    * @param angle
    */
   public void linearStrafe(double angle, double distance, double acceleration, SyncTask task) {

      double r = angle * PI / 180.0;
      Point dest = new Point(distance * cos(r), distance * sin(r) * -1);
      linearStrafe(dest, acceleration, task);
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



      while ((Math.abs(error) > MOE) && isActive()) {

         //              PID                     //
         error = actual_target_angle - current_angle;
         pid_return = rotationPID.update(error) * -1;
         turn_direction = (pid_return > 0) ? 1 : -1;


         //              Power Ramping            //
         powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
         power = OpModeUtils.powerRamp(powerRampPosition, startDeltaAngle, 0.1);


         //turn = (turn > 0) ? Range.clip(turn, 0.1, 1) : Range.clip(turn, -1, -0.1);


         //        Check timeout             //
         //double elapsedTime = Math.abs(System.currentTimeMillis() - startTime);
         //if (elapsedTime > 3000) break;


         //        Set Power                 //
         setDrivePower(0, 0, turn_direction, power);
         current_angle = imu.getAngle();



         //          Logging                 //
         multTelemetry.addData("Error", error);
         multTelemetry.addData("Turn", turn_direction);
         multTelemetry.addData("Power", power);
         multTelemetry.addData("IMU", current_angle);
         multTelemetry.addData("Finished", (Math.abs(error) <= MOE));
         multTelemetry.update();
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



      while ((Math.abs(error) > MOE) && isActive()) {

         if (task != null) task.execute();


         //              PID                     //
         error = actual_target_angle - current_angle;
         pid_return = rotationPID.update(error) * -1;
         turn_direction = (pid_return > 0) ? 1 : -1;


         //              Power Ramping            //
         powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
         power = OpModeUtils.powerRamp(powerRampPosition, startDeltaAngle, acceleration);


         //        Set Power                 //
         setDrivePower(0, 0, turn_direction, power);
         current_angle = imu.getAngle();



         //          Logging                 //
         multTelemetry.addData("Error", error);
         multTelemetry.addData("Turn", turn_direction);
         multTelemetry.addData("Power", power);
         multTelemetry.addData("IMU", current_angle);
         multTelemetry.addData("Finished", (Math.abs(error) <= MOE));
         multTelemetry.update();
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


      if ((Math.abs(error) > MOE) && isActive()) {

         if (task != null) task.execute();

         //                   PID                 //
         error = actual_target_angle - current_angle;
         pid_return = rotationPID.update(error) * -1;
         turn_direction = (pid_return > 0) ? 1 : -1;

         //              Power Ramping            //
         powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
         power = OpModeUtils.powerRamp(powerRampPosition, startDeltaAngle, acceleration);

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
            multTelemetry.addData("Status", "Shooting RIGHT");
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

            multTelemetry.addData("Finished Turning CENTER", (Math.abs(error) <= MOE));
            break;


         case CENTER:
            power = 0;
            if (shooter.getFeederCount() < 1) shooter.feederState(true);
            else current_ps_state = TURNING_LEFT;
            multTelemetry.addData("Status", "Shooting CENTER");
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

            multTelemetry.addData("Finished Turning LEFT", (Math.abs(error) <= MOE));
            break;

         case LEFT:
            power = 0;
            if (shooter.getFeederCount() < 1) shooter.feederState(true);
            else current_ps_state = END;

            multTelemetry.addData("Status", "Shooting LEFT");
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

         // Retrieve angle and MOE
         double upperBound = targetAngle + MOE;
         double lowerBound = targetAngle - MOE;
         while ((lowerBound >= currentAngle || currentAngle >= upperBound) && isActive()) {

            // Power Ramping based off a logistic piecewise
            double currentDeltaAngle = targetAngle - currentAngle;
            double anglePosition = deltaAngle - currentDeltaAngle + 0.01; // Added the 0.01 so that it doesn't get stuck at 0
            double relativePosition = map(currentAngle, startAngle, targetAngle, 0, deltaAngle);
            double direction = -Math.signum(currentDeltaAngle);
            // RelativePosition must be calculated to match domain restrictions of powerRamp
            // We want to map our currentAngle relative to a range of [0, and distance it needs to travel]

            // Modeling a piece wise of power as a function of distance
            power = OpModeUtils.powerRamp(relativePosition, deltaAngle, 0.05);

            // Handle clockwise (+) and counterclockwise (-) motion
            setDrivePower(0, 0, direction, power);

            currentAngle = imu.getAngle();

            OpModeUtils.telemetry.addData("IMU", imu.getAngle());
            OpModeUtils.telemetry.addData("Direction", direction);
            OpModeUtils.telemetry.addData("Relative Position", relativePosition);
            OpModeUtils.telemetry.addData("Delta Angle", deltaAngle);
            OpModeUtils.telemetry.addData("Power", power);

            OpModeUtils.telemetry.addData("Lower", lowerBound);
            OpModeUtils.telemetry.addData("Upper", upperBound);
            OpModeUtils.telemetry.update();
         }

         // Stop power
         setAllPower(0);

         sleep(100);
      }
}