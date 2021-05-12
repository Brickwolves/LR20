package org.firstinspires.ftc.teamcode.Hardware;

import android.os.Build;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Navigation.Odometry;
import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Navigation.Point;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.floorMod;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static java.lang.StrictMath.abs;
import static java.lang.StrictMath.atan2;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.min;
import static java.lang.StrictMath.pow;
import static java.lang.StrictMath.sin;
import static java.lang.StrictMath.sqrt;
import static java.lang.StrictMath.toRadians;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.isActive;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.print;


public class Mecanum implements Robot {

   private DcMotor fr, fl, br, bl;

   public Odometry odom;
   public IMU imu;
   public Arm arm;
   public Claw claw;
   public Intake2 intake;
   public Shooter shooter;
   public Wings wings;

   public PID rotationPID = new PID(Dash_Movement.p, Dash_Movement.i, Dash_Movement.d, 0, 100, true);

   public static ElapsedTime time = new ElapsedTime();
   public static RingBuffer<Double> angleBuffer          = new RingBuffer<>(5, 0.0);
   public static RingBuffer<Double> frBuffer             = new RingBuffer<>(5,  0.0);
   public static RingBuffer<Double> flBuffer             = new RingBuffer<>(5,  0.0);
   public static RingBuffer<Double> brBuffer             = new RingBuffer<>(5,  0.0);
   public static RingBuffer<Double> blBuffer             = new RingBuffer<>(5,  0.0);
   public static RingBuffer<Double> xPosBuffer           = new RingBuffer<>(5,  0.0);
   public static RingBuffer<Double> yPosBuffer           = new RingBuffer<>(5,  0.0);

   public static RingBuffer<Double> intakeBuffer         = new RingBuffer<>(5,  0.0);

   public static RingBuffer<Double> timeBuffer           = new RingBuffer<>(5,  0.0);

   public static Map<String, Double> motorRPMs = new HashMap<String, Double>();


   public enum Units {
      TICKS, INCHES, FEET
   }

   public Mecanum(){
      initRobot();
   }

   public void initRobot() {
      multTelemetry.addData("Status", "Initialized");
      multTelemetry.update();

      // Init Motors
      fr = hardwareMap.get(DcMotor.class, "front_right_motor");
      fl = hardwareMap.get(DcMotor.class, "front_left_motor");
      br = hardwareMap.get(DcMotor.class, "back_right_motor");
      bl = hardwareMap.get(DcMotor.class, "back_left_motor");
      resetMotors();

      motorRPMs.put("FR", 0.0);
      motorRPMs.put("FL", 0.0);
      motorRPMs.put("BR", 0.0);
      motorRPMs.put("BL", 0.0);

      imu = new IMU("imu");
      odom = new Odometry(0, 0, imu.getAngle());
      claw = new Claw("eservo_2", "eservo_1");
      arm = new Arm("eservo_0");
      intake = new Intake2("intake", "cservo_2", "cservo_4");
      shooter = new Shooter("spinny_1", "spinny_2", "cservo_1", "cservo_0");
      wings = new Wings("cservo_3", "cservo_5");
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



   public static double powerRamp(double position, double distance, double acceleration){
      /*
       *  The piece wise function has domain restriction [0, inf] and range restriction [0, 1]
       *  Simply returns a proportional constant
       */

      double relativePosition = (position / distance) * 10; // Mapped on [0, 10]

      // Modeling a piece wise of power as a function of distance
      double p1       = sqrt(acceleration * relativePosition);
      double p2       = 1;
      double p3       = (sqrt(acceleration * (10 - relativePosition)));
      double power    = min(min(p1, p2), p3);
      power           = clip(power, 0.2, 1);

      return power;
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
   @RequiresApi(api = Build.VERSION_CODES.N)
   public static double findClosestAngle(double targetAngle, double currentAngle) {
      double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
      return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
   }

   /**
    * @param currentAngle
    * @return
    */
   @RequiresApi(api = Build.VERSION_CODES.N)
   public static double findClosestAngleNearGoal(double currentAngle){

      double MOE = 20;
      double a1 = 180 - MOE;
      double a2 = 180 + MOE;

      double simpleTargetDelta1 = floorMod(Math.round(((360 - a1) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta1 = -1 * (360 - simpleTargetDelta1);
      double min1 = min(abs(simpleTargetDelta1), abs(alternateTargetDelta1));

      double simpleTargetDelta2 = floorMod(Math.round(((360 - a2) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta2 = -1 * (360 - simpleTargetDelta2);
      double min2 = min(abs(simpleTargetDelta2), abs(alternateTargetDelta2));

      double final_angle = (min1 > min2) ? a2 : a1;
      return findClosestAngle(final_angle, currentAngle);
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
      if (sin(r) == 0) unShiftedX = 0;
      return new Point(unShiftedX, unShiftedY);
   }

   public double getRComp(){
      double r1 = 0.5 * (bl.getCurrentPosition() - fr.getCurrentPosition());
      double r2 = 0.5 * (fl.getCurrentPosition() - br.getCurrentPosition());
      return (r1 + r2) / 2.0;
   }

   public double getRComp(double fr, double fl, double br, double bl){
      double r1 = 0.5 * (bl - fr);
      double r2 = 0.5 * (fl - br);
      return (r1 + r2) / 2.0;
   }

   public double getXComp(){
      double x1 = 0.5 * (fl.getCurrentPosition() + br.getCurrentPosition() - (2 * getYComp()));
      double x2 = -0.5 * (fr.getCurrentPosition() + bl.getCurrentPosition() - (2 * getYComp()));
      return (x1 + x2) / 2.0;
   }

   public double getXComp(double fr, double fl, double br, double bl){
      double x1 = 0.5 * (fl + br - (2 * getYComp()));
      double x2 = -0.5 * (fr + bl - (2 * getYComp()));
      return (x1 + x2) / 2.0;
   }

   public double getYComp(){
      return (fr.getCurrentPosition() + fl.getCurrentPosition() + br.getCurrentPosition() + bl.getCurrentPosition()) / 4.0;
   }

   public double getYComp(double fr, double fl, double br, double bl){
      return (fr + fl + br + bl) / 4.0;
   }

   public void update(){

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


   public double eurekaSub(double x){
      return x - toRadians(10) * sin(4 * x);
   }

   public double correctTargetRadians(double x){
      double c1 = 7.8631064977;
      double c2 = 8;
      return x + toRadians(c2) * sin(4 * x - 0.2);
   }

   public void linearStrafe(double angle, double cm, double acceleration, double targetAngle, double waitTurnTime, double waitTaskTime, SyncTask task) {

      resetMotors();

      ElapsedTime time = new ElapsedTime(); time.reset();

      double strafeAngle = toRadians(angle);
      double ticks = cm; //centimeters2Ticks(cm);

      Orientation startO = odom.getOrientation();
      Orientation curO = odom.getOrientation();

      double power;
      double px0 = cos(strafeAngle);
      double py0 = -sin(strafeAngle);
      double pr0 = 0;

      print("PX: " + px0);
      print("PY: " + py0);
      print("StrafeAngle: " + strafeAngle);
      print("\n");

      double curC = 0;
      while (curC < ticks && isActive()){

         // Execute task synchronously
         if (task != null && time.seconds() > waitTaskTime) task.execute();

         // Power ramping
         power = powerRamp(curC, ticks, acceleration);

         // PID CONTROLLER
         pr0 = clip(rotationPID.update(targetAngle - imu.getAngle()) * -1, -1, 1);
         if (time.seconds() < waitTurnTime) pr0 = 0;

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
         //System.out.println("atan2(y, x): " + toDegrees(atan2(curO.y, curO.x)));
         multTelemetry.addData("Power", power);
         multTelemetry.addData("curC", curC);
         multTelemetry.addData("Ticks", ticks);
         multTelemetry.update();
      }
      odom.update(curO);
      setAllPower(0);
   }


   public void linearStrafe(Orientation destination, double acceleration, SyncTask task){

      // Initialize starter variables
      resetMotors();

      destination.y *= -1;     // NO IDEA WHY

      /*
      dest.x = (dest.x > 0) ? centimeters2Ticks(dest.x) : -centimeters2Ticks(abs(dest.x));
      dest.y = (dest.y > 0) ? centimeters2Ticks(dest.y): -centimeters2Ticks(abs(dest.y));

      dest.x = (dest.x == 7) ? 0 : dest.x;
      dest.y = (dest.y == 7) ? 0 : dest.y;
       */

      // Retrieve current positions
      Orientation startO = odom.getOrientation();
      Orientation curO = new Orientation(startO.x, startO.y, startO.a);

      // Calculate distances to travel
      double distX = destination.x - startO.x;
      double distY = destination.y - startO.y;
      double distC = sqrt(pow(distX, 2) + pow(distY, 2));

      // Calculate strafe angle
      double strafeAngle = correctTargetRadians(atan2(distY, distX)); //double strafeAngle = atan2(distY, distX);

      // Calculate powers to move
      double px0 = cos(strafeAngle);
      double py0 = sin(strafeAngle);
      double pr = 0;

      // Logging
      print("DISTX: " + distX);
      print("DISTY: " + distY);
      print("PX: " + px0);
      print("PY: " + py0);
      print("StrafeAngle: " + strafeAngle);
      print("\n");

      double curC = 0;
      while (curC < distC && isActive()){

         // Execute task synchronously
         if (task != null) task.execute();

         // Get current position
         Point relPos = unShift(getXComp(), getYComp(), curO.a % 360);
         curO.x = relPos.x + startO.x;
         curO.y = relPos.y + startO.y;
         curO.a = imu.getAngle();
         curC = sqrt(pow(relPos.x, 2) + pow(relPos.y, 2));


         // Set Driver Power
         double power = powerRamp(curC, distC, acceleration);
         Point shiftedPowers = shift(px0, py0, curO.a % 360);
         pr = clip(rotationPID.update(destination.a - imu.getAngle()) * -1, -1, 1);
         setDrivePower(shiftedPowers.y, shiftedPowers.x, pr, power);

         // LOGGING
         multTelemetry.addData("Power", power);
         multTelemetry.addData("curC", curC);
         multTelemetry.addData("distC", distC);
         multTelemetry.update();
      }
      odom.update(curO);
      setAllPower(0);
   }

   @RequiresApi(api = Build.VERSION_CODES.N)
   public void linearTurn(double target_angle, double MOE, SyncTask task) {

      double turn_direction, pid_return, power, powerRampPosition;


      //            Calc Power Ramping and PID Values           //
      double current_angle = imu.getAngle();
      double startAngle = current_angle;
      double actual_target_angle = findClosestAngle(target_angle, current_angle);
      double startDeltaAngle = Math.abs(actual_target_angle - current_angle);
      double error = actual_target_angle - current_angle;



      while ((Math.abs(error) > MOE) && isActive()) {

         // Execute task synchronously
         if (task != null) task.execute();

         //              PID                     //
         error = actual_target_angle - current_angle;
         pid_return = rotationPID.update(error) * -1;
         turn_direction = (pid_return > 0) ? 1 : -1;


         //              Power Ramping            //
         powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
         power = powerRamp(powerRampPosition, startDeltaAngle, 0.1);


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
   public void linearTurn(double target_angle, double MOE) {

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
         power = powerRamp(powerRampPosition, startDeltaAngle, 0.1);


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
   public void linearTurn(double target_angle, double MOE, double acceleration, SyncTask task) {

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
         power = powerRamp(powerRampPosition, startDeltaAngle, acceleration);


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
}