package org.firstinspires.ftc.teamcode.Hardware;

import android.os.Build;

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
import org.firstinspires.ftc.teamcode.Utilities.Task;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.floorMod;
import static java.lang.StrictMath.abs;
import static java.lang.StrictMath.atan2;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.min;
import static java.lang.StrictMath.pow;
import static java.lang.StrictMath.sin;
import static java.lang.StrictMath.sqrt;
import static java.lang.StrictMath.toRadians;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.closestAngle;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.shift;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.unShift;
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
   public Intake intake;
   public Shooter shooter;
   public Wings wings;

   public PID rotationPID = new PID(Dash_Movement.p, Dash_Movement.i, Dash_Movement.d, 0, 100, true);
   public static ElapsedTime time = new ElapsedTime();

   public enum Units {
      TICKS, INCHES, FEET
   }

   public Mecanum(){
      initRobot();
   }

   public void initRobot() {

      // Init Motors
      fr = hardwareMap.get(DcMotor.class, "front_right_motor");
      fl = hardwareMap.get(DcMotor.class, "front_left_motor");
      br = hardwareMap.get(DcMotor.class, "back_right_motor");
      bl = hardwareMap.get(DcMotor.class, "back_left_motor");
      resetMotors();

      imu      = new IMU("imu");
      odom     = new Odometry(0, 0, imu.getAngle());
      claw     = new Claw("eservo_2", "eservo_1");
      arm      = new Arm("eservo_0");
      intake   = new Intake("intake", "cservo_2", "cservo_4");
      shooter  = new Shooter("spinny_1", "spinny_2", "cservo_1", "cservo_0");
      wings    = new Wings("cservo_3", "cservo_5");

      multTelemetry.addData("Status", "Initialized");
      multTelemetry.update();
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
    * @param position
    * @param distance
    * @param acceleration
    * @return the coefficient [0, 1] of our velocity
    */
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

   public double eurekaSub(double x){
      return x - toRadians(10) * sin(4 * x);
   }

   public double correctTargetRadians(double x){
      double c1 = 7.8631064977;
      double c2 = 8;
      return x + toRadians(c2) * sin(4 * x - 0.2);
   }

   public void linearStrafe(double angle, double cm, double acceleration, double targetAngle, double waitTurnTime, double waitTaskTime, Task task) {

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


   public void linearStrafe(Orientation destination, double acceleration, Task task){

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
   public void linearTurn(double target_angle, double MOE, Task task) {

      double turn_direction, pid_return, power, powerRampPosition;


      //            Calc Power Ramping and PID Values           //
      double current_angle = imu.getAngle();
      double startAngle = current_angle;
      double actual_target_angle = closestAngle(target_angle, current_angle);
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
      double actual_target_angle = closestAngle(target_angle, current_angle);
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
   public void linearTurn(double target_angle, double MOE, double acceleration, Task task) {

      double turn_direction, pid_return, power, powerRampPosition;


      //            Calc Power Ramping and PID Values           //
      double current_angle = imu.getAngle();
      double startAngle = current_angle;
      double actual_target_angle = closestAngle(target_angle, current_angle);
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