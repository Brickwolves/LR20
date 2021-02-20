package org.firstinspires.ftc.teamcode.Hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import static android.os.SystemClock.sleep;
import static java.lang.Math.floorMod;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.SHOOT_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_ServoDiagnostic.LOCK_SERVO_HOME;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.convertInches2Ticks;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.map;


public class MecanumRobot implements Robot {

   private DcMotor fr, fl, br, bl;
   //public TouchSensor touchSensor;
   //public ColorSensorImpl colorSensor;
   //public ColorSensor colorSensorBase;
   public IMU imu;
   public Claw claw;
   public Arm arm;
   public Intake intake;
   public Shooter shooter;
   private DcMotor spinny_1, spinny_2;
   private Servo lock_servo, shoot_servo;

   private LinearOpMode opMode;

   private double initAngle;
   private double currentPosition;

   public PID rotationPID = new PID(Dash_Movement.p, Dash_Movement.i, Dash_Movement.d, 100, true);

   public MecanumRobot(){
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
      claw = new Claw("servo_1");
      arm = new Arm("servo_0");
      intake = new Intake("intake", "servo_2", "servo_3");
      shooter = new Shooter("spinny_1", "spinny_2", "servo_4", "servo_5");

      initAngle = imu.getAngle();
   }

   /**
    * (Re)Init Motors
    */
   public void resetMotors(){
      try {

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
      catch (Exception e){
         throw new Error("Could not initialize motors, and therefore cannot set mode");
      }
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


   /**
    * @param angle
    * @param inches
    */
   public void strafe(double angle, double inches, double directionFacingAngle, double acceleration, SyncTask task){

      System.out.println(angle + " " + inches);
      double ticks = convertInches2Ticks(inches);


      resetMotors();                                              // Reset Motor Encoder
      double radians = (angle + 90) * Math.PI / 180;              // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
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


         Utils.multTelemetry.addData("Error", error);
         Utils.multTelemetry.addData("Turn", turn);
         Utils.multTelemetry.addData("Drive", drive * power);
         Utils.multTelemetry.addData("Strafe", strafe * power);
         Utils.multTelemetry.update();
      }
      setAllPower(0);
      sleep(200);
   }

   @RequiresApi(api = Build.VERSION_CODES.N)
   public static double turnTarget(double targetAngle, double currentAngle){
      double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
      double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
      return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
   }

   @RequiresApi(api = Build.VERSION_CODES.N)
   public void turn(double target_angle, double MOE) {

      double startTime = System.currentTimeMillis();

      double turn;
      double current_angle = imu.getAngle();
      double actual_target_angle = turnTarget(target_angle, current_angle);
      double error = actual_target_angle - current_angle;

      while ((Math.abs(error) > MOE) && Utils.isActive()) {
         actual_target_angle = turnTarget(target_angle, current_angle);
         error = actual_target_angle - current_angle;
         turn = rotationPID.update(error) * -1;
         setDrivePower(0, 0, turn, Dash_Movement.velocity);

         current_angle = imu.getAngle();

         // Check timeout
         double elapsedTime = Math.abs(System.currentTimeMillis() - startTime);
         if (elapsedTime > 3000) break;


         Utils.multTelemetry.addData("Error", error);
         Utils.multTelemetry.addData("Turn", turn);
         Utils.multTelemetry.addData("IMU", current_angle);
         Utils.multTelemetry.update();
      }
   }


   public void turnPID(double targetAngle, double MOE){

      double startTime = System.currentTimeMillis();

      double currentAngle = imu.getAngle();
      double error = targetAngle - currentAngle;

      while ((Math.abs(error) > MOE) && Utils.isActive()) {
         error = targetAngle - currentAngle;
         double turn = rotationPID.update(error) * -1;
         setDrivePower(0 ,0, turn, Dash_Movement.velocity);

         currentAngle = imu.getAngle();


         double elapsedTime = Math.abs(System.currentTimeMillis() - startTime);
         if (elapsedTime > 3000) break;


         Utils.multTelemetry.addData("Error", error);
         Utils.multTelemetry.addData("Turn", turn);
         Utils.multTelemetry.addData("IMU", currentAngle);
         Utils.multTelemetry.update();
      }

      setDrivePower(0, 0, 0, 0);
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
}