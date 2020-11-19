package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import java.util.ArrayList;

public class MecanumRobot implements Robot {

   private HardwareMap hardwareMap;
   private DcMotor fr, fl, br, bl;
   public TouchSensor touchSensor;
   public ColorSensor colorSensor;
   public WebcamName webCam;
   public IMU imu;


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
      initMotors();

      // Sensors
      touchSensor = Utils.hardwareMap.get(TouchSensor.class, "touch_sensor");
      colorSensor = Utils.hardwareMap.get(ColorRangeSensor.class, "color_sensor");
      webCam = Utils.hardwareMap.get(WebcamName.class, "webcam");
      imu = new IMU("imu");
   }

   /**
    * (Re)Init Motors
    */
   public void initMotors(){
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

   /**
    * @param targetAngle
    * @param MOE
    */
   public double autoTurn(double targetAngle, double MOE) {

      System.out.println("Turning to " + targetAngle + " degrees");

      targetAngle += imu.getDeltaAngle();
      double currentAngle = imu.getAngle();
      double upperBound = targetAngle + MOE;
      double lowerBound = targetAngle - MOE;
      if (lowerBound >= currentAngle || currentAngle >= upperBound){
         double coTermAngle = Utils.coTerminal(targetAngle - currentAngle);
         return (coTermAngle <= 0) ? 0.3 : -0.3;
      }
      else return 0;
   }
}