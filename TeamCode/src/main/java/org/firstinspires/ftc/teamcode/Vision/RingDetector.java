package org.firstinspires.ftc.teamcode.Vision;

// Vision Hardware
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class RingDetector {

   private Vuforia vuforia;
   private WebcamName webcam;
   private double[] orangeRGB;

   public RingDetector(){
      orangeRGB = new double[]{250.0, 175.0, 60.0};
   }

   public int getRingConfig(){

      /*double[] currentRGB = mecanumRobot.colorSensor.red();
      double colorMargin = 10;
      if (Utils.distance2Color(orangeRGB, currentRGB) < colorMargin){
         telemetry.addData("Detecting Ring", true);
      }
      else telemetry.addData("Detecting Ring", false);
      */

     return 0;
   }

}
