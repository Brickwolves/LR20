package org.firstinspires.ftc.teamcode.Vision;

// Vision Hardware
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class RingDetector {

   private Vuforia vuforia;
   private ObjectDetector od;

   public enum Config {
      NONE,
      SINGLE,
      QUAD
   }

   public RingDetector(){
      vuforia = new Vuforia();
      ObjectDetector od = new ObjectDetector();
   }

   public Config getRingConfig(){

      Recognition recog = od.getPrimaryObject();
      if (recog == null) throw new Error("No object detected");

      switch (recog.getLabel()){

         case "Single":
            return Config.SINGLE;
         case "Quad":
             return Config.QUAD;
      }
     return Config.NONE;
   }
}
