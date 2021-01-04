package org.firstinspires.ftc.teamcode.Vision;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import java.util.List;

public class ObjectDetector {

    public TFObjectDetector tfod;
    private Vuforia vuforia = new Vuforia();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public ObjectDetector(){

        int tfodMonitorViewId = Utils.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", Utils.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia.getVuforiaLocalizer());
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void shutdown(){
        tfod.shutdown();
    }

    public Recognition getPrimaryObject(){

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            Utils.telemetry.addData("# Object Detected", updatedRecognitions.size());

            // step through the list of recognitions and display boundary info.
            // Retrieve recognition with max confidence
            // CHECK 0 OBJECTS, 0.8 min confidence
            int i = 0;
            int alphaIndex = 0;
            double alphaConfidence = -1.0;
            for (Recognition recognition : updatedRecognitions) {

                double confidence = recognition.getConfidence();
                if (confidence > alphaConfidence) alphaIndex = i;

                /*
                Utils.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                Utils.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                Utils.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                 */
            }
            if(updatedRecognitions.size() == 0){
                System.out.println("this works");
                Log.i("Testing ", "Testing ");
                return null;
            }
            return updatedRecognitions.get(alphaIndex);

        }

        return null;
    }

    public int getRingConfig(){

        Recognition recog = getPrimaryObject();
        if (recog == null) return 0;
        else if (recog.getLabel().equals("Single")) return 1;
        else if (recog.getLabel().equals("Quad")) return 4;
        throw new Error("Uh oh");
    }

}
