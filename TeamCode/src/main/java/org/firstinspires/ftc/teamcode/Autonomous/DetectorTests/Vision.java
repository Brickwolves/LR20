package org.firstinspires.ftc.teamcode.Autonomous.DetectorTests;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.SanicPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

@Disabled
@Autonomous(name="Vision", group="Autonomous Linear Opmode")
public class Vision extends LinearOpMode
{

    public void initialize(){
        setOpMode(this);
        setUpWebcam();
    }

    public void setUpWebcam(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam2"), cameraMonitorViewId);
        VisionUtils.webcam_front.setPipeline(new SanicPipe());
        VisionUtils.webcam_front.openCameraDeviceAsync(() -> VisionUtils.webcam_front.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode()
    {

        initialize();

        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();


        /*
        ACTION
         */
        if (opModeIsActive()){

            multTelemetry.update();
            VisionUtils.webcam_front.stopStreaming();
        }
    }

    class Pipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input)
        {
            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;
            if (viewportPaused) VisionUtils.webcam_front.pauseViewport();
            else VisionUtils.webcam_front.resumeViewport();
        }
    }
}
