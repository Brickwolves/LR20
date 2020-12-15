package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AbuxcJX/////AAABmXadAYnA80uwmb4Rhy4YmvIh7qg/f2yrRu1Nd8O7sSufbUWHSv1jDhunwDBItvFchrvkc8EjTzjh97m2kAPy8YOjBclQbEBtuR8qcIfrGofASCZh2M6vQ0/Au+YbhYh0MLLdNrond+3YjkLswv6+Se3eVGw9y9fPGamiABzIrosjUdanAOWemf8BtuQUW7EqXa4mNPtQ+2jpZQO2sqtqxGu1anHQCD0S/PvdZdB7dRkyWaH6XTZCat5gZ0fpFH/aLWMFP4yiknlgYbjT7gklUAqyDX81pNrQhWWY4dOFnz2WiWhkCt+MNZMLKH5SdsyC7gwKI/r3h51pTwgXZfyYymB60eYAFqEUpeTrL+4LmltN";


    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId"," id", Utils.hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters();
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}
