package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Encoder {


    private DcMotor encoder;
    private double startPosition;

    public Encoder(String id){

        encoder = Utils.hardwareMap.get(DcMotor.class, id);
        startPosition = encoder.getCurrentPosition();
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
