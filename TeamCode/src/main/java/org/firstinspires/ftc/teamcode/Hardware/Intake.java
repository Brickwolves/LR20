package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class Intake {

    private DcMotor motor;
    private double currentPower;
    private String DcMotor_id;

    public Intake(String DcMotor_id){

        try {
            this.DcMotor_id = DcMotor_id;
            motor = Utils.hardwareMap.get(DcMotor.class, DcMotor_id);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
        catch (Exception e){
            throw new Error("Cannot find motor with id: " + DcMotor_id + "\n. Could not initialize motor.");
        }

    }

    public void setPower(double power){
        currentPower = power;
    }
    public void update(){
        try {
            motor.setPower(currentPower);
        }
        catch (Exception e){
            //throw new Error("Cannot find motor with id: " + DcMotor_id + "\n. Could not initialize motor.");
            Utils.telemetry.addData("Status", "Could not initialize motors");
            Utils.telemetry.update();
        }
    }
}
