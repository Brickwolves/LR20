package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utilities.IMU;
import org.firstinspires.ftc.teamcode.Utilities.Utils;


@TeleOp(name = "Mecanum Drive", group="TeleOp")
//@Disabled
public class MecanumTeleOp extends OpMode {


    private DcMotor fr, fl, br, bl;
    private IMU imu;
    private TouchSensor touch;
    private ColorRangeSensor color;

    private double[] orangeRGB;
    private double startHeading;
    private boolean DPAD_Toggle;
    private boolean RB_Toggle;
    private boolean RBLastCycle;
    private boolean velocityToggle;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");
        touch = hardwareMap.get(TouchSensor.class, "touch_sensor");
        color = hardwareMap.get(ColorRangeSensor.class, "color_sensor");

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        // Init IMU
        Utils.setHardwareMap(hardwareMap);
        imu = new IMU("imu");

        orangeRGB = new double[]{250.0, 175.0, 60.0};

        startHeading = imu.getAngle();
        DPAD_Toggle = false;
        RB_Toggle = false;
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {

    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // Gamepad Values
        double strafe = gamepad1.right_stick_x;
        double drive = gamepad1.right_stick_y * -1;
        double turn = gamepad1.left_stick_x;
        double velocity = 1;



        // Gamepad Toggles
        if (gamepad1.right_bumper && !RBLastCycle ){
            velocityToggle = !velocityToggle;
            RBLastCycle = true;
        }
        else if (!gamepad1.right_bumper) RBLastCycle = false;
        if (velocityToggle) velocity = 0.5;


        // Auto Turn
        if (gamepad1.dpad_up) turn = turn2(0, 0.5);
        else if (gamepad1.dpad_right) turn = turn2(-90, 0.5);
        else if (gamepad1.dpad_left) turn = turn2(90, 0.5);
        else if (gamepad1.dpad_down) turn = turn2(180, 0.5);


        //touch sensor concept
       if(touch.isPressed()){
           turn = 5;
       }
       //color sensor concept



       double[] currentRGB = {color.red(), color.green(), color.blue()};
       double colorMargin = 10;
       if (Utils.distance2Color(orangeRGB, currentRGB) < colorMargin){
            telemetry.addData("Detecting Ring", true);
       }
       else telemetry.addData("Detecting Ring", false);





        // Setting driver power
        fr.setPower((drive - strafe - turn) * velocity);
        fl.setPower((drive + strafe + turn) * velocity);
        br.setPower((drive + strafe - turn) * velocity);
        bl.setPower((drive - strafe + turn) * velocity);



        // Telemetry
        telemetry.addData("Velocity Toggle", velocityToggle);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Drive", drive);
        telemetry.addData("Turn", turn);
        telemetry.addData("IMU", imu.getAngle());
        telemetry.addData("Error", startHeading - imu.getAngle());
        /*
        telemetry.addData("Fl", fl.getCurrentPosition());
        telemetry.addData("FR", fr.getCurrentPosition());
        telemetry.addData("BL", bl.getCurrentPosition());
        telemetry.addData("BR", br.getCurrentPosition());
        telemetry.addData("Position", (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0);
        */
        telemetry.addData("touch", touch.isPressed());
        telemetry.addData("red", color.red() * 256/8192);
        telemetry.addData("green", color.green() * 256/8192);
        telemetry.addData("blue", color.blue() * 256/8192);
        //telemetry.addData("colorType", color.getClass().getName());



    }



    /**
     * @param targetAngle
     * @param MOE
     */
    public double turn2(double targetAngle, double MOE) {

        System.out.println("Turning to " + targetAngle + " degrees");

        targetAngle += imu.getDeltaAngle();
        double currentAngle = imu.getAngle();
        double upperBound = targetAngle + MOE;
        double lowerBound = targetAngle - MOE;
        if (lowerBound >= currentAngle || currentAngle >= upperBound){
            double coTermAngle = coTerminal(targetAngle - currentAngle);
            return (coTermAngle <= 0) ? 0.3 : -0.3;
        }
        else return 0;
    }
    /**
     * @param angle
     * @return coTermAngle
     */
   public double coTerminal(double angle){

        double coTermAngle = (angle + 180) % 360;
        coTermAngle -= 180;
        return coTermAngle;
   }



}


