package org.firstinspires.ftc.teamcode.Autonomous;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import org.firstinspires.ftc.teamcode.Utilities.SyncTask;

@Autonomous(name="ServoDiagnosticAuto", group="Autonomous Linear Opmode")
//@Disabled
public class ServoDiagnosticAuto extends LinearOpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;

    private double currentPosition, currentAngle;


    private Servo servo;


    public void initialize(){
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();

        servo = hardwareMap.get(Servo.class, "servoTest");
        servo.setDirection(Servo.Direction.FORWARD);

    }


    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
        telemetry.addData("started", true);


        double power = 1;
        double servoPosition;
        while (opModeIsActive()){
            servoPosition = servo.getPosition();
            if (servoPosition == 1 || servoPosition == -1) power *= -1;
            servo.setPosition(power);


            telemetry.addData("Servo Pos", servoPosition);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
    /**
     * @param angle
     * @param ticks
     */
    public void strafe(double angle, int ticks, double startAngle, SyncTask task){

        System.out.println(angle + " " + ticks);


        mecanumRobot.initMotors();                                              // Reset Motor Encoders

        double learning_rate = 0.000001;
        double radians = angle * Math.PI / 180;                     // Convert to radians
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


        currentPosition = mecanumRobot.getPosition();
        while (mecanumRobot.getPosition() < distance && opModeIsActive()){

            // Execute task synchronously
            if (task != null) task.execute();

            // Power ramping
            double power = Utils.powerRamp(currentPosition, distance, 0.075);

            // PID Controller
            double error = startAngle - mecanumRobot.imu.getAngle();
            turn += error * learning_rate;
            mecanumRobot.setDrivePower(drive * power, strafe * power, turn, 1);

            // Log and get new position
            currentPosition = mecanumRobot.getPosition();
            log();
        }
        mecanumRobot.setAllPower(0);
    }

    /**
     * Logs out Telemetry Data
     */
    public void log(){
        telemetry.addData("IMU", mecanumRobot.imu.getAngle());
        telemetry.addData("RGB", "(${mecanumRobot.colorSensor.red()}, ${mecanumRobot.colorSensor.green()}, ${mecanumRobot.colorSensor.blue()}");
        telemetry.addData("Error", mecanumRobot.imu.getStartAngle() - mecanumRobot.imu.getAngle());
        //telemetry.addData("touch", mecanumRobot.touchSensor.isPressed());
        telemetry.addData("webcam", mecanumRobot.webCam.getConnectionInfo());
    }
}
