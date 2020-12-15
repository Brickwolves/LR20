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
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Vision.ObjectDetector;


@Autonomous(name="AlphaAuto", group="Autonomous Linear Opmode")
//@Disabled
public class AlphaAuto extends LinearOpMode {

    private MecanumRobot mecanumRobot;
    private ObjectDetector od;

    public void initialize() {
        Utils.setHardwareMap(hardwareMap);
        Utils.setTelemetry(telemetry);

        mecanumRobot = new MecanumRobot();
        od = new ObjectDetector();
    }


    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
        telemetry.addData("started", true);


        //  Ring identifier
        int rings = od.getRingConfig();
        for (int i = 0; i < rings; i++) {
            turn(360, 0.5);
        }



        // Shutdown robot
        od.shutdown();
    }


    public void turn(MecanumRobot.Direction direction, double MOE) {
        System.out.println("Turning to " + direction + " degrees");

        double targetAngle = 0;
        switch (direction) {
            case NORTH:
                targetAngle = 0;
                break;
            case WEST:
                targetAngle = 90;
                break;
            case EAST:
                targetAngle = -90;
                break;
            case SOUTH:
                targetAngle = 180;
                break;
        }
        targetAngle += mecanumRobot.imu.getDeltaAngle();
        double currentAngle = mecanumRobot.imu.getAngle();
        double upperBound = targetAngle + MOE;
        double lowerBound = targetAngle - MOE;
        while ((lowerBound >= currentAngle || currentAngle >= upperBound) && opModeIsActive()){
            double coTermAngle = Utils.coTerminal(targetAngle - currentAngle);
            double turn = (coTermAngle <= 0) ? 1 : -1;
            mecanumRobot.setDrivePower(0, 0, turn, 0.3);
        }
        telemetry.addData("TargetAngle", targetAngle);
    }


    /**
     * @param targetAngle
     * @param MOE
     */
    public void turn(double targetAngle, double MOE) {
        System.out.println("Turning to " + targetAngle + " degrees");

        double currentAngle = mecanumRobot.imu.getAngle();
        double deltaAngle = Math.abs(targetAngle - currentAngle);
        double power;
        double position = mecanumRobot.getPosition();


        // Retrieve angle and MOE
        double upperBound = targetAngle + MOE;
        double lowerBound = targetAngle - MOE;
        while ((lowerBound >= currentAngle || currentAngle >= upperBound) && opModeIsActive()) {

            // Power Ramping based off a logistic piecewise
            double currentDeltaAngle = targetAngle - currentAngle;
            double anglePosition = deltaAngle - currentDeltaAngle + 0.01; // Added the 0.01 so that it doesn't get stuck at 0


            // Modeling a piece wise of power as a function of distance
            power = Utils.powerRamp(anglePosition, deltaAngle, 0.1);

            // Handle clockwise (+) and counterclockwise (-) motion
            mecanumRobot.setDrivePower(0, 0, 1, power);

            currentAngle = mecanumRobot.imu.getAngle();
        }

        // Stop power
        mecanumRobot.setAllPower(0);
    }



}