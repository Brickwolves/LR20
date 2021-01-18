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

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.Utils.telemetry;


@Autonomous(name="DiagnosticAuto", group="Autonomous Linear Opmode")
//@Disabled
public class DiagnosticAuto extends LinearOpMode {

    private MecanumRobot mecanumRobot;


    private double currentPosition, currentAngle;

    public void initialize(){
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
    }


    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        /*
        sleep(5000);
        mecanumRobot.strafe(0, DashConstants.diagnosticInches, 0, 0.05, null);
        sleep(5000);
        mecanumRobot.strafe(180, DashConstants.diagnosticInches, 0, 0.05, null);
         */


        Utils.multTelemetry.addData("Status", "Turning 45");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(45, 0.01);


        Utils.multTelemetry.addData("Status", "Turning 0");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(0, 0.01);

        /*---------------------------------------------------*/

        Utils.multTelemetry.addData("Status", "Turning 60");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(60, 0.01);


        Utils.multTelemetry.addData("Status", "Turning 0");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(0, 0.01);

        /*---------------------------------------------------*/

        Utils.multTelemetry.addData("Status", "Turning 90");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(90, 0.01);

        Utils.multTelemetry.addData("Status", "Turning 0");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(0, 0.01);

        /*---------------------------------------------------*/

        Utils.multTelemetry.addData("Status", "Turning 45");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(180, 0.01);


        Utils.multTelemetry.addData("Status", "Turning 0");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(0, 0.01);

        /*---------------------------------------------------*/

        Utils.multTelemetry.addData("Status", "Turning -360");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(-360, 0.01);

        Utils.multTelemetry.addData("Status", "Turning 0");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(0, 0.01);



        Utils.multTelemetry.addData("Status", "Turning -360");
        Utils.multTelemetry.addData("IMU", mecanumRobot.imu.getAngle());
        Utils.multTelemetry.update();
        sleep(2000);
        mecanumRobot.turnPID(-360, 0.01);


        Utils.multTelemetry.addData("Status", "Shutting Down");
        Utils.multTelemetry.update();
        sleep(2000);
    }
}
