package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ControllerCollin;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;

@Disabled
@Autonomous(name="AutoDiag", group="Autonomous Linear Opmode")
public class AutoDiag extends LinearOpMode {

    private MecanumRobot robot;
    private ButtonControls BC;
    private ElapsedTime time = new ElapsedTime();

    public void breakpoint(){
        while (true){
            Utils.multTelemetry.addData("Status", "Holding");
            Utils.multTelemetry.update();
            if (BC.get(CROSS, DOWN)) break;
        }
        Utils.multTelemetry.addData("Status", "Continuing");
        Utils.multTelemetry.update();
    }

    public void initialize(){
        Utils.setOpMode(this);
        robot = new MecanumRobot();
        BC = new ButtonControls(gamepad1);
    }

    public void shoot(int rings){
        time.reset();
        while (true) {
            robot.shooter.setRPM(3500);

            if (time.seconds() > 2) {
                if (robot.shooter.feederCount() < rings) robot.shooter.feederState(true);
                else break;
            }

            Utils.multTelemetry.addData("Position", robot.shooter.getPosition());
            Utils.multTelemetry.addData("RPM", robot.shooter.getRPM());

            Utils.multTelemetry.update();
        }
        robot.shooter.setPower(0);
        robot.shooter.setFeederCount(0);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();

        Utils.multTelemetry.addLine("Waiting for start");
        Utils.multTelemetry.update();
        waitForStart();

        shoot(4);
    }
}
