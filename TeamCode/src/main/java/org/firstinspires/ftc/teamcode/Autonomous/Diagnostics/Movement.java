package org.firstinspires.ftc.teamcode.Autonomous.Diagnostics;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement.diag_deg;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement.diag_inches;

@Autonomous(name="AutoDiag", group="Autonomous Linear Opmode")
public class Movement extends LinearOpMode {

    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime time = new ElapsedTime();

    public void BREAKPOINT(){
        while (true && opModeIsActive()){
            Utils.multTelemetry.addData("Status", "Holding");
            Utils.multTelemetry.addData("Turn", robot.getTurn());
            Utils.multTelemetry.update();
            if (BC.get(CROSS, DOWN)) break;
        }
        Utils.multTelemetry.addData("Status", "Continuing");
        Utils.multTelemetry.update();
    }

    public void initialize(){
        Utils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();

        Utils.multTelemetry.addLine("Waiting for start");
        Utils.multTelemetry.update();
        waitForStart();

        B();

        while (!BC.get(CROSS, DOWN) && opModeIsActive()) {
            Utils.multTelemetry.addLine("BREAKPOINT");
            Utils.multTelemetry.addData("Angle", robot.imu.getAngle());
            Utils.multTelemetry.update();
        }
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    public void B(){
        robot.strafe(diag_deg, diag_inches, 90, 0.5, null);
    }

}