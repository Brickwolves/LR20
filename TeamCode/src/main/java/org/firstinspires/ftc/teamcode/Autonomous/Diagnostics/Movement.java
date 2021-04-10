package org.firstinspires.ftc.teamcode.Autonomous.Diagnostics;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.diag_deg;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Movement.diag_inches;

@Autonomous(name="AutoDiag", group="Autonomous Linear Opmode")
public class Movement extends LinearOpMode {

    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime time = new ElapsedTime();

    public void BREAKPOINT(){
        while (true && opModeIsActive()){
            OpModeUtils.multTelemetry.addData("Status", "Holding");
            OpModeUtils.multTelemetry.addData("Turn", robot.getTurn());
            OpModeUtils.multTelemetry.update();
            if (BC.get(CROSS, DOWN)) break;
        }
        OpModeUtils.multTelemetry.addData("Status", "Continuing");
        OpModeUtils.multTelemetry.update();
    }

    public void initialize(){
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();

        OpModeUtils.multTelemetry.addLine("Waiting for start");
        OpModeUtils.multTelemetry.update();
        waitForStart();

        B();

        while (!BC.get(CROSS, DOWN) && opModeIsActive()) {
            OpModeUtils.multTelemetry.addLine("BREAKPOINT");
            OpModeUtils.multTelemetry.addData("Angle", robot.imu.getAngle());
            OpModeUtils.multTelemetry.update();
        }
    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    public void B(){
        robot.strafe(diag_deg, diag_inches, 90, 0.5, null);
    }

}
