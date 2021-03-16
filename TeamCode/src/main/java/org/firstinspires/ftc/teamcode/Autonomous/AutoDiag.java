package org.firstinspires.ftc.teamcode.Autonomous;

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

@Autonomous(name="AutoDiag", group="Autonomous Linear Opmode")
public class AutoDiag extends LinearOpMode {

    private Mecanum robot;
    private ButtonControls BC;
    private ElapsedTime time = new ElapsedTime();

    public void BREAKPOINT(){
        while (true){
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

        robot.turn(0, 0.5);
        robot.turn(180, 0.5);
        robot.turn(-90, 0.5);
        robot.turn(0, 0.5);
        robot.turn(5, 0.5);


    }
}
