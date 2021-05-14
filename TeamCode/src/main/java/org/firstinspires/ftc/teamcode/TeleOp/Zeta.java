package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Navigation.Oracle;
import org.firstinspires.ftc.teamcode.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.StrictMath.round;
import static org.firstinspires.ftc.teamcode.DashConstants.Dash_AimBot.DEBUG_MODE_ON;
import static org.firstinspires.ftc.teamcode.DashConstants.Deprecated.Dash_Intake.INTAKE_RPM_FORWARDS;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_DN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_L;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_R;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.RB2;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.SQUARE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.TOUCHPAD;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.getAngle;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.getAngularVelocity;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.getXVelocity;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setAVelocity;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setAngle;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setIntakePosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setUpdateTask;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setXPosition;
import static org.firstinspires.ftc.teamcode.Navigation.Oracle.setYPosition;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.closestAngle;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PowerShot;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PowerShot.PS_LEFT;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PowerShot.PS_MIDDLE;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.PowerShot.PS_RIGHT;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.Target;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.Target.GOAL;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.Target.OUT_OF_RANGE;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.Target.POWERSHOTS;


@TeleOp(name = "Zeta TeleOp", group="Linear TeleOp")
public class Zeta extends LinearOpMode {

    // Main Stuff
    private Mecanum robot;
    private ButtonControls BC1, BC2;
    private JoystickControls JC1;

    // Camera stuff
    private AimBotPipe aimBot = new AimBotPipe();
    private Target aimTarget = GOAL;
    private PowerShot powerShot = PS_RIGHT;

    // PID Stuff
    private double PID_ANGLE;
    private boolean PID_ON = false;
    private boolean last_cycle_pid_state = true;


    public void initialize() {
        setOpMode(this);
        robot = new Mecanum();
        BC1 = new ButtonControls(gamepad1);
        BC2 = new ButtonControls(gamepad2);
        JC1 = new JoystickControls(gamepad1);

        // Initialize Oracle
        setUpdateTask(() -> {
            setAngle(robot.imu.getAngle());
            setXPosition(robot.getXComp());
            setYPosition(robot.getYComp());
            setIntakePosition(robot.intake.getIntakePosition());
        });

        setUpVision();


        multTelemetry.addLine("------USER 1----------------------------");
        multTelemetry.addData("Velocity Ranger", "[LB2]");
        multTelemetry.addData("Quick Turn", "[DPAD]");
        multTelemetry.addData("Power Shot", "[RB1 / RB2]");
        multTelemetry.addData("Wings", "[CROSS]");

        multTelemetry.addLine("");

        multTelemetry.addLine("------USER 2----------------------------");
        multTelemetry.addData("Claw", "[TRIANGLE]");
        multTelemetry.addData("Intake ON/OFF", "[RB1]");
        multTelemetry.addData("Intake Reverse", "[LB1]");
        multTelemetry.addData("Roller Toggle", "[DPAD DOWN]");
        multTelemetry.addData("Arm Out", "[DPAD RIGHT]");
        multTelemetry.addData("Arm Up", "[DPAD UP]");
        multTelemetry.addData("Arm In", "[DPAD LEFT]");
        multTelemetry.addData("Shooter ON/OFF", "[CIRCLE]");
        multTelemetry.addData("Shoot", "[RB2]");
        multTelemetry.addData("Switch Target", "[SQUARE]");


        multTelemetry.addData("Shutdown Keys", "[TOUCHPAD] simultaneously");
        multTelemetry.update();

    }

    public void shutdown(){
        multTelemetry.addData("Status", "Shutting Down");
        multTelemetry.update();
        robot.intake.shutdown();
        robot.arm.up();
        robot.wings.up();
    }

    public void setUpVision(){
        /*
        Set up camera, and pipeline
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        VisionUtils.webcam_front.setPipeline(aimBot);
        VisionUtils.webcam_front.openCameraDeviceAsync(() -> VisionUtils.webcam_front.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
    }



    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {

            double VELOCITY = 1;


            /*
             ----------- H A R D W A R E    F U N C T I O N A L I T Y -----------
                                                                               */

            ButtonControls.update();
            JoystickControls.update();
            Oracle.update();


            //                  ARM                         //
            if (BC2.get(DPAD_R, DOWN)) robot.arm.out();
            else if (BC2.get(DPAD_L, DOWN)) robot.arm.in();
            else if (BC2.get(DPAD_UP, TAP)) {
                if (robot.arm.getStatus() == Arm.STATUS.UP) robot.arm.up2();
                else robot.arm.up();
            }

            //                  CLAW                        //
            if (BC2.get(TRIANGLE, TOGGLE)) robot.claw.close();
            else robot.claw.open();


            //                INTAKE CODE                   //
            if (BC2.get(RB1, TOGGLE)) {
                if (BC2.get(LB1, DOWN)) robot.intake.setIntakePower(-1);
                else robot.intake.setRPM(INTAKE_RPM_FORWARDS);
            }
            else robot.intake.setIntakePower(0);


            //                  INTAKE ARM                  //
            if (BC2.get(DPAD_DN, TOGGLE)) robot.intake.rollerUp();
            else robot.intake.rollerMid();


            //                  WINGS LOGIC                 //
            if (!BC1.get(CROSS, TOGGLE)) robot.wings.mid();
            else robot.wings.out();


            // Square toggles aiming at goal or powershots
            if (BC2.get(SQUARE, TOGGLE))    aimTarget = POWERSHOTS;
            else                            aimTarget = GOAL;

            // Get degree error and correct
            double errorToGoal     = (abs(robot.imu.getModAngle()) - 180);
            double goalAngle       = aimBot.getGoalAngle(getAngle(), getXVelocity());
            double powerShotAngle  = aimBot.getPowerShotAngle(powerShot, getAngle());

            //              SHOOTER                 //
            robot.shooter.feederState(BC2.get(RB2, DOWN));
            if (BC2.get(CIRCLE, TOGGLE)) {

                // Move arm out of way of shooter
                robot.intake.rollerMid();

                // Fix Steering controls
                setAVelocity(0);

                // Check if Goal is found, if not, set RPM to default, and orient nearby goal
                double RPM = 3400;
                if (errorToGoal > 40) aimTarget = OUT_OF_RANGE;

                switch (aimTarget){
                    case GOAL:
                        PID_ANGLE = goalAngle;
                        RPM = aimBot.calcRPM();
                        break;

                    case POWERSHOTS:
                        PID_ANGLE = powerShotAngle;
                        RPM = aimBot.calcRPM() - 300;
                        break;

                    case OUT_OF_RANGE:
                        PID_ANGLE = closestAngle(180, getAngle());
                        break;
                }

                // Set the RPM
                robot.shooter.setRPM(RPM);
            }
            else robot.shooter.setPower(0);





            /*

             ----------- S T E E R I N G    F U N C T I O N A L I T Y -----------

                                                                                */

            //           ABSOLUTE CONTROL MODE          //
            JC1.setShifted(RIGHT, (getAngle() - 90) % 360);
            /*
            if (BC1.get(SQUARE, TAP) && !square_pressed) {
                robot.imu.setOffsetAngle(-((robot.imu.getAngle() + 180) % 360));
                robot.imu.resetDeltaAngle();
                square_pressed = true;
            }
             */

            //              DRIVER VALUES               //
            double drive    = JC1.get(RIGHT, INVERT_SHIFTED_Y);
            double strafe   = JC1.get(RIGHT, SHIFTED_X);
            double turn     = JC1.get(LEFT, X);

            //              VELOCITY RANGER             //
            if (BC1.get(LB2, DOWN))             VELOCITY = clip((1 - gamepad1.left_trigger), 0.5, 1);
            else if (BC1.get(RB2, DOWN))        VELOCITY = clip((1 - gamepad1.right_trigger), 0.2, 1);

            //              DPAD AUTO TURN              //
            if (BC1.get(DPAD_UP, DOWN))         PID_ANGLE = closestAngle(90,    getAngle());
            else if (BC1.get(DPAD_R, DOWN))     PID_ANGLE = closestAngle(0,     getAngle());
            else if (BC1.get(DPAD_L, DOWN))     PID_ANGLE = closestAngle(180,   getAngle());
            else if (BC1.get(DPAD_DN, DOWN))    PID_ANGLE = closestAngle(270,   getAngle());

            //            POWER SHOT INCREMENT          //
            if (BC1.get(SQUARE, DOWN))          powerShot = PS_LEFT;
            else if (BC1.get(TRIANGLE, DOWN))   powerShot = PS_MIDDLE;
            else if (BC1.get(CIRCLE, DOWN))     powerShot = PS_RIGHT;

            powerShotAngle = aimBot.getPowerShotAngle(powerShot, getAngle());
            if (BC1.get(RB1, TAP) || BC1.get(LB1, TAP)) PID_ANGLE = powerShotAngle;



            /*
            ----------- P I D -----------
                                       */

            // Turn off PID if finished manually turning
            if (JC1.get(LEFT, X) != 0) PID_ON = false;
            else if (getAngularVelocity() == 0.0) PID_ON = true;

            // Setting the PID_ANGLE
            if (PID_ON && !last_cycle_pid_state) PID_ANGLE = getAngle();
            else if (PID_ON) turn = robot.rotationPID.update(PID_ANGLE - getAngle()) * -1;

            last_cycle_pid_state = PID_ON;



            /*
            ----------- S E T    P O W E R -----------
                                                    */

            // Turn Powers based on shooter, and PID_ANGLE states
            if (BC2.get(CIRCLE, TOGGLE))            turn *= 0.5;
            if ((round(abs(PID_ANGLE)) % 90) == 0)  turn *= 0.85;

            robot.setDrivePower(drive, strafe, turn, VELOCITY);



            /*
             ----------- L O G G I N G -----------
                                                */
            if (DEBUG_MODE_ON){
                multTelemetry.addLine("--DEBUG-------------------------------------");
                multTelemetry.addData("Goal Distance",          aimBot.getGoalDistance());
                multTelemetry.addData("Goal Found",             aimBot.isGoalFound());
                multTelemetry.addData("Goal Error",             aimBot.getGoalDegreeError(getAngle(), getXVelocity()));
                multTelemetry.addData("Goal Angle",             aimBot.getGoalAngle(getAngle(), getXVelocity()));
                multTelemetry.addData("Goal Position Offset",   aimBot.calcGoalOffset(getAngle()));
                multTelemetry.addData("Goal X-Velocity Offset", aimBot.calcGoalXVelocityOffset(getXVelocity()));
            }

            multTelemetry.addLine("--DRIVER-------------------------------------");
            multTelemetry.addData("Target", (aimTarget == POWERSHOTS) ? powerShot : aimTarget);
            multTelemetry.addData("Angle", getAngle());
            multTelemetry.addData("PID ANGLE", PID_ANGLE);


            multTelemetry.addLine("--HARDWARE-------------------------------------");
            multTelemetry.addData("Intake Forward", (!BC2.get(LB1, TOGGLE)) ? "FORWARD" : "REVERSE");
            multTelemetry.addData("Shooter", (BC2.get(CIRCLE, TOGGLE)) ? "ON" : "OFF");
            multTelemetry.addData("RPM", robot.shooter.getRPM());

            multTelemetry.update();




            /*
             ----------- S H U T D O W N -----------
                                                  */

            if (BC1.get(TOUCHPAD, DOWN) || BC2.get(TOUCHPAD, DOWN)){
                shutdown();
                break;
            }
        }
    }
}


