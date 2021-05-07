package org.firstinspires.ftc.teamcode.Diagnostics;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID.RingBuffer;
import org.firstinspires.ftc.teamcode.Vision.AimBotPipe;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.RB2;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.TOUCHPAD;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Vision.VisionUtils.Target.GOAL;


@TeleOp(name = "WingTest TeleOp", group="Linear TeleOp")
public class WingTest extends LinearOpMode {

    // Main Stuff
    private Mecanum robot;
    private ButtonControls BC1, BC2;
    private JoystickControls JC1;

    // Camera stuff
    private AimBotPipe goalFinder = new AimBotPipe();
    private VisionUtils.Target aimTarget = GOAL;
    private VisionUtils.PowerShot powerShot = VisionUtils.PowerShot.PS_RIGHT;


    // PID Stuff
    private double  locked_direction;
    private boolean pid_on = false;
    private boolean last_cycle_pid_state = true;

    // Time Variables
    private double last_nanoseconds = 0.0;
    private double current_nanoseconds = 0.0;
    private ElapsedTime elapsedTime;

    // Hardware Positions
    private double current_angle = 0.0;
    private double last_angle = 0.0;

    private double current_intake_position = 0.0;
    private double last_intake_position = 0.0;

    private ElapsedTime intake_time = new ElapsedTime();

    private RingBuffer<Double> angle_ring_buffer = new RingBuffer<>(5, 0.0);
    private RingBuffer<Double> time_ring_buffer = new RingBuffer<>(5,  0.0);

    private double feederCount = 3;

    public void initialize() {
        OpModeUtils.setOpMode(this);
        robot = new Mecanum();
        BC1 = new ButtonControls(gamepad1);
        BC2 = new ButtonControls(gamepad2);
        JC1 = new JoystickControls(gamepad1);
        startVision();

        multTelemetry.addLine("------USER 1----------------------------");
        multTelemetry.addData("Velocity Ranger", "[LB2]");
        multTelemetry.addData("Quick Turn", "[DPAD]");
        multTelemetry.addData("Power Shot", "[RB1 / RB2]");

        multTelemetry.addLine("");

        multTelemetry.addLine("------USER 2----------------------------");
        multTelemetry.addData("Claw", "[TRIANGLE]");
        multTelemetry.addData("Intake ON/OFF", "[RB1]");
        multTelemetry.addData("Intake Direction", "[LB1]");
        multTelemetry.addData("Bumper Toggle", "[DPAD DOWN]");
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

    public void startVision(){
        /*
        Set up camera, and pipeline
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        VisionUtils.webcam.setPipeline(goalFinder);
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
    }



    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        if (opModeIsActive()){
            elapsedTime = new ElapsedTime();
        }

        while (opModeIsActive()) {

            double velocity = 1;


             /*

             ----------- C A L C U L A T I O N S -----------

                                                          */

            // Calculate Angular Velocity
            current_nanoseconds = elapsedTime.nanoseconds();
            current_angle = robot.imu.getAngle();

            double delta_angle = current_angle - angle_ring_buffer.getValue(current_angle);
            double delta_time = current_nanoseconds - time_ring_buffer.getValue(current_nanoseconds);
            double current_angular_velocity = delta_angle / delta_time;




            last_nanoseconds = current_nanoseconds;
            last_angle       = current_angle;



            /*

             ----------- H A R D W A R E    F U N C T I O N A L I T Y -----------

                                                                               */

            ButtonControls.update();
            JoystickControls.update();


            //                  WINGS LOGIC

            boolean nearGoal = false;
            if (abs(180 - abs(robot.imu.getModAngle())) < 30){
                nearGoal = true;
            }

            // Override toggle
            if (!BC2.get(CROSS, TOGGLE)){

                // Ultimate condition to check if we can shoot
                if (feederCount > 0){

                    // NearGoal : wings out
                    if (nearGoal){
                        robot.wings.out();
                    }
                    else {
                        robot.wings.mid();
                    }
                }
                else {
                    robot.wings.mid();
                }

                if (BC2.get(CIRCLE, TAP)){
                    if (BC2.get(CIRCLE, TOGGLE)) feederCount = 0;
                    else feederCount += 1;
                }

                /*
                // If we are facing near goal
                if (nearGoal && feederCount > 0 && !BC2.get(CIRCLE, TOGGLE)) robot.wings.out();
                else robot.wings.mid();

                // If intake is on
                if (BC2.get(RB1, TOGGLE)) robot.wings.out();
                else {
                    // If shooter on
                    if (BC2.get(CIRCLE, TOGGLE)){
                        if (feederCount > 0) robot.wings.out();
                        else robot.wings.mid();
                    }
                    // If we just start shooter, set feeder count to 0
                    else if (BC2.get(CIRCLE, TAP) && !BC2.get(CIRCLE, TOGGLE)) feederCount = 0;
                }

                 */
            }
            else robot.wings.mid();


            // SHOOTER

            if (BC2.get(RB2, TAP)){
                feederCount += 1;
            }
            if (BC2.get(CIRCLE, TOGGLE)) {
                // Slow down the robot
                velocity = 0.5;
            }





            /*

             ----------- S T E E R I N G    F U N C T I O N A L I T Y -----------

                                                                                */

            //           ABSOLUTE CONTROL MODE          //
            JC1.setShifted(RIGHT, (robot.imu.getAngle() - 90) % 360);

            //              DRIVER VALUES               //
            double drive = JC1.get(RIGHT, INVERT_SHIFTED_Y);
            double strafe = JC1.get(RIGHT, SHIFTED_X);
            double turn = JC1.get(LEFT, X);

            //              VELOCITY RANGER             //
            if (BC1.get(LB2, DOWN)) velocity = Range.clip((1 - gamepad1.left_trigger), 0.5, 1);
            else if (BC1.get(RB2, DOWN)) velocity = Range.clip((1 - gamepad1.right_trigger), 0.2, 1);

            /*

            ----------- P I D -----------

                                       */

            if (JC1.get(LEFT, X) != 0) pid_on = false;
            else if (current_angular_velocity == 0.0) pid_on = true;

            if (pid_on && !last_cycle_pid_state) locked_direction = robot.imu.getAngle();
            else if (pid_on) turn = robot.rotationPID.update(locked_direction - current_angle) * -1;

            last_cycle_pid_state = pid_on;



            /*

            ----------- S E T    P O W E R -----------

                                                    */
            robot.setDrivePower(drive, strafe, turn, velocity);



            /*

             ----------- L O G G I N G -----------

                                                */

            multTelemetry.addLine("--DRIVER-------------------------------------");
            multTelemetry.addData("Angle", robot.imu.getAngle());
            multTelemetry.addData("Locked Angle", locked_direction);
            multTelemetry.addData("Mod Angle", robot.imu.getModAngle());
            multTelemetry.addData("NearGoal", (nearGoal) ? "YES" : "NO");


            multTelemetry.addLine("--HARDWARE-------------------------------------");
            multTelemetry.addData("Shooter", (BC2.get(CIRCLE, TOGGLE)) ? "ON" : "OFF");
            multTelemetry.addData("FeederCount", feederCount);
            multTelemetry.addData("Intake", (BC2.get(RB1, DOWN)) ? "ON" : "OFF");
            multTelemetry.addData("Override Toggle", (!BC2.get(CROSS, TOGGLE)) ? "ON" : "OFF");
            multTelemetry.addData("Wing Status", robot.wings.getStatus());


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


