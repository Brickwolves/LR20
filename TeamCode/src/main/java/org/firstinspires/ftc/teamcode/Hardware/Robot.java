package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Robot {

    /**
     * Initialize all components of robot
     */
    public void initRobot();

    /**
     * Initialize all motors
     */
    public void initMotors();

    /**
     * @return {double} Robot's current position
     */
    public double getPosition();

    /**
     * Set power to all motors
     */
    public void setAllPower(double power);

}
