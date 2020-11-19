package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.hardware.ams.AMSColorSensorImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Had to create this wrapper class because I can't simply extend every possible implementation such as BroadCom, Lynx, or AMS.
 * That's why I rewrote it to take in any possible implementation.
 * This class also provides the necessary transformations to the RGB values
 */
public class ColorSensorImpl {

    ColorSensor sensor;
    public ColorSensorImpl(ColorSensor sensor){
       this.sensor = sensor;
    }

    public double[] getRGB(){
        return new double[] {this.red(), this.green(), this.blue()};
    }

    public int red()
    {
        return sensor.red() * 256/8192;
    }
    public int green()
    {
        return sensor.green() * 256/8192;
    }
    public int blue()
    {
        return sensor.blue() * 256/8192;
    }

    public int alpha(){
        return sensor.alpha();
    }

    public int argb(){
        return sensor.argb();
    }

    public void enableLed(boolean enable){
        sensor.enableLed(enable);
    }

    public void setI2cAddress(I2cAddr newAddress){
        sensor.setI2cAddress(newAddress);
    }

    public I2cAddr getI2cAddress(){
        return sensor.getI2cAddress();
    }

}
