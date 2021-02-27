package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.Gamepad;

import static android.os.SystemClock.sleep;

public class Controller {

    /*
    --------------- S I N G L E   P R E S S E S -------------
     */

    public boolean LB1_tap                      = false;
    public boolean RB1_tap                      = false;
    public boolean LB2_tap                      = false;
    public boolean RB2_tap                      = false;

    public boolean triangle_tap                 = false;
    public boolean square_tap                   = false;
    public boolean circle_tap                   = false;
    public boolean cross_tap                    = false;

    public boolean right_stick_btn_tap          = false;
    public boolean left_stick_btn_tap           = false;


    /*
    --------------- T O G G L E S -------------
     */
    public boolean LB1_toggle                   = false;
    public boolean RB1_toggle                   = false;
    public boolean LB2_toggle                   = false;
    public boolean RB2_toggle                   = false;

    public boolean triangle_toggle              = false;
    public boolean square_toggle                = true;
    public boolean circle_toggle                = false;
    public boolean cross_toggle                 = false;

    public boolean right_stick_btn_toggle       = false;
    public boolean left_stick_btn_toggle        = false;


    /*
    --------------- D E A D Z O N E S -------------
     */
    double right_trigger_deadzone               = 0.75;
    double left_trigger_deadzone                = 0.75;
    double right_stick_deadzone                 = 0.3;
    double left_stick_deadzone                  = 0.3;
    double default_deadzone                     = 0.3;

    private int buttonWaitSeconds               = 200;


    // Default Vars
    public Gamepad src;
    public Controller(Gamepad src){
        this.src = src;
    }


    public Thumbstick getRightThumbstick() {
        return new Thumbstick(src.right_stick_x, src.right_stick_y);
    }

    public Thumbstick getLeftThumbstick() {
        return new Thumbstick(src.left_stick_x, src.left_stick_y);
    }

    public class Thumbstick {

        private double rawX;
        private double rawY;
        private double shiftedX;
        private double shiftedY;

        public Thumbstick(Double x, Double y) {
            this.rawX = x;
            this.rawY = y;
        }

        public Thumbstick(Float x, Float y) {
            this.rawX = x;
            this.rawY = y;
        }

        public double getX() {
            return rawX;
        }

        public double getY() {
            return rawY;
        }

        public void setShift(double shiftAngle) {
            this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
            this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedX() {
            return shiftedX;
        }

        public double getShiftedY() {
            return shiftedY;
        }

        public double getShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getInvertedX() {
            return rawX * -1;
        }

        public double getInvertedY() {
            return rawY * -1;
        }

        public double getInvertedShiftedX() {
            return shiftedX * -1;
        }

        public double getInvertedShiftedY() {
            return shiftedY * -1;
        }

        public double getInvertedShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
        }

        public double getInvertedShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
        }
    }
    public boolean DPADPress() {return src.dpad_down || src.dpad_left || src.dpad_right || src.dpad_up;}


    public void updateToggles(){

        // Check if buttons were tapped
        RB1_tap = buttonTapped(src.right_bumper, RB1_tap);
        LB1_tap = buttonTapped(src.left_bumper, LB1_tap);
        RB2_tap = buttonTapped(src.right_trigger, RB2_tap, right_stick_deadzone);
        LB2_tap = buttonTapped(src.left_trigger, LB2_tap, left_stick_deadzone);

        triangle_tap = buttonTapped(src.triangle, triangle_tap);
        cross_tap = buttonTapped(src.cross, cross_tap);
        square_tap = buttonTapped(src.square, square_tap);
        circle_tap = buttonTapped(src.circle, circle_tap);

        right_stick_btn_tap = buttonTapped(src.right_stick_button, right_stick_btn_tap);
        left_stick_btn_tap = buttonTapped(src.left_stick_button, left_stick_btn_tap);


        // Update toggles accordingly
        if (RB1_tap) {
            RB1_toggle = !RB1_toggle;
            sleep(buttonWaitSeconds);
        }
        if (LB1_tap) {
            LB1_toggle = !LB1_toggle;
            sleep(buttonWaitSeconds);
        }
        if (RB2_tap) {
            RB2_toggle = !RB2_toggle;
            sleep(buttonWaitSeconds);
        }
        if (LB2_tap) {
            LB2_toggle = !LB2_toggle;
            sleep(buttonWaitSeconds);
        }


        if (triangle_tap) {
            triangle_toggle = !triangle_toggle;
            sleep(buttonWaitSeconds);
        }
        if (square_tap) {
            square_toggle = !square_toggle;
            sleep(buttonWaitSeconds);
        }
        if (circle_tap) {
            circle_toggle = !circle_toggle;
            sleep(buttonWaitSeconds);
        }
        if (cross_tap) {
            cross_toggle = !cross_toggle;
            sleep(buttonWaitSeconds);
        }


        if (right_stick_btn_tap) {
            right_stick_btn_toggle = !right_stick_btn_toggle;
            sleep(buttonWaitSeconds);
        }
        if (left_stick_btn_tap) {
            left_stick_btn_toggle = !left_stick_btn_toggle;
            sleep(buttonWaitSeconds);
        }

    }


    /**
     * Super simple method to check toggles on buttons
     * @param current
     * @param previous
     * @return
     */
    public Boolean buttonTapped(boolean current, boolean previous){
        if (current && !previous )return true;
        else if (!current) return false;
        else return previous;
    }

    public Boolean buttonTapped(float current_float, boolean previous, double deadzone){
        boolean current = false;
        double current_double = current_float;
        if (current_double > deadzone) current = true;
        if (current && !previous )return true;
        else if (!current) return false;
        else return previous;
    }
}
