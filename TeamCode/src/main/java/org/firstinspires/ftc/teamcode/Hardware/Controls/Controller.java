package org.firstinspires.ftc.teamcode.Hardware.Controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.Controller.Input.*;


/**
 * @author 99% of all code by Alex Appleby, Team 16896
 * @author Modifications by Jamie Gabbay, Team 18224
 */
public class Controller {
    private Map<Input, Button> buttons = new HashMap<>();

    private static List<Controller> instances = new ArrayList<>();

    public Gamepad src;

    public enum Input {
        TRIANGLE, SQUARE, X, CIRCLE,
        L_BUMPER, R_BUMPER,
        L_TRIGGER, R_TRIGGER,
        DPAD_UP, DPAD_DN, DPAD_L, DPAD_R,
        TOUCHPAD;
    }

    public Controller(Gamepad gamepad){
        src = gamepad;

        instances.add(this);

        buttons.put(TRIANGLE, new Button(() -> gamepad.a));
        buttons.put(SQUARE, new Button(() -> gamepad.b));
        buttons.put(X, new Button(() -> gamepad.x));
        buttons.put(CIRCLE, new Button(() -> gamepad.y));
        buttons.put(L_BUMPER, new Button(() -> gamepad.left_bumper));
        buttons.put(R_BUMPER, new Button(() -> gamepad.right_bumper));
        buttons.put(DPAD_UP, new Button(() -> gamepad.dpad_up));
        buttons.put(DPAD_DN, new Button(() -> gamepad.dpad_down));
        buttons.put(DPAD_L, new Button(() -> gamepad.dpad_left));
        buttons.put(DPAD_R, new Button(() -> gamepad.dpad_right));
        buttons.put(TOUCHPAD, new Button(() -> gamepad.touchpad));
        buttons.put(L_TRIGGER, new Button(() -> gamepad.left_trigger > 0.75));
        buttons.put(R_TRIGGER, new Button(() -> gamepad.right_trigger > 0.75));
    }

    public static void update(){
        for (Controller c : instances) {
            c.updateInstance();
        }
    }

    private void updateInstance(){
        for (Button b : buttons.values()) {
            b.update();
        }
    }

    public void setTapAction(Input input, Action action){
        buttons.get(input).tapAction = action;
    }
    public void setIdleAction(Input input, Action action){
        buttons.get(input).idleAction = action;
    }
    public void setOnAction(Input input, Action action){
        buttons.get(input).onAction = action;
    }
    public void setOffAction(Input input, Action action){
        buttons.get(input).offAction = action;
    }


    private static class EmptyAction implements Action {
        @Override
        public void execute() {}
    }

    public interface Action {
        void execute();
    }

    /**
     * Specify the gamepad attribute to check for this Button.
     * Example: gamepad1.a;
     */
    private interface ButtonCheck {
        boolean check();
    }

    private class Button{
        private boolean toggle = false;
        private boolean last = false;
        private final ButtonCheck buttonCheck;
        private Action tapAction = new EmptyAction();
        private Action idleAction = new EmptyAction();
        private Action onAction = new EmptyAction();
        private Action offAction = new EmptyAction();

        public void update(){
            if(!last && buttonCheck.check()) {
                tapAction.execute();
                toggle = !toggle;
            }
            else idleAction.execute();
            last = buttonCheck.check();

            if (toggle) onAction.execute();
            else offAction.execute();
        }

        public Button(ButtonCheck buttonCheck) {
            this.buttonCheck = buttonCheck;
        }
    }
}

