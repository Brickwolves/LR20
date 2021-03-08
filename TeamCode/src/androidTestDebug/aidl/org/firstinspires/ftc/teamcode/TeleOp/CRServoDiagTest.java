package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CRServoDiagTest {

    @Test
    void testClawMachine() {
    }

    enum STATE {
        IDLE,
        OPEN,
        CLOSE
    }
    private STATE current_state;

    private ElapsedTime time;
    private long millis_to_open = 1000;
    private long millis_to_close = 1000;

    private double power;

    @org.junit.jupiter.api.Test
    void clawMachine() {



        ElapsedTime time = new ElapsedTime();
        STATE current_state = STATE.IDLE;

        assertEquals(0, testMachine(false, false, false));
        assertEquals(0.5, testMachine(true, false, false));
        assertEquals(0.5, testMachine(false, false, false));
        hold(millis_to_open);
        assertEquals(0, testMachine(false, false, false));





    }

    public void hold(long duration){
        try { Thread.sleep(duration); }
        catch (Exception e){ }
    }
    public double testMachine(boolean DPAD_UP, boolean DPAD_DOWN, boolean SQUARE){

        switch (current_state) {

            case IDLE:
                power = 0;
                time.reset();

                if (DPAD_UP) current_state = STATE.OPEN;
                else if (DPAD_DOWN) current_state = STATE.CLOSE;

                break;

            case OPEN:
                if (SQUARE) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < millis_to_open) power = 0.5;
                else current_state = STATE.IDLE;
                break;

            case CLOSE:
                if (SQUARE) {
                    current_state = STATE.IDLE;
                    break;
                }
                else if (time.milliseconds() < millis_to_close) power = -0.5;
                else current_state = STATE.IDLE;
                break;
        }
        return power;
    }
}