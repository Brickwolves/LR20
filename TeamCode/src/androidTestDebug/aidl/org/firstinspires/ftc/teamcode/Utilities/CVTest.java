package org.firstinspires.ftc.teamcode.Utilities;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Test.*;

class CVTest {

    @Test
    void main(){

        CV a = new CV(0.5);
        System.out.println(a.getCurrent());

        a.update(0.3);
        System.out.println(a.getCurrent());
        System.out.println(a.getLast());

    }

    @Test
    void getCurrent() {
    }

    @Test
    void getLast() {
    }

    @org.junit.jupiter.api.Test
    void setCurrent() {
    }

    @org.junit.jupiter.api.Test
    void update() {
    }
}