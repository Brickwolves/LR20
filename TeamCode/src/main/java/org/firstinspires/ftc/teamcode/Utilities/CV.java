package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;
import java.util.List;

public class CV<T> {
        protected T current_v;
        protected T last_v;

        public CV(T v){
            current_v = v;
        }

        public T getCurrent() {
            return current_v;
        }
        public T getLast(){
            return last_v;
        }

        public void update(T v){
            last_v = current_v;
            current_v = v;
        }

}
