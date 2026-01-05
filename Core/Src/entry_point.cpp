//
// Created by hhw on 2025/12/21.
//

#include "../Inc/entry_point.h"
#include "motor.h"

void start_up() {
        while (true) {
                motor m;
                m.set_speed(100);
                m.set_direction(true);
                m.move_to_position(50);


        }
}





