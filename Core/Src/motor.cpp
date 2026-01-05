//
// Created by hhw on 2025/12/21.
//

#include "../Inc/motor.h"
motor::motor() : speed(0), direction(0), position(0) {};
void motor::set_speed(int s) {
        speed = s;
}
void motor::set_direction(bool dir) {
        direction = dir;
}
void motor::move_to_position(int pos) {
        position = pos;
}
int motor::get_speed() {
        return speed;
}
bool motor::get_direction() {
        return direction;
}
int motor::get_position() {
        return position;
}
