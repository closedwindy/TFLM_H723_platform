//
// Created by hhw on 2025/12/21.
//

#ifndef INC_45245254_MOTOR_H
#define INC_45245254_MOTOR_H

class motor {

        public:
        motor();
        ~motor() = default;
        virtual void set_speed(int s);
        virtual void set_direction(bool dir);
        virtual void move_to_position(int pos);
        virtual int  get_speed();
        virtual bool get_direction();
        virtual int  get_position();


        private:
        int  speed;
        bool direction;
        int  position;

};

#endif //INC_45245254_MOTOR_H