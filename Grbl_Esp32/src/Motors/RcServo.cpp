/*
    RcServo.cpp

    This allows an RcServo to be used like any other motor. Serrvos
    do have limitation in travel and speed, so you do need to respect that.

    Part of Grbl_ESP32

    2020 -	Bart Dring

    The servos travel will be mapped against the axis xMaxTravel

    The rotation can be inverted with by $Stepper/DirInvert

    It will also respect HOMING_FORCE_POSITIVE_SPACE

    Homing simply sets the axis Mpos to the endpoint as determined by $Homing/DirInver

    Direction: The direction can be changed using the $3 setting for the axis

    Calibration is part of the setting (TBD) fixed at 1.00 now

    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "RcServo.h"

namespace Motors {
    RcServo::RcServo() {}

    RcServo::RcServo(uint8_t axis_index, uint8_t pwm_pin, float cal_min, float cal_max) {
        type_id               = RC_SERVO_MOTOR;
        this->axis_index      = axis_index % MAX_AXES;
        this->dual_axis_index = axis_index < MAX_AXES ? 0 : 1;  // 0 = primary 1 = ganged
        this->_pwm_pin        = pwm_pin;
        _cal_min              = cal_min;
        _cal_max              = cal_max;
        init();
    }

    void RcServo::init() {
        read_settings();
        _channel_num = sys_get_next_PWM_chan_num();
        ledcSetup(_channel_num, SERVO_PULSE_FREQ, SERVO_PULSE_RES_BITS);
        ledcAttachPin(_pwm_pin, _channel_num);
        _current_pwm_duty = 0;
        is_active         = true;  // as opposed to NullMotors, this is a real motor
        _can_home         = false; // this axis cannot be confensionally homed

        set_axis_name();
        config_message();
    }

    void RcServo::config_message() {
        grbl_msg_sendf(CLIENT_SERIAL,
                       MSG_LEVEL_INFO,
                       "%s Axis RC Servo motor Output:%d Cal Min:%5.3fmm Cal Max:%5.3fmm Min:%5.3fmm Max:%5.3fmm",
                       _axis_name,
                       _pwm_pin,
                       _cal_min,
                       _cal_max,
                       _pwm_pulse_min,
                       _pwm_pulse_max);
    }

    void RcServo::_write_pwm(uint32_t duty) {
        // to prevent excessive calls to ledcWrite, make sure duty hass changed
        if (duty == _current_pwm_duty)
            return;

        _current_pwm_duty = duty;
        
        ledcWrite(_channel_num, duty);
    }

    // sets the PWM to zero. This allows most servos to be manually moved
    void RcServo::set_disable(bool disable) {
        return;
        _disabled = disable;
        if (_disabled)
            _write_pwm(0);
    }

    // Homing justs sets the new system position and the servo will move there 
    void RcServo::set_homing_mode(uint8_t homing_mask, bool isHoming) {
        float home_pos = 0.0;

        if (homing_dir_mask->get() & bit(axis_index))
            home_pos = _position_min;
        else
            home_pos = _position_max;

        sys_position[axis_index] = home_pos * axis_settings[axis_index]->steps_per_mm->get();  // convert to steps
    }

    void RcServo::update() { set_location(); }

    void RcServo::set_location() {
        uint32_t servo_pulse_len;
        float    servo_pos, mpos, offset;
        // skip location if we are in alarm mode
        
        _get_calibration();

        if (sys.state == STATE_ALARM) {
            set_disable(true);
            return;
        }

        mpos      = system_convert_axis_steps_to_mpos(sys_position, axis_index);            // get the axis machine position in mm
        offset    = 0;  // gc_state.coord_system[axis_index] + gc_state.coord_offset[axis_index];  // get the current axis work offset
        servo_pos = mpos - offset;                                                          // determine the current work position

        // determine the pulse length
        servo_pulse_len = (uint32_t)mapConstrain(servo_pos, _position_min, _position_max, _pwm_pulse_min, _pwm_pulse_max);

        //grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "Servo Set %d", servo_pulse_len);

        _write_pwm(servo_pulse_len);
    }

    void RcServo::read_settings() { _get_calibration(); }

    // this should change to use its own settings.
    void RcServo::_get_calibration() {
       

#ifndef HOMING_FORCE_POSITIVE_SPACE
        _position_min = -axis_settings[axis_index]->max_travel->get();
        _position_max = 0;
#else
        _position_min = 0;
        _position_max = axis_settings[axis_index]->max_travel->get();
#endif

       

        _pwm_pulse_min = SERVO_MIN_PULSE;
        _pwm_pulse_max = SERVO_MAX_PULSE;

        //grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "Servo Pulse Min:%5.3f Max:%5.3f", _pwm_pulse_min, _pwm_pulse_max);

        if (bit_istrue(dir_invert_mask->get(), bit(axis_index))) {  // normal direction
            swap(_pwm_pulse_min, _pwm_pulse_max);
            _pwm_pulse_min *= (2.0 - _cal_min);
            _pwm_pulse_max *= (2.0 - _cal_max);

        } else {  // inverted direction
            _pwm_pulse_min *= _cal_min;
            _pwm_pulse_max *= _cal_max;
        }
        
    }
}
