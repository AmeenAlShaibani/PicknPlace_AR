#include <MotorController.h>


//Explicit Constructor
Motor::Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB, float ticks_per_rot) : encoder(encA,encB){
    _EN = EN;
    _PWM1 = PWM1;
    _PWM2 = PWM2;
    _SENSE = SENSE;

    _ticks_per_rot = ticks_per_rot;   

    _enc_count = 0;
    _previousTimeUpdate = 0;
}

Motor::Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB) : encoder(encA,encB){
    _EN = EN;
    _PWM1 = PWM1;
    _PWM2 = PWM2;
    _SENSE = SENSE;

    _enc_count = 0;
    _previousTimeUpdate = 0;
}

void Motor::init_motor(){
    pinMode(_EN,OUTPUT);
    pinMode(_PWM1,OUTPUT);
    pinMode(_PWM2,OUTPUT);
    //pinMode(_SENSE,INPUT); //V1.0 has incorrectly setup OP-AMP circuit
}

void Motor::drive_motor(int pwm_duty_cycle){
    pwm_duty_cycle = pwm_duty_cycle * _direction;
    if(pwm_duty_cycle < 0){
        analogWrite(_PWM2,abs(pwm_duty_cycle));
        analogWrite(_PWM1,0);
    }

    else{
        analogWrite(_PWM1,pwm_duty_cycle);
        analogWrite(_PWM2,0); 
    }
}

void Motor::drive_motor_setpoint(){
    int pwm_duty_cycle = _setpoint_pwm;
    if(pwm_duty_cycle < 0){
        analogWrite(_PWM2,abs(pwm_duty_cycle));
        analogWrite(_PWM1,0);
    }

    else{
        analogWrite(_PWM1,pwm_duty_cycle);
        analogWrite(_PWM2,0); 
    }
}

int Motor::read_enc(){
    _enc_count = encoder.read();
    return _enc_count;
}

void Motor::enable_motor(){
    digitalWrite(_EN,HIGH);

}

void Motor::disable_motor(){
    digitalWrite(_EN,LOW);
}

void Motor::brake_motor(int brake_power){
    analogWrite(_PWM1,brake_power);
    analogWrite(_PWM2,brake_power);
}

void Motor::assignSetpoint_pwm(int setpoint){
    _setpoint_pwm = setpoint;
}

int Motor::getSetpoint_pwm(){
    return _setpoint_pwm;
}

void Motor::setPID_vars_pos(float kP, float kI, float kD){
    _kP_pos = kP;
    _kI_pos = kI;
    _kD_pos = kD;
}

void Motor::setPID_vars_vel(float kP, float kI, float kD){
    _kP_vel = kP;
    _kI_vel = kI;
    _kD_vel = kD;
}

int Motor::pid_position(int setpoint){
  _currentTime = millis();
  int position = read_enc();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = float(setpoint - position);


  float output = 0.0;

  if(abs(_error) > 0.0){
    _cumulativeError += _error*_elapsedTime;
    _rateError = (_error-_lastError)/_elapsedTime;

    output = _kP_pos*_error + _kI_pos*_cumulativeError + _kD_pos*_rateError;
    
    if(output > 255.0) output = 255.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
  }

  else{
    drive_motor(0);
  }

return position;
}

float Motor::getVelocity(){
    _currentTime_vel = millis();
    _encoderCurrent_vel = read_enc();

    float deltaTime = (_currentTime_vel - _previousTime_vel)/MILLIS_PER_MIN;
    float deltaEncoder = (_encoderCurrent_vel - _encoderPast_vel)/_ticks_per_rot;

    _encoderPast_vel = _encoderCurrent_vel;
    _previousTime_vel = _currentTime_vel;

    return deltaEncoder/deltaTime; //RPM
}

float Motor::pid_velocity(float setpoint){
  _currentTime = millis();
  float vel = getVelocity();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = setpoint - vel;
  if(setpoint != _setpoint_vel_old) _cumulativeError = 0;

  float output = 0.0;

  if(setpoint == 0) drive_motor(0);

  else if(abs(_error) > 0.0){
    _cumulativeError += _error*_elapsedTime;
    _rateError = (_error-_lastError);

    output = _kP_vel*_error + _kI_vel*_cumulativeError + _kD_vel*_rateError;
    
    if(output > 255.0) output = 255.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
    _setpoint_vel_old = setpoint;
  }

  //else drive_motor(0);

  return vel;
}

void Motor::update_PID_Pos(int goal){
    _currentTime = millis();

    if(_currentTimeUpdate - _previousTimeUpdate >= _updateTime_PID){
        pid_position(goal);
        _previousTimeUpdate = millis();
    }

    else;

}

float Motor::update_PID_Vel(float setpoint){
    float vel = getVelocity();
    _currentTime = millis();
    if(_currentTimeUpdate - _previousTimeUpdate >= _updateTime_PID){
        vel = pid_velocity(setpoint);
        _previousTimeUpdate = millis();
    }
    return vel;
}

void Motor::update_PID_Vel_setpoint(){
    _currentTime = millis();;
    if(_currentTimeUpdate - _previousTimeUpdate >= _updateTime_PID){
        pid_velocity_setpoint();
        _previousTimeUpdate = millis();
    }
}

void Motor::setPIDUpdateRate(float millis){
    _updateTime_PID = millis;
}

void Motor::assignSetpoint_pos(int setpoint){
    _setpoint_pos = setpoint;
}

void Motor::assignSetpoint_vel(float setpoint){
    _setpoint_vel = setpoint;
}

void Motor::pid_position_setpoint(){
_currentTime = millis();
  int position = read_enc();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = float(_setpoint_pos - position);


  float output = 0.0;

  if(abs(_error) > 0.0){
    _cumulativeError += _error*_elapsedTime;
    _rateError = (_error-_lastError)/_elapsedTime;

    output = _kP_pos*_error + _kI_pos*_cumulativeError + _kD_pos*_rateError;
    
    if(output > 255.0) output = 255.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
  }

  else{
    drive_motor(0);
  }
}

float Motor::pid_velocity_setpoint(){
  _currentTime = millis();
  float vel = getVelocity();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = _setpoint_vel - vel;
  if(_setpoint_vel != _setpoint_vel_old) _cumulativeError = 0;


  float output = 0.0;

  if(_setpoint_vel == 0) drive_motor(0);

  else if(abs(_error) > 0.0){
    _cumulativeError += _error*_elapsedTime;
    _rateError = (_error-_lastError);

    output = _kP_vel*_error + _kI_vel*_cumulativeError + _kD_vel*_rateError;
    
    if(output > 255.0) output = 128.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
    _setpoint_vel_old = _setpoint_vel;
  }

  else drive_motor(0);


  return vel;
}

void Motor::setDirection(bool direction){
    if(direction == true) _direction = 1;
    else if(direction == false) _direction = -1;
}

void Motor::set_Ticks_Per_Rotation(float ticks_per_rotation){
    _ticks_per_rot = ticks_per_rotation;
}



/////////////////Communicators///////////////////////////
Serial_Comms::Serial_Comms(int baudrate, float timeout){
    _baudrate = baudrate;
    _timeout = timeout;
}

void Serial_Comms::init(){
    Serial.begin(_baudrate);
    Serial.setTimeout(_timeout);
}

void Serial_Comms::check_for_data(Message_Parser::Comm_Data& data){
    if (Serial.available() > 0){

        data.command_old = data.command;

        data.rx_str = Serial.readStringUntil('\n');

        DeserializationError   error = deserializeJson(data.rx_json, data.rx_str);
        if (error) Serial.println(error.c_str()); 


        if(data.rx_json["command"] == "pwm_direct"){
            data.command = PWM_DIRECT;
            data.goal_pwm[0] = data.rx_json["pwm0"];
            data.goal_pwm[1] = data.rx_json["pwm1"];
            data.goal_pwm[2] = data.rx_json["pwm2"];
            data.goal_pwm[3] = data.rx_json["pwm3"];
        }

        if(data.rx_json["command"] == "pos_pid"){
            data.command = POS_PID;
            data.goal_position[0] = data.rx_json["pos0"];
            data.goal_position[1] = data.rx_json["pos1"];
            data.goal_position[2] = data.rx_json["pos2"];
            data.goal_position[3] = data.rx_json["pos3"];
        }

        if(data.rx_json["command"] == "vel_pid"){
            data.command = VEL_PID;
            data.goal_velocity[0] = data.rx_json["vel0"];
            data.goal_velocity[1] = data.rx_json["vel1"];
            data.goal_velocity[2] = data.rx_json["vel2"];
            data.goal_velocity[3] = data.rx_json["vel3"];
        }

        if(data.rx_json["command"] == "pid_vars_pos_all"){
            data.command = PID_VARS_POS_ALL;
            for(int i = 0; i < 4; i++){
                data.kP_pos[i] = data.rx_json["P"];
                data.kI_pos[i] = data.rx_json["I"];
                data.kD_pos[i] = data.rx_json["D"];
            }
        }

        if(data.rx_json["command"] == "pid_vars_vel_all"){
            data.command = PID_VARS_VEL_ALL;
            for(int i = 0; i < 4; i++){
                data.kP_vel[i] = data.rx_json["P"];
                data.kI_vel[i] = data.rx_json["I"];
                data.kD_vel[i] = data.rx_json["D"];
            }
        }

        if(data.rx_json["command"] == "pid_vars_solo_pos"){
            if(data.rx_json["mot_num"] == -1) return; //Invalid motor num
            data.command = PID_VARS_SOLO_POS;
            int mot_num = data.rx_json["mot_num"];
            data.solo_motor = mot_num;
            data.kP_pos[mot_num] = data.rx_json["P"];
            data.kI_pos[mot_num] = data.rx_json["I"];
            data.kD_pos[mot_num] = data.rx_json["D"];
        }

        if(data.rx_json["command"] == "pid_vars_solo_vel"){
            if(data.rx_json["mot_num"] == -1) return; //Invalid motor num
            data.command = PID_VARS_SOLO_VEL;
            int mot_num = data.rx_json["mot_num"];
            data.solo_motor = mot_num;
            data.kP_vel[mot_num] = data.rx_json["P"];
            data.kI_vel[mot_num] = data.rx_json["I"];
            data.kD_vel[mot_num] = data.rx_json["D"];
        }

        if(data.rx_json["command"] == "set_dir"){
            if(data.rx_json["mot_num"] == -1) return; //Invalid motor num
            data.command = SET_DIR;
            data.solo_motor = data.rx_json["mot_num"];
            data.direction = data.rx_json["dir"];
        }

        if(data.rx_json["command"] == "set_tpr"){
            if(data.rx_json["mot_num"] == -1) return; //Invalid motor num
            data.command = SET_GEAR_RATIO;
            data.solo_motor = data.rx_json["mot_num"];
            data.ticks_per_rotation = data.rx_json["tpr"];
        }

        if(data.rx_json["command"] == "set_tpr_all"){
            data.command = SET_GEAR_RATIO_ALL;
            data.ticks_per_rotation = data.rx_json["tpr"];
        }

       return;
    }
    else return;
}

void Serial_Comms::send_feedback_data(Message_Parser::Comm_Data& data){
    data.tx_json["pos0"] = data.pos_feedback[0];
    data.tx_json["pos1"] = data.pos_feedback[1];
    data.tx_json["pos2"] = data.pos_feedback[2];
    data.tx_json["pos3"] = data.pos_feedback[3];
    data.tx_json["vel0"] = data.velocity_feedback[0];
    data.tx_json["vel1"] = data.velocity_feedback[1];
    data.tx_json["vel2"] = data.velocity_feedback[2];
    data.tx_json["vel3"] = data.velocity_feedback[3];

    serializeJson(data.tx_json,Serial);
    Serial.println();
    
    // Serial.print(data.tx_str);
    // Serial.flush();
    // data.tx_json.clear();
}
