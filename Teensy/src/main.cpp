/**
 * Author: Dan Stoianovici
 * Email: stoianovici.dan.s@gmail.com
 * License: MIT
 * 
 *  ROS Enabled Motor Controller for Teensy4.0 Motor Controller Board
 * 
**/


#include <MotorController_Pins.h>
#include <MotorController.h>

#define SERIAL_COM

#define GEAR_RATIO 50.9
#define COUNT_PER_ROT_ENC 12
#define COUNT_PER_ROT 1425.1

#define BAUD 115200
#define TIMEOUT 0.5

Message_Parser::Comm_Data comm_data;


Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B);
Motor mot1(MOT1_EN,MOT1_PWM1,MOT1_PWM2,SENSE0,ENC1_A,ENC1_B); 
Motor mot2(MOT2_EN,MOT2_PWM1,MOT2_PWM2,SENSE0,ENC2_A,ENC2_B); 
Motor mot3(MOT3_EN,MOT3_PWM1,MOT3_PWM2,SENSE0,ENC3_A,ENC3_B);

std::vector<Motor*> _motors;



#ifdef SERIAL_COM
Serial_Comms serial_comms(BAUD,TIMEOUT);
#endif



void setup() {

  #ifdef SERIAL_COM

    serial_comms.init();
    
  #endif

  _motors.push_back(&mot0);
  _motors.push_back(&mot1);
  _motors.push_back(&mot2);
  _motors.push_back(&mot3);

  for(std::size_t i = 0; i < _motors.size(); i++){
          _motors[i]->init_motor();
          _motors[i]->enable_motor();
          // _motors[i]->setPID_vars_pos(1.0,0.00,0.00);
          // _motors[i]->setPID_vars_vel(0.5,0.00,0.00);
          // _motors[i]->set_Ticks_Per_Rotation(COUNT_PER_ROT);
  }

  mot0.setPID_vars_pos(2.5,0,0);
  mot0.setPID_vars_vel(0.5,0.0,0.0);
  mot0.set_Ticks_Per_Rotation(COUNT_PER_ROT);

  mot1.setPID_vars_pos(2.35,0,0);
  mot1.setPID_vars_vel(0.5,0.0,0.0);
  mot1.set_Ticks_Per_Rotation(COUNT_PER_ROT);

  mot2.setPID_vars_pos(2.56,0.0,0.0);
  mot2.setPID_vars_vel(0.5,0.0,0.0);
  mot2.set_Ticks_Per_Rotation(COUNT_PER_ROT);

  mot3.setPID_vars_pos(2.67,0,0);
  mot3.setPID_vars_vel(0.6,0.0,0.0);
  mot3.set_Ticks_Per_Rotation(COUNT_PER_ROT);

}

void loop() {


  #ifdef SERIAL_COM

  serial_comms.check_for_data(comm_data);
 
  switch(comm_data.command){
      default:
        break;

      case NONE:
        for(std::size_t i = 0; i < _motors.size(); i++){
          _motors[i]->drive_motor(0);
          comm_data.pos_feedback[i] = _motors[i]->read_enc();
          comm_data.velocity_feedback[i] = _motors[i]->getVelocity();
        }
        break;

      case PWM_DIRECT:
        for(std::size_t i = 0; i < _motors.size(); i++){
          _motors[i]->drive_motor(comm_data.goal_pwm[i]);
          comm_data.pos_feedback[i] = _motors[i]->read_enc();
          comm_data.velocity_feedback[i] = _motors[i]->getVelocity();
        }
        break;

      case POS_PID:
        for(std::size_t i = 0; i < _motors.size(); i++){
          comm_data.pos_feedback[i] = _motors[i]->pid_position(comm_data.goal_position[i]);
          comm_data.velocity_feedback[i] = _motors[i]->getVelocity();
        }
        break;
      
      case VEL_PID:
        for(std::size_t i = 0; i < _motors.size(); i++){
          comm_data.pos_feedback[i] = _motors[i]->read_enc();
          comm_data.velocity_feedback[i] = _motors[i]->pid_velocity(comm_data.goal_velocity[i]);
        }
        break;

      case SET_DIR:
        _motors[comm_data.solo_motor]->setDirection(comm_data.direction);
        comm_data.command = comm_data.command_old;
        break;
      
      // case PID_VARS_SOLO_POS:
      //   _motors[comm_data.solo_motor]->setPID_vars_pos(
      //     comm_data.kP_pos[comm_data.solo_motor],
      //     comm_data.kI_pos[comm_data.solo_motor],
      //     comm_data.kD_pos[comm_data.solo_motor]);
      //   comm_data.command = comm_data.command_old;
      //   break;
      
      // case PID_VARS_SOLO_VEL:
      //   _motors[comm_data.solo_motor]->setPID_vars_vel(
      //     comm_data.kP_vel[comm_data.solo_motor],
      //     comm_data.kI_vel[comm_data.solo_motor],
      //     comm_data.kD_vel[comm_data.solo_motor]);
      //   comm_data.command = comm_data.command_old;
      //   break;
      
      // case SET_GEAR_RATIO:
      //   _motors[comm_data.solo_motor]->set_Ticks_Per_Rotation(comm_data.ticks_per_rotation);
      //   comm_data.command = comm_data.command_old;
      //   break;
      
      // case SET_GEAR_RATIO_ALL:
      //   for(std::size_t i = 0; i < _motors.size(); i++){
      //     _motors[i]->set_Ticks_Per_Rotation(comm_data.ticks_per_rotation);
      //   }
      //   comm_data.command = comm_data.command_old;
      //   break;
  }

  serial_comms.send_feedback_data(comm_data);


  delay(50);
}



  #endif
