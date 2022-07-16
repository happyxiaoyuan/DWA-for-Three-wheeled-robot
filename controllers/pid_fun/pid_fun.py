from controller import Robot
def pid_c(p,i,d,target):
     previous_err=0
     intergral=0
     while robot.step(timestep) != -1:
          v_now=left_motor.getVelocity()
          if(v_now==15):
              continue
          print(v_now)
          err=target-v_now
          intergral=intergral+err
          derivative=err-previous_err
          out=p*err+i*intergral+d*derivative
          if(out>20):
              out=20
          if(out<-20):
              out=-20;
          #print(out)
          previous_err=err
          left_motor.setVelocity(out)
          right_motor.setVelocity(out)
if __name__=='__main__':
      robot = Robot()
      timestep = int(robot.getBasicTimeStep())
      target_v=15
      previous_err=0
      intergral=0
      left_motor = robot.getDevice('left_motor')
      right_motor = robot.getDevice('right_motor')
      left_motor.setPosition(float('inf'))
      left_motor.setVelocity(0.0)
      right_motor.setPosition(float('inf'))
      right_motor.setVelocity(0.0)
      pid_c(0.01,0.5,0.01,target_v)
