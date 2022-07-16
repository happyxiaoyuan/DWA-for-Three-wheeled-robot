from controller import Robot
if __name__=='__main__':
      robot = Robot()
      timestep = int(robot.getBasicTimeStep())
      left_motor = robot.getDevice('left_motor')
      right_motor = robot.getDevice('right_motor')
      target = 10000
      l=left_motor.getPositionSensor()
      r=right_motor.getPositionSensor()
      l.enable(100*timestep)
      r.enable(100*timestep)
      while robot.step(timestep) != -1:
        left_motor.setPosition(target)
        right_motor.setPosition(target)
        print(l.getValue())
        print(r.getValue())
