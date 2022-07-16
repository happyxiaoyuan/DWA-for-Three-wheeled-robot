
#coding='gbk'
from controller import Supervisor
import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)  #根据速度限制和加速度限制得到的线速度和角速度范围

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = 0  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1  # [s]
        self.to_goal_cost_gain = 0.5
        self.speed_cost_gain = 1.5
        self.obstacle_cost_gain = 0.5
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.2  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.1  # [m] for collision check
        self.robot_length = 0.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array(np.loadtxt(r'F:\webots_work\bz3\webots_work\ob.tex')) 


    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):    #u为[线速度,角速度]
    """
    motion model
    """
    if u[1] != 0:
        x[0] = x[0] - u[0]/u[1] * math.sin(x[2]) + u[0]/u[1] * math.sin(x[2] + u[1] * dt) #更新x位置
        x[1] = x[1] + u[0]/u[1] * math.cos(x[2]) - u[0]/u[1] * math.cos(x[2] + u[1] * dt)   #更新y位置
    else:
        x[0] = x[0] + u[0]*dt*math.cos(x[2])
        x[1] = x[1] + u[0]*dt*math.sin(x[2])
    x[2] += u[1] * dt    #更新角度
    x[3] = u[0]                           #更新速度
    x[4] = u[1]                            #更新角速度


    return x                             #返回更新后机器人状态


def move(x, u, dt,left_motor,right_motor,l_p_s,r_p_s,node1):
    if u[1] != 0:
        x1 = abs(u[1])*dt*(u[0]/abs(u[1]) + 0.07)/0.03
        x2 = abs(u[1])*dt*(u[0]/abs(u[1]) - 0.07)/0.03
        r = r_p_s.getValue()
        #print(r)
        l = l_p_s.getValue()
        while robot.step(timestep) != -1:
            if u[1] > 0:
                #print('1')
                right_motor.setPosition(r-x2)
                left_motor.setPosition(l-x1)
                if abs(r_p_s.getValue()-right_motor.getTargetPosition()) < 0.005:
                    break
            else:
                #print('2')
                right_motor.setPosition(r-x1)
                left_motor.setPosition(l-x2)
                if abs(l_p_s.getValue()-left_motor.getTargetPosition()) < 0.005:
                    break
          
    else:
        x1 = u[0]*dt/0.03
        x2 = x1
        while robot.step(timestep) != -1: 
            right_motor.setPosition(r-x1)
            left_motor.setPosition(l-x2) 
            if abs(r_p_s.getValue()-right_motor.getTargetPosition()) < 0.005:
                    break
    #print(r_p_s.getValue()-right_motor.getTargetPosition())
    #print(l_p_s.getValue()-left_motor.getTargetPosition())
    ps = node1.getPosition()
    angle_r = node1.getOrientation()
    x[0] = ps[0] #更新x位置
    x[1] = ps[2]   #更新y位置
    x[2] = -math.acos(angle_r[0])   #更新角度
    x[3] = u[0]                           #更新速度
    x[4] = u[1]                            #更新角速度  
    
    return x

def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    #print(dw)
    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)      #一个dt后在该速度和角速度下的状态
        trajectory = np.vstack((trajectory, x))  #将状态添加到路径点集合
        time += config.dt                     #进入下一采样时间

    return trajectory                     #返回路径点集


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)   #对应线速度和角速度下一个predict_time后的路径
            # calc cost  评估函数（代价计算）
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)   #角度偏差*角度增益
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])    #最后一个状态点的速度和最大速度的差*速度增益
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)  #距离增益*1/最小距离

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:    #防止最优速度为0陷入僵直
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None] #路径点到每一个障碍物x距离
    dy = trajectory[:, 1] - oy[:, None]  #路径点到每一个障碍物y距离
    r = np.hypot(dx, dy)           #欧式距离计算

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]   #角度
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]) #
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2   #前向距离
        right_check = local_ob[:, 1] <= config.robot_width / 2    #右向距离
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2  #后向距离
        left_check = local_ob[:, 1] >= -config.robot_width / 2      #左向距离
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):    #计算最后一个路径点的角度偏差
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))   #返回pi到-pi之间角度

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")


def main(gx, gy, robot_type,x,left_motor,right_motor,l_p_s,r_p_s,node1):
  
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    trajectory = np.array(x) #路径点
    ob = config.ob    #障碍物
    while robot.step(timestep) != -1:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)  #得到最优[线速度，角速度]和最优预测路径（3s时间内）
        #x = motion(x, u, config.dt)
        x = move(x, u, config.dt,left_motor,right_motor,l_p_s,r_p_s,node1)  # simulate robot 按最优[线速度，角速度]运行一个采样时间
        #print(x)
        #print(predicted_trajectory[1] - x)
        trajectory = np.vstack((trajectory, x))  # store state history 将新到达的点储存到实际路径中

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    robot = Supervisor()
    #robot.__init__()
    node1 = robot.getFromDef('myrobot')
   
    # get the time step of the current world.
    timestep = 64
    left_motor = robot.getDevice('left_motor')
    right_motor = robot.getDevice('right_motor')
    r_p_s = robot.getDevice('r_p_s')
    l_p_s = robot.getDevice('l_p_s')
    r_p_s.enable(timestep)
    l_p_s.enable(timestep)
    ps = node1.getPosition()
    w = node1.getVelocity()
    angle_r = node1.getOrientation()
    x = np.array([ps[0],ps[2], math.acos(angle_r[0]), math.sqrt(w[0]**2+w[2]**2), w[4]])
    #print(x)
    gx = 0
    gy = -1
    main(gx,gy,RobotType.rectangle,x,left_motor,right_motor,l_p_s,r_p_s,node1)
    # main(robot_type=RobotType.circle)