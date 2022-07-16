#coding=utf-8
from controller import Supervisor
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.animation import FuncAnimation

#def distance_initial():
    
def move_around():
    
    distance = 10 - ds.getValue()
    distance_r = 10 - ds_r.getValue()
    distance_l = 10 - ds_l.getValue()
    #print(distance)
    if(distance>0.5):
        if(distance_r<0.3 and distance_l>0.3):
            lp = left_motor.getTargetPosition()
            rp = right_motor.getTargetPosition() - 3.14/16
            left_motor.setPosition(lp)
            right_motor.setPosition(rp)
            #print('1')
            return
        if(distance_r>0.3 and distance_l<0.3):
            lp = left_motor.getTargetPosition() - 3.14/16
            rp = right_motor.getTargetPosition()
            left_motor.setPosition(lp)
            right_motor.setPosition(rp)
            #print('2')
            return 
        left_motor.setPosition(left_motor.getTargetPosition() - 3.14/16)
        right_motor.setPosition(right_motor.getTargetPosition() - 3.14/16)
        #print('3')
    else:
        lp = left_motor.getTargetPosition() - 3.14/16
        rp = right_motor.getTargetPosition()
        left_motor.setPosition(lp)
        right_motor.setPosition(rp)
        #print('5')
def init():
    ax[0].set_xlim(-4, 4)
    ax[0].set_ylim(-4, 4)
    
    return ln,ln1


def update(n):
    n = 30
    while robot.step(timestep) != -1:
        n = n - 1
        if(n!=0):
            move_around() 
            #print(n) 
            continue
        k = 10  
        pr = node1.getPosition()
        #print(pr)
        l = laser_sensor.getPointCloud(data_type='list')
        angle_r = node1.getOrientation()
        for i in range(360):
           x_w = pr[0] + l[i].x * angle_r[0] - l[i].y * angle_r[2]
           y_w = pr[2] + l[i].x * angle_r[6] - l[i].y * angle_r[8]
           ydata1.append(y_w)
           xdata1.append(x_w)
           #print(x_w)
           if(not(np.isnan(x_w)) and not(np.isnan(y_w))):
               print(x_w)
               x1_index = int((x_w + 4)/0.1)
               y1_index = int((y_w + 4)/0.1)
               a[resolution-1-y1_index][x1_index] = a[resolution-1-y1_index][x1_index] - 0.9
               if(a[resolution-1-y1_index][x1_index] < -1):
                   a[resolution-1-y1_index][x1_index] = -1
               b[resolution-1-y1_index][x1_index] = int(a[resolution-1-y1_index][x1_index]*127.5+127.5)
               x0_index = int((pr[0] + 4)/0.1)
               y0_index = int((pr[2] + 4)/0.1)
               dx = x1_index - x0_index
               dy = y1_index - y0_index
               ux = 1 if dx>0 else -1
               uy = 1 if dy>0 else -1
               dx2 = abs(2*dx)
               dy2 = abs(2*dy)
               if(abs(dx)>abs(dy)):
                   e = -dx
                   x_index = x0_index
                   y_index = y0_index
                   while (x_index!=x1_index):
                       e = e + dy2
                       if(e>0):
                           if(y_index != y1_index):
                               y_index = y_index + uy
                           e = e - dx2
                       x_index = x_index + ux
                       a[resolution-1-y_index][x_index] = a[resolution-1-y_index][x_index] + 0.05
                       if(a[resolution-1-y_index][x_index] > 1):
                           a[resolution-1-y_index][x_index] = 1
                       b[resolution-1-y_index][x_index] = int(a[resolution-1-y_index][x_index]*127.5+127.5)
               else:
                   e = -dy
                   x_index = x0_index
                   y_index = y0_index
                   while (y_index!=y1_index):
                       e = e + dx2
                       if(e>0):
                           if(x_index != x1_index):
                               x_index = x_index + ux
                           e = e - dy2
                       y_index = y_index + uy
                       a[resolution-1-y_index][x_index] = a[resolution-1-y_index][x_index] + 0.05
                       if(a[resolution-1-y_index][x_index] > 1):
                           a[resolution-1-y_index][x_index] = 1
                       b[resolution-1-y_index][x_index] = int(a[resolution-1-y_index][x_index]*127.5+127.5)
               
		
        np.savetxt(r'C:\Users\C\Desktop\webots_work\map.tex',b)
        xdata.append(pr[0])
        ydata.append(pr[2])
        dort = ax[1].scatter(xdata1,ydata1,s = 1)
        #print('yyds')
        g = ax[1].imshow(b,cmap='gray')
        ln = ax[0].scatter(xdata1,ydata1,s=1,c='b')
        ln1 = ax[0].scatter(xdata,ydata,s=15,c='r',marker='.')
        
        #print(xdata1)
        return ln,ln1,g

if __name__=="__main__":

    # create the Robot instance.
    robot = Supervisor()
    #robot.__init__()
    node1 = robot.getFromDef('myrobot')
    # get the time step of the current world.
    timestep = 64
    nan = float('nan')
    ds = robot.getDevice('distance_sensor')
    ds.enable(timestep)
    ds_r = robot.getDevice('distance_sensor_r')
    ds_r.enable(timestep)
    ds_l = robot.getDevice('distance_sensor_l')
    ds_l.enable(timestep)
    left_motor = robot.getDevice('left_motor')
    right_motor = robot.getDevice('right_motor')
    laser_sensor = robot.getDevice('LDS-01')
    laser_sensor.enable(timestep)
    laser_sensor.enablePointCloud()
    r_p_s = robot.getDevice('r_p_s')
    l_p_s = robot.getDevice('l_p_s')
    r_p_s.enable(timestep)
    l_p_s.enable(timestep)
    left_motor.setPosition(left_motor.getTargetPosition())
    right_motor.setPosition(right_motor.getTargetPosition())
    n = laser_sensor.getNumberOfPoints()
    
    resolution = 80
    a = np.zeros([resolution, resolution])
    b = np.zeros([resolution, resolution])
    for i in range(resolution):
        for j in range(resolution):
            b[i][j] = int(a[i][j] * 127.5 + 127.5)
    b[0][0] = 0
    b[resolution-1][resolution-1] = 255
    xdata,ydata,xdata1,ydata1 = [],[],[],[]
    #while robot.step(timestep) != -1:
     #   move_around() 
    fig,ax = plt.subplots(ncols = 1,nrows = 2)
    #ax.plot(xdata,ydata)
    ax[0].set_xlabel('x')
    ax[0].set_ylabel('y')
    ln = ax[0].scatter([],[],s=1,c='b')
    ln1 = ax[0].scatter([],[],s=1,c='r')   
    ani = FuncAnimation(fig,update,10,init_func=init,blit=True,interval=100)
    plt.show()
        
