#!/usr/bin/env python

#============================================================#
#                                                            #
#                    IN THE NAME OF GOD                      #
#                    ------------------                      #
#                                                            #
#               Erfan Zekri Esfahani - 9815294               #
#                      (Particle Filter)                     #
#                 Final Project of Robotics                  #
#                                                            #
#============================================================#

import rospy
import numpy as np
import random
import time
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist


class Particle:
    def __init__(self, x, y, theta=0, w=None):
        # super().__init__()
        self.x = x
        self.y = y
        self.theta = theta
        self.w = w

class robot:

    def __init__(self, world_size = 100.0, measurement_range = 30.0,
                 motion_noise = 1.0, measurement_noise = 1.0):
        self.measurement_noise = 0.0
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size / 2.0
        self.y = world_size / 2.0
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.landmarks = []
        self.num_landmarks = 0
        self.velocity = 0
        self.theta = 0


    def rand(self):
        return random.random() * 2.0 - 1.0

    def move(self, u_t):

        x_t_1 = (self.x, self.y, self.theta)
        (new_x, new_y, new_theta) = sample_motion_model_velocity(u_t, x_t_1)

        if new_x < 0.0 or new_x > self.world_size or new_y < 0.0 or new_y > self.world_size:
            return False
        else:
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
            return True
    
    def add_noise(self, x):
      x_n = x + self.rand() #* self.motion_noise
      return x_n

    def sense(self):

        measurements = []
        all_landmarks = []

        for lm in range(num_landmarks):
          dx = self.add_noise(self.landmarks[lm][0]) - self.x
          dy = self.add_noise(self.landmarks[lm][1]) - self.y
          r_t = math.sqrt(math.pow(dx, 2)+math.pow(dy, 2))
          all_landmarks.append(lm)
          if r_t <= measurement_range:
            ct = lm
            phi_t = math.atan2(dy, dx)
            s_t = self.landmarks[lm][2]
            measurements.append([r_t, phi_t, s_t, lm])
        return measurements


    
    def make_landmarks(self, num_landmarks):
        self.landmarks = []
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size), i])
        self.num_landmarks = num_landmarks
    
    
    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]'  % (self.x, self.y)


fig = plt.figure()
ax = fig.add_subplot(111)
plt.ion()

def Show(world_size, position, landmarks=None, particles=None, wait=10):
    # plot the whole problem
    fig.clear()

    plt.xlim(-1, world_size+1)
    plt.ylim(-1, world_size+1)

    plt.scatter(position[0], position[1], marker='o', s=70, color = 'r')
    
    # Draw landmarks if they exists
    if(landmarks is not None):
        # loop through all path indices and draw a dot (unless it's at the car's location)
        for pos in landmarks:
            if(pos != position):
                plt.scatter(pos[0], pos[1], marker='^', s=60, color = 'purple')
    
    # Draw particles if they exists
    if(particles is not None):
        for par in particles:
            plt.scatter(par.x, par.y, marker='o', s=1, color = 'blue')
    
    plt.pause(wait)

def Gaussian_Sampler(mean, var):
    return np.random.normal(mean, var)

def prob(a,b):
  return (1/math.sqrt(2*math.pi*b))*math.exp(-0.5*math.pow(a,2)/b)

def sample_motion_model_velocity(u_t, x_t_1, delta_t=1):
    # print("hello")
    alpha1 = 0.5
    alpha2 = 0.5
    alpha3 = 0.1
    alpha4 = 0.1
    alpha5 = 0.1
    alpha6 = 0.1
    v = u_t[0]
    w = u_t[1]
    x = x_t_1[0]
    y = x_t_1[1]
    theta = x_t_1[2]
    v_hat = v + Gaussian_Sampler(0, alpha1*v*v + alpha2*w*w)
    w_hat = w + Gaussian_Sampler(0, alpha3*v*v + alpha4*w*w)
    gama_hat = Gaussian_Sampler(0, alpha5*v*v + alpha6*w*w)
    x_prim = x - (v_hat/w_hat)* math.sin(theta) + (v_hat/w_hat) * math.sin(theta + w_hat*delta_t)
    y_prim = y + (v_hat/w_hat)* math.cos(theta) - (v_hat/w_hat) * math.cos(theta + w_hat*delta_t)
    theta_prim = theta + w_hat * delta_t + gama_hat * delta_t
    return (x_prim, y_prim, theta_prim)

def landmark_model_known_correspondence(ft,ct,xt,m):
      j = ct
      r_t = ft[0]
      phi_t = ft[1]
      s_t = ft[2]
      x = xt[0]
      y = xt[1]
      theta = xt[2]
      r_hat = math.sqrt(math.pow(m[j][0] - x,2)+math.pow(m[j][1] - y,2))
      phi_hat = math.atan2(m[j][1] - y,m[j][0] - x)
      s_hat = m[j][2]
      q = prob(r_t - r_hat,1)*prob(phi_t - phi_hat,1)*prob(s_t - s_hat,1)
      return q


def Particle_Filter(chi_t_1, u_t, z_t):
    # print("filter step")
    global robot
    chi_bar_t = []
    chi_t = []
    weights = []
    sum = 0
    for i in range(num_of_particles):
        # Sampling 
        x_t_1 = (chi_t_1[i].x, chi_t_1[i].y, chi_t_1[i].theta)
        (new_x, new_y, new_theta) = sample_motion_model_velocity(u_t, x_t_1)
        # Set Weights
        m =  robot.landmarks
        x_t = [chi_t_1[i].x, chi_t_1[i].y, chi_t_1[i].theta]
        features = robot.sense()
        if features==[]:
            w = 0.5
        else:
            w = 1
            for f in features:
                f_t = [f[0], f[1], f[2]]
                c_t = f[3]
                w *= landmark_model_known_correspondence(f_t, c_t, x_t, m)
        # add to new chi
        if (new_x <= world_size and new_x>=0) and (new_y <= world_size and new_y>=0):
            chi_bar_t.append(Particle(new_x, new_y, theta=new_theta, w=w))
            weights.append(w)
            sum += w
        else:
            chi_bar_t.append(Particle(x_t_1[0], x_t_1[1], theta=new_theta, w=w))
            weights.append(w)
            sum += w

    for i in range(num_of_particles):
        weights[i] = weights[i]/sum
    
    chi_t = np.random.choice(chi_bar_t, size=num_of_particles, p=weights)
        
    # chi_t = chi_bar_t
    return chi_t


def Show_listening_data(data):
    global velocity, angular
    velocity = data.linear.x
    angular = data.angular.z


if __name__ == "__main__":
    # welcome text
    print("Hello\nMy name is Erfan Zekri Esfahani\nThis project is implementation of Particle Filter for localization for Robotics course\nat Isfahan University of Technology (IUT)\nStarting...\n\n")
    
    # initial values
    global velocity, angular
    velocity = 0
    angular = 0
    particles = []
    world_size         = 30.0     # size of world (square)
    measurement_range  = 15.0     # range at which we can sense landmarks
    motion_noise       = 0.2      # noise in robot motion
    measurement_noise  = 0.2      # noise in the measurements
    num_landmarks = 5             # number of landmarks
    num_of_particles = 500        # number of particles
    r_variance = 0.05             # variance of movement
    theta_variance = 0.05         # variance of rotation

    # instantiate a robot, r
    global robot
    robot = robot(world_size, measurement_range, motion_noise, measurement_noise)

    # create landmarks
    robot.make_landmarks(num_landmarks)
    
    # define figure size
    plt.rcParams["figure.figsize"] = (7, 7)

    # # build particles
    for i in range(num_of_particles):
        x = random.random()*world_size
        y = random.random()*world_size
        particles.append(Particle(x, y))

    # Get Input from user
    rospy.init_node("particle_show", anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, Show_listening_data)

    while not rospy.is_shutdown():
        if(velocity!=0 or angular!=0):
            # particle filter
            particles = Particle_Filter(particles, [velocity, angular], None)

            # robot movement
            robot.move([velocity, angular])
        Show(int(world_size), [robot.x, robot.y], robot.landmarks, particles, 0.05)
        

    
    plt.show()