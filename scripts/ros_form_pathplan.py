#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from outdoor_gcs.msg import PathPlan


class Form_PathPlan(object):

    def __init__(self, dt):

        print("Formation!!")

        self.dt = dt
        self.change_time = rospy.Time.now()

        self.agents_num, self.max_agents_num = 0, 9
        self.cur_pos = np.zeros((3*self.max_agents_num)) # max 5 drones, each with xyz
        self.cur_vel = np.zeros((3*self.max_agents_num))
        self.des_pos = np.zeros((3*self.max_agents_num))

        self.start_sim = False
        self.uavs_id = [False,False,False,False,False,False,False,False,False]

        self.agents = []
        
        

        self.pathplan_pub_msg, self.pathplan_pub_msg_nxt = PathPlan(), PathPlan()
        rospy.Subscriber("/uavs/pathplan", PathPlan, self.pathplan_callback)
        self.pathplan_pub = rospy.Publisher("/uavs/pathplan_nxt", PathPlan, queue_size=1)

    def pathplan_callback(self, msg):
        self.pathplan_pub_msg = msg
        self.start_sim = msg.start
        self.agents_num = msg.num
        self.uavs_id = msg.uavs_id
        self.cur_pos = np.asarray(msg.cur_position)
        self.cur_vel = np.asarray(msg.cur_velocity)
        self.des_pos = np.asarray(msg.des_position)
        if (msg.params[0]!= 0.0): self.c1 = msg.params[0]
        if (msg.params[1]!= 0.0): self.c2 = msg.params[1]
        if (msg.params[2]!= 0.0): self.RG = msg.params[2]
        if (msg.params[3]!= 0.0): self.r_alpha = msg.params[3]
        if (msg.params[4]!= 0.0): self.MaxAcc = msg.params[4]
        if (msg.params[5]!= 0.0): self.MaxVelo = msg.params[5]
        # self.c1, self.c2, self.RG = msg.params[0], msg.params[1], msg.params[2]
        # self.r_alpha, self.MaxAcc, self.MaxVelo = msg.params[3], msg.params[4], msg.params[5]

    def update_nxtpos(self):
        self.pathplan_pub_msg_nxt.header.stamp = rospy.Time.now()
        nxt, j = [], 0
        for i in range(self.max_agents_num):
            if (self.uavs_id[i]):
                pos = self.agents[j].pos_global_frame
                nxt.extend([pos[0], pos[1], pos[2]])
                # print([self.sim.getAgentPosition(j)[0], self.sim.getAgentPosition(j)[1], self.sim.getAgentPosition(j)[2]])
                j+=1
            else:
                nxt.extend([0,0,0])
        self.pathplan_pub_msg_nxt.nxt_position = list(nxt)
        # print(self.pathplan_pub_msg.nxt_position)

    def publish_msg(self):
        self.pathplan_pub_msg_nxt.header.stamp = rospy.Time.now()
        self.pathplan_pub.publish(self.pathplan_pub_msg_nxt)

    def iteration(self, event):
        if (self.start_sim and rospy.Time.now()-self.change_time > rospy.Duration(secs=5)):
            self.change_time = rospy.Time.now()
            self.pathplan_pub_msg_nxt.start, self.start_sim = False, False
            for i in range(self.max_agents_num):
                if (self.uavs_id[i] and len(self.agents) != self.agents_num):
                    self.agents.append(agent.Agent(self.cur_pos[i*3], self.cur_pos[i*3+1], self.cur_pos[i*3+2],
                        self.des_pos[i*3], self.des_pos[i*3+1], self.des_pos[i*3+2], i))
        if (self.agents_num == len(self.agents)):
            self.Move_One_Step()
            # self.update_nxtpos()
            self.publish_msg()

    def Move_One_Step(self):
        
        if (self.cur_pos[0]-self.des_pos[0])**2 + (self.cur_pos[1]-self.des_pos[1])**2 > 1:
            # 位置誤差判斷長機與目標點
            r1 = 0.01745*(-kp1r*(0-uav1Lp[0]))
            p1= 0.01745*(-kp1p*(30-uav1Lp[1]))
            a1L= [  -F/m*(math.cos(r1)*math.sin(p1)*math.sin(y1)+math.sin(r1)*math.cos(y1)),
                    -F/m*(math.cos(r1)*math.sin(p1)*math.cos(y1)+math.sin(r1)*math.sin(y1))]
            uav1Lv[0] += a1L[0]*dt
            uav1Lv[1] += a1L[1]*dt
            uav1Lp[0] += uav1Lv[0]*dt + 0.5*a1L[0]*dt*dt
            uav1Lp[1] += uav1Lv[1]*dt + 0.5*a1L[1]*dt*dt
            uav2dp = [uav1Lp[0]+d*math.sin(0.01745*(150+theta)), uav1Lp[1]+d*math.cos(0.01745*(150+theta))]
            uav3dp = [uav1Lp[0]+d*math.sin(0.01745*(210+theta)), uav1Lp[1]+d*math.cos(0.01745*(210+theta))]
            if uav1Lv[1] > 2:
                uav1Lv[1] = 2
                # a1L = [0,0]
            
            # 位置誤差判斷黃點與僚機
            r2 = 0.01745*(-kp2r*(uav2dp[0]-uav2Fp[0]))
            p2 = 0.01745*(-kp2p*(uav2dp[1]-uav2Fp[1]))
            a2F= [  -F/m*(math.cos(r2)*math.sin(p2)*math.sin(y2)+math.sin(r2)*math.cos(y2)),
                    -F/m*(math.cos(r2)*math.sin(p2)*math.cos(y2)+math.sin(r2)*math.sin(y2))]
            uav2Fv[0] += a2F[0]*dt
            uav2Fv[1] += a2F[1]*dt
            uav2Fp[0] += uav2Fv[0]*dt + 0.5*a2F[0]*dt*dt
            uav2Fp[1] += uav2Fv[1]*dt + 0.5*a2F[1]*dt*dt
            
            r3 = 0.01745*(-kp3r*(uav3dp[0]-uav3Fp[0]))
            p3 = 0.01745*(-kp3p*(uav3dp[1]-uav3Fp[1]))
            a3F= [  -F/m*(math.cos(r3)*math.sin(p3)*math.sin(y3)+math.sin(r3)*math.cos(y3)),
                    -F/m*(math.cos(r3)*math.sin(p3)*math.cos(y3)+math.sin(r3)*math.sin(y3))]
            uav3Fv[0] += a3F[0]*dt
            uav3Fv[1] += a3F[1]*dt
            uav3Fp[0] += uav3Fv[0]*dt + 0.5*a3F[0]*dt*dt
            uav3Fp[1] += uav3Fv[1]*dt + 0.5*a3F[1]*dt*dt

            if uav2Fv[1] > 4:
                uav2Fv[1] = 4
                # a2F = [0,0]
            if uav3Fv[1] > 4:
                uav3Fv[1] = 4
                # a3F = [0,0]
            if uav2dp[1]-uav2Fp[1] < 0.05 or uav3dp[1]-uav3Fp[1] < 0.05:
                uav2Fv[1] = uav1Lv[1]
                uav3Fv[1] = uav1Lv[1]


        for i in range(self.max_agents_num):\
            if i<3:
                nxt.extend(self.cur_pos[i*3:i*3+3] + vel*self.dt)
            else:
                nxt.extend([0,0,0])
        self.pathplan_pub_msg_nxt.nxt_position = list(nxt)


if __name__ == '__main__':

    r, p = 0, 0
    F, m, d = 6, 2, 5
    dt, tm = 0.1, 20
    kp1p, kp1r =0.2, 8
    kp2p, kp2r, kp2dr, kp2dp = 1.4, 8, 5, 7
    kp3p, kp3r, kp3dr, kp3dp = 1.4, 8, 10.5, 5.5
    theta = 0
    y1, y2, y3 = 0, 0, 0

    rospy.init_node('ros_form_pathplan', anonymous=True)
    dt = 1.0/15*3
    pathplan_run = Form_PathPlan(dt)
    rospy.Timer(rospy.Duration(dt), pathplan_run.iteration)
    rospy.spin()


     





