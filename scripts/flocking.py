import numpy as np
from scipy.spatial.transform import Rotation as R
from Config import Config
from gekko import GEKKO
import sympy as sym

def move(host, agents, c1, c2, RG, ra, rx_mat, ry_mat, height_mat):
    # c1, c2 = Config.c1, Config.c2
    f_navigational = navigational_feedback(host, c1, c2)
    f_repulsive = repulsive_force(host, agents, RG, ra, rx_mat, ry_mat, height_mat)
    f_total = f_repulsive + f_navigational
    # if host.id == 0:
    #     print('total',f_total)

    return f_total

def navigational_feedback(host, c1, c2):
    f = -c1*(host.vel_global_frame) - c2*(host.pos_global_frame - host.goal_global_frame)
    # print(f)
    return f

def repulsive_force(host, agents, RG, ra, rx_mat, ry_mat, height_mat):
    f = np.array([0.0, 0.0, 0.0])

    # if host.id == 0:
    #     return f

    r_alpha = ra
    # r_alpha, h_alpha = Config.r_alpha, Config.h_alpha
    # RG = Config.RepulsiveGradient
    # print(RG)
    neighbors, rx_mat, ry_mat, height_mat = get_neighbors(host, agents, ra, rx_mat, ry_mat, height_mat)

    for neighbor in neighbors:
        dist_v = neighbor.pos_global_frame - host.pos_global_frame
        dist, distxy = np.linalg.norm(dist_v), np.linalg.norm(dist_v[:2])
        if not Config.CYL_MODEL: # Spherical model
            ForceComponent = -RG*bump_fn(dist/r_alpha) * np.square(dist-r_alpha)
            f += ForceComponent * (dist_v)/dist
        else: # Cylindrical model
            if abs(dist_v[2]) > height_mat[host.id][neighbor.id] and Config.TEST_DW_F and Config.ADAPT_H:
                continue
            else:
                h_alpha = height_mat[host.id][neighbor.id]
                print(h_alpha)
            ForceComponent_xy = -RG*bump_fn(distxy/r_alpha) * np.square(distxy-r_alpha)
            ForceComponent_z = -RG*bump_fn(abs(dist_v[2])/h_alpha) * np.square(abs(dist_v[2])-h_alpha)

            f[:2] += ForceComponent_xy*dist_v[:2]/dist
            f[2] += ForceComponent_z*dist_v[2]/dist
        # print(host.id, f)
    return f

def new_repulsive_force(dist_v, rx, ry, height):
    f = np.array([0.0, 0.0, 0.0])
    if abs(dist_v[0])<=rx and abs(dist_v[1])<=ry and abs(dist_v[2])<=height:
        f[0] = -Config.MaxAcc*adp_bump_fn(abs(dist_v[0])/rx, Config.r_alpha_min/rx)*abs(dist_v[0])/dist_v[0]
        f[1] = -Config.MaxAcc*adp_bump_fn(abs(dist_v[1])/ry, Config.r_alpha_min/ry)*abs(dist_v[1])/dist_v[1]
        f[2] = -Config.MaxAcc*adp_bump_fn(abs(dist_v[2])/height, Config.h_alpha_min/height)*abs(dist_v[2])/dist_v[2]
    return f

def get_neighbors(host, agents, ra, rx_mat, ry_mat, height_mat):
    neighbors = []
    r_alpha, h_alpha = ra, Config.h_alpha #max
    # r_alpha, h_alpha = Config.r_alpha, Config.h_alpha #max
    for other_agent in agents:
        if other_agent.id == host.id:
            continue
        dist_v = other_agent.pos_global_frame - host.pos_global_frame
        dist, distxy = np.linalg.norm(dist_v), np.linalg.norm(dist_v[:2])

        # Spherical and Cylindrical model
        if ((not Config.CYL_MODEL and dist < r_alpha) or 
            (Config.CYL_MODEL and distxy < r_alpha and abs(dist_v[2]) < h_alpha)):
            neighbors.append(other_agent)

    if (Config.CYL_MODEL and Config.TEST_DW_F and Config.ADAPT_H): # Adaptive Height Model Only
        rx_mat, ry_mat, height_mat = get_adp_height(host, neighbors, agents, rx_mat, ry_mat, height_mat)
    elif (Config.CYL_MODEL and Config.ADAPT_RH): # Adaptive Radius and Height Model
        rx_mat, ry_mat, height_mat = get_adp_radheight(host, neighbors, rx_mat, ry_mat, height_mat)
    elif Config.CYL_MODEL:
        for other_agent in agents:
            rx_mat[host.id][other_agent.id] = rx_mat[other_agent.id][host.id] = r_alpha
            ry_mat[host.id][other_agent.id] = ry_mat[other_agent.id][host.id] = r_alpha 
            height_mat[host.id][other_agent.id] = height_mat[other_agent.id][host.id] = Config.h_alpha

    return neighbors, rx_mat, ry_mat, height_mat


def get_adp_height(host, neighbors, agents, rx_mat, ry_mat, height_mat):
    dw_neighbor, tau_start, host_z = host, np.inf, np.inf
    r_alpha = Config.r_alpha
    h_min, h_max = Config.h_alpha_min, Config.h_alpha_max
    pos_i, goal_i, vel_i = host.pos_global_frame, host.goal_global_frame, host.vel_global_frame

    for other_agent in neighbors:
        if not (height_mat[host.id].get(other_agent.id) == None): 
            continue # since reciprocal
        dist_v = other_agent.pos_global_frame - pos_i
        vel_v = other_agent.vel_global_frame - vel_i
        dist, distxy = np.linalg.norm(dist_v), np.linalg.norm(dist_v[:2])

        rx_mat[host.id][other_agent.id] = rx_mat[other_agent.id][host.id] = r_alpha
        ry_mat[host.id][other_agent.id] = ry_mat[other_agent.id][host.id] = r_alpha 
        height_mat[host.id][other_agent.id] = height_mat[other_agent.id][host.id] = h_min
        if abs(dist_v[2]) < h_min:
            continue 
        sol_exist, tau_start_, tau_end_ = get_tau(dist_v, vel_v)
        if sol_exist and tau_start > tau_start_: # there is only one agent that can cause downwash
            dw_neighbor, tau_start, tau_end = other_agent, tau_start_, tau_end_
            t, host_z, vz = tau_start, pos_i[2], vel_i[2]
            while t < tau_end: # predict how altitude change due to dw
                dz = other_agent.pos_global_frame[2] - host_z
                f_n = 7*(goal_i[2] - pos_i[2])+9*vz
                dw_a = 25.0*(host.radius**2*np.pi)*9.81/(2*np.pi*(dz**2))
                vz -= (dw_a - max(min(f_n, Config.MaxAcc), -Config.MaxAcc))*0.01
                host_z += vz*0.01
                t += 0.01

    if dw_neighbor.id != host.id:
        pos_j = dw_neighbor.pos_global_frame
        # dw_a = -25.0*(host.radius**2*np.pi)*9.81/(2*np.pi*((pos_j-pos_i)[2]**2))
        pos_i_rep = pos_i + vel_i*tau_end + 0.5*Config.MaxAcc*(tau_end**2)*(pos_j-pos_i+0.0001)/(abs(pos_j-pos_i)+0.0001)
        pos_i_dw = pos_i+tau_end*vel_i
        pos_i_dw[2] = host_z
        agent_k, k_alt = get_victim(host, agents, dw_neighbor, tau_start, tau_end, host_z)
        height_mat[host.id][dw_neighbor.id] = height_mat[dw_neighbor.id][host.id] = (pos_j-pos_i)[2]*max(min(k_alt)/pos_i[2], Config.z_min/pos_i[2], np.linalg.norm(goal_i-pos_i_dw)/np.linalg.norm(goal_i-pos_i_rep))

    return rx_mat, ry_mat, height_mat



def get_adp_radheight(host, neighbors, rx_mat, ry_mat, height_mat):
    dw_neighbor, tau_start, host_z = host, np.inf, np.inf
    r_min = [Config.r_alpha_min, Config.r_alpha_min, Config.h_alpha_min]
    r_max = [Config.r_alpha_max, Config.r_alpha_max, Config.h_alpha_max]
    pos_i, goal_i, vel_i = host.pos_global_frame, host.goal_global_frame, host.vel_global_frame

    for other_agent in neighbors:
        if not (height_mat[host.id].get(other_agent.id) == None): 
            continue # since reciprocal

        pos_j = other_agent.pos_global_frame
        dist_v = pos_j - pos_i
        vel_v = other_agent.vel_global_frame - vel_i
        dist = np.linalg.norm(dist_v)

        rx_mat[host.id][other_agent.id] = rx_mat[other_agent.id][host.id] = r_min[0]
        ry_mat[host.id][other_agent.id] = ry_mat[other_agent.id][host.id] = r_min[1] 
        height_mat[host.id][other_agent.id] = height_mat[other_agent.id][host.id] = r_min[2] 

        # Check if uav is already inside min interaction region
        if abs(dist_v[0]) <= r_min[0] and abs(dist_v[1]) <= r_min[1] and abs(dist_v[2]) <= r_min[2]:
            continue
        
        # Downwash and height consideration
        h_req = r_min[2]
        if pos_i[2] < Config.z_min:
            h_req = r_max[2]
        else:
            sol_exist, tau_start, tau_end = get_tau(dist_v, vel_v)
            if sol_exist:
                t, host_z, vz = tau_start, pos_i[2], vel_i[2]
                while t < tau_end: # predict how altitude change due to dw
                    dz = other_agent.pos_global_frame[2] - host_z
                    f_n = 7*(goal_i[2] - pos_i[2])+9*vz
                    dw_a = 25.0*(host.radius**2*np.pi)*9.81/(2*np.pi*(dz**2))
                    vz -= (dw_a - max(min(f_n, Config.MaxAcc), -Config.MaxAcc))*0.01
                    host_z += vz*0.01
                    t += 0.01
                agent_k, k_alt = get_victim(host, neighbors, other_agent, tau_start, tau_end, host_z)
                if (len(agent_k)!=0 and min(k_alt)>=(pos_i)[2]): #(pos_j-pos_i)[2]
                    h_req = min(r_max[2], pos_j[2]-min(k_alt))

        # Determine proper radius and height
        delta_p, vel = np.copy(dist_v), np.copy(vel_i)
        while True: # Check if any time in the below process, the UAVs are in the minimum intersection region
            rep_force = new_repulsive_force(delta_p, abs(dist_v[0]), abs(dist_v[1]), max(abs(dist_v[2]), h_req))
            vel += rep_force*0.01
            for i in range(3):
                vel[i] = max(min(vel[i], Config.MaxVelo), -Config.MaxVelo)
            delta_p += vel*0.01
            if delta_p[0] <= r_min[0] and delta_p[1] <= r_min[1] and delta_p[2] <= r_min[2]: # If yes, determine the proper range
                determine_range = True
                break
            elif np.linalg.norm(vel_i) < 0.01 or (np.linalg.norm(delta_p) > np.linalg.norm(dist_v)): # getting further 
                determine_range = False
                break

        # print(determine_range)        
        delta_p, vel = np.copy(dist_v), np.copy(vel_i)
        if determine_range:
            ra, t_vss = [0,0,0], [0,0,0]
            for i in range(3):
                ra[i] = max(a_vss(dist_v[i],vel_i[i]), r_min[i]) #min(a_vss(dist_v[i],vel_i[i]), r_max[i]) # check if >rmin ??
                while True:
                    t_vss[i] += 0.01
                    if vel[i] == 0.0: break
                    a = Config.MaxAcc*adp_bump_fn(abs(delta_p[i])/ra[i], r_min[i]/ra[i])*abs(dist_v[i])/dist_v[i]
                    vel[i] = max(min(vel[i]+a*0.01, Config.MaxVelo), -Config.MaxVelo)
                    delta_p[i] += vel[i]*0.01
                    if vel[i] * (vel[i]+a*0.01) < 0: break
                    elif abs(delta_p[i])>ra[i]:      break
            index = t_vss.index(min(t_vss))
            for i in range(3):
                if i == index: continue
                if abs(dist_v[i]) < r_min[i]: 
                    ra[i] = r_min[i]
                    continue
                a_des = 2*(dist_v[i]-r_min[i]-vel_i[i]*t_vss[i])/(t_vss[i]**2)
                if abs(a_des) >= Config.MaxAcc:
                    ra[i] = r_max[i]
                else:
                    ra[i] = r_min[i] + np.pi*(abs(dist_v[i])-r_min[i]) / np.arccos(2*abs(a_des)/Config.MaxAcc - 1)
                    ra[i] = min(ra[i], r_max[i])
                    # if ra[i] < 0:
                    #     print(np.arccos(2*abs(a_des)/Config.MaxAcc - 1), ra[i])

            rx_mat[host.id][other_agent.id] = rx_mat[other_agent.id][host.id] = ra[0]
            ry_mat[host.id][other_agent.id] = ry_mat[other_agent.id][host.id] = ra[1]
            height_mat[host.id][other_agent.id] = height_mat[other_agent.id][host.id] = max(ra[2], h_req)

        else: height_mat[host.id][other_agent.id] = height_mat[other_agent.id][host.id] = h_req 

    return rx_mat, ry_mat, height_mat 

def a_vss(dp, v): #r_min, a_max, 
    
    if v == 0.0: return abs(dp)

    r_min, r_max, a_max = Config.r_alpha_min, Config.r_alpha_max, Config.MaxAcc
    dp = abs(dp)

    err = np.inf
    ra = dp + 0.01
    while ra < r_max:
        e = abs(v**2/2 - a_max*((dp-r_min)/(2*ra) + (1-r_min/ra)*np.sin(np.pi*(dp-r_min)/(ra-r_min))/(2*np.pi)))
        if err > e:
            err = e
            ra += 0.01
        else:
            # print(ra)
            return ra
    return r_max


def get_tau(dist_v, vel_v):
    A, C, z = dist_v
    B, D, vz = vel_v
    a, b, c = B**2+D**2, 2*(A*B+C*D), A**2+C**2-(2*Config.radius)**2
    if b**2-4*a*c > 0 and dist_v[2]>0:
        sol1, sol2 = (-b+np.sqrt(b**2-4*a*c))/(2*a), (-b-np.sqrt(b**2-4*a*c))/(2*a)
        if sol1 > 0 and sol2 > 0:
            tau_start, tau_end = min(sol1, sol2), max(sol1, sol2)
            return True, tau_start, tau_end
    return False, np.inf, np.inf


def get_victim(host, agents, dw_neighbor, tau_start, tau_end, host_z):
    agent_k, k_alt = [], [] # victim agent and their minimum height
    pos, vel = host.pos_global_frame, host.vel_global_frame+0.001
    P1, P2, P3, P4 = pos+tau_start*vel, pos+tau_end*vel, pos+tau_end*vel, pos+tau_start*vel
    P2[2], P3[2], P4[2] = P1[2], host_z, host_z
    n = np.cross(P2-P1, P3-P1)
    P_add1, P_sub1 = P1+2*host.radius*n/np.linalg.norm(n), P1-2*host.radius*n/np.linalg.norm(n)
    P_add2, P_sub2 = P_add1+(P2-P1)+2*host.radius*(P2-P1)/np.linalg.norm(P2-P1), P_sub1+(P2-P1)+2*host.radius*(P2-P1)/np.linalg.norm(P2-P1)
    P_nor1, P_nor2 = P1, P2+2*host.radius*(P2-P1)/np.linalg.norm(P2-P1)
    # P_ext = P2+2*host.radius*(P2-P1)/np.linalg.norm(P2-P1)
    x_range, y_range, z_range = [], [], []
    for i, other_agent in enumerate(agents):
        if other_agent.id == host.id or other_agent.id == dw_neighbor.id:
            continue
        pos_k, vel_k = other_agent.pos_global_frame, other_agent.vel_global_frame
        if (vel_k[0] == 0 and vel_k[1] == 0):
            if ((min(P_add1[0], P_add2[0]) < pos_k[0] < max(P_add1[0], P_add2[0]) and 
                min(P_add1[1], P_add2[1]) < pos_k[1] < max(P_add1[1], P_add2[1]) and
                (P1[2]+Config.height < pos_k[2] < P3[2]-Config.height)) or
                (min(P_nor1[0], P_nor2[0]) < pos_k[0] < max(P_nor1[0], P_nor2[0]) and 
                min(P_nor1[1], P_nor2[1]) < pos_k[1] < max(P_nor1[1], P_nor2[1]) and
                (P1[2]+Config.height < pos_k[2] < P3[2]-Config.height)) or
                (min(P_sub1[0], P_sub2[0]) < pos_k[0] < max(P_sub1[0], P_sub2[0]) and 
                min(P_sub1[1], P_sub2[1]) < pos_k[1] < max(P_sub1[1], P_sub2[1]) and
                (P1[2]+Config.height < pos_k[2] < P3[2]-Config.height))):

                agent_k.append(other_agent)
                k_alt.append(min(l_0[2], l_1[2]))
            continue
        l_0, l_1, l_v = pos_k+tau_start*vel_k, pos_k+tau_end*vel_k, (tau_end-tau_start)*vel_k
        # if (np.dot(l_v,n) == 0):
        #     continue
        d0, d1, d2 = np.dot((P_add1 - l_0), n)/(np.dot(l_v,n)+0.001), np.dot((P1 - l_0), n)/(np.dot(l_v,n)+0.001), np.dot((P_sub1 - l_0), n)/(np.dot(l_v,n)+0.001)
        if ((0<=d0 and d0<=1) or (0<=d1 and d1<=1) or (0<=d2 and d2<=1)):
            P_int0, P_int1, P_int2 = l_0+l_v*d0, l_0+l_v*d1, l_0+l_v*d2
            if ((min(P_add1[0], P_add2[0]) < P_int0[0] < max(P_add1[0], P_add2[0]) and 
                min(P_add1[1], P_add2[1]) < P_int0[1] < max(P_add1[1], P_add2[1]) and
                (P1[2]+Config.height < P_int0[2] < P3[2]-Config.height)) or
                (min(P_nor1[0], P_nor2[0]) < P_int1[0] < max(P_nor1[0], P_nor2[0]) and 
                min(P_nor1[1], P_nor2[1]) < P_int1[1] < max(P_nor1[1], P_nor2[1]) and
                (P1[2]+Config.height < P_int1[2] < P3[2]-Config.height)) or
                (min(P_sub1[0], P_sub2[0]) < P_int2[0] < max(P_sub1[0], P_sub2[0]) and 
                min(P_sub1[1], P_sub2[1]) < P_int2[1] < max(P_sub1[1], P_sub2[1]) and
                (P1[2]+Config.height < P_int2[2] < P3[2]-Config.height))):

                agent_k.append(other_agent)
                k_alt.append(min(l_0[2], l_1[2]))

    if len(k_alt) == 0: k_alt.append(-1)

    return agent_k, k_alt


def bump_fn(z):
    if not Config.BUMP or (z >= 0 and z < Config.h_bump):
        return 1
    elif z >= Config.h_bump and z <= 1:
        return 0.5*(1+np.cos(np.pi*((z-Config.h_bump)/(1-Config.h_bump))))
    return 0

def adp_bump_fn(z, h_bump):
    if (z >= 0 and z < h_bump):
        return 1
    elif z >= h_bump and z <= 1:
        return 0.5*(1+np.cos(np.pi*((z-h_bump)/(1-h_bump))))
    return 0

