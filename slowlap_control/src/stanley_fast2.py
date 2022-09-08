#! /usr/bin/env python

import rospy
import numpy as np
import math


from copy import deepcopy
import matplotlib.pyplot as plt
import sys


from numpy import *
from matplotlib import *
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray

# paramters
dt = 0.1

SHOW_ANIMATION = False # plot ìœ¼ë¡œ ê²°ê³¼ ë³´ì—¬ì¤„ì§€ ë§ì§€
# k =0.5
k = 1  # control gain

# initialize

MIN_T = 1 # minimum terminal time [s]
MAX_T = 2.5 # maximum terminal time [s], default = 2
DT_T = 0.5 # dt for terminal time [s] : MIN_T ì—ì„œ MAX_T ë¡œ ì–´ë–¤ dt ë¡œ ëŠ˜ë ¤ê°ˆì§€ë¥¼ ë‚˜íƒ€ëƒ„
DT = 0.1 # timestep for update

STEER_MAX = math.radians(45)
V_MAX = 50 / 3.6	  # maximum velocity [m/s]
ACC_MAX = V_MAX / DT # maximum acceleration [m/ss]
K_MAX = STEER_MAX/ V_MAX # maximum curvature [1/m]

TARGET_SPEED = 20 / 3.6 # target speed [m/s]
LANE_WIDTH = 2  # lane width [m]

# cost weights
K_J = 0.1 # weight for jerk, default = 0.1
K_T = 0.1 # weight for terminal time
K_D = 1.0 # weight for consistency, default = 1.0
K_D2 = 3.0 # weight for global path tracking
K_V = 1.0 # weight for getting to target speed
K_LAT = 1.0 # weight for lateral direction
K_LON = 1.0 # weight for longitudinal direction

# Vehicle parameters - plot ì„ ìœ„í•œ íŒŒë¼ë¯¸í„°
#LENGTH = 4.475  # [m]
#WIDTH = 1.850  # [m]
#BACKTOWHEEL = 0.1  # [m]
#WHEEL_LEN = 0.7  # [m]
#WHEEL_WIDTH = 0.5  # [m]
#TREAD = 1  # [m]
WB = 2.6  # [m]

# lateral planning ì‹œ terminal position condition í›„ë³´  (ì–‘ ì°¨ì„  ì¤‘ì•™), default len(DF_SET) = 2
DF_SET = np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2])
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians

def normalize_angle(angle):
	while angle > np.pi:
		angle -= 2.0 * np.pi

	while angle < -np.pi:
		angle += 2.0 * np.pi

	return angle


def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws, L):
	# find nearest point
	global k
	min_dist = 1e9
	min_index = 0
	n_points = len(map_xs)
	front_x = x + L * np.cos(yaw)
	front_y = y + L * np.sin(yaw)

	for i in range(n_points):
		dx = front_x - map_xs[i]
		dy = front_y - map_ys[i]

		dist = np.sqrt(dx * dx + dy * dy)
		if dist < min_dist:
			min_dist = dist
			min_index = i
	# compute cte at front axle
	map_x = map_xs[min_index]
	map_y = map_ys[min_index]
	map_yaw = map_yaws[min_index]
	dx = map_x - front_x
	dy = map_y - front_y

	perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
	cte = np.dot([dx, dy], perp_vec) # Cross track error

	# control law
#	yaw_term = normalize_angle(map_yaw - yaw) * np.sin(np.pi/2 / (1+v/5))
	yaw_term = normalize_angle(map_yaw - yaw) #heading error
	cte_term = np.arctan2(k*cte, v) # cross track error
	w_yaw = 1
	w_cte = 1
	# steering
	steer = w_yaw * yaw_term + w_cte * cte_term
	
	return steer#, [w_yaw, w_cte, k, yaw_term, cte_term]



class State:

	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=2.6):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
		self.dt = dt
		self.WB = WB

	def update(self, a, delta):
		dt = self.dt
		WB = self.WB

		self.x += self.v * math.cos(self.yaw) * dt
		self.y += self.v * math.sin(self.yaw) * dt
		self.yaw += self.v / WB * math.tan(delta) * dt
		self.yaw = pi_2_pi(self.yaw)
		self.v += a * dt
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

	def calc_distance(self, point_x, point_y):
		dx = self.rear_x - point_x
		dy = self.rear_y - point_y
		return math.hypot(dx, dy)

def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi

def next_waypoint(x, y, mapx, mapy):
	closest_wp = get_closest_waypoints(x, y, mapx, mapy)

	map_vec = [mapx[closest_wp + 1] - mapx[closest_wp], mapy[closest_wp + 1] - mapy[closest_wp]]
	ego_vec = [x - mapx[closest_wp], y - mapy[closest_wp]]

	direction  = np.sign(np.dot(map_vec, ego_vec))

	if direction >= 0:
		next_wp = closest_wp + 1
	else:
		next_wp = closest_wp

	return next_wp


def get_closest_waypoints(x, y, mapx, mapy):
	min_len = 1e10
	closest_wp = 0

	for i in range(len(mapx)):
		_mapx = mapx[i]
		_mapy = mapy[i]
		dist = get_dist(x, y, _mapx, _mapy)
		
		if dist < min_len:
			min_len = dist
			closest_wp = i		
	return closest_wp


def get_dist(x, y, _x, _y):
	return np.sqrt((x - _x)**2 + (y - _y)**2)

def get_frenet(x, y, mapx, mapy):
	next_wp = next_waypoint(x, y, mapx, mapy)
	prev_wp = next_wp -1

	n_x = mapx[next_wp] - mapx[prev_wp]
	n_y = mapy[next_wp] - mapy[prev_wp]
	x_x = x - mapx[prev_wp]
	x_y = y - mapy[prev_wp]

	proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
	proj_x = proj_norm*n_x
	proj_y = proj_norm*n_y

	#-------- get frenet d
	frenet_d = get_dist(x_x,x_y,proj_x,proj_y)

	ego_vec = [x-mapx[prev_wp], y-mapy[prev_wp], 0]
	map_vec = [n_x, n_y, 0]
	d_cross = np.cross(ego_vec,map_vec)
	if d_cross[-1] > 0:
		frenet_d = -frenet_d

	#-------- get frenet s
	frenet_s = 0
	for i in range(prev_wp):
		frenet_s = frenet_s + get_dist(mapx[i],mapy[i],mapx[i+1],mapy[i+1])

	frenet_s = frenet_s + get_dist(0,0,proj_x,proj_y)

	return frenet_s, frenet_d


def get_cartesian(s, d, mapx, mapy, maps):
	prev_wp = 0

	s = np.mod(s, maps[-2])

	while(s > maps[prev_wp+1]) and (prev_wp < len(maps)-2):
		prev_wp = prev_wp + 1

	next_wp = np.mod(prev_wp+1,len(mapx))

	dx = (mapx[next_wp]-mapx[prev_wp])
	dy = (mapy[next_wp]-mapy[prev_wp])

	heading = np.arctan2(dy, dx) # [rad]

	# the x,y,s along the segment
	seg_s = s - maps[prev_wp]

	seg_x = mapx[prev_wp] + seg_s*np.cos(heading)
	seg_y = mapy[prev_wp] + seg_s*np.sin(heading)

	perp_heading = heading + 90 * np.pi/180
	x = seg_x + d*np.cos(perp_heading)
	y = seg_y + d*np.sin(perp_heading)

	return x, y, heading

class QuinticPolynomial:

	def __init__(self, xi, vi, ai, xf, vf, af, T):
		# calculate coefficient of quintic polynomial
		# used for lateral trajectory
		self.a0 = xi
		self.a1 = vi
		self.a2 = 0.5*ai

		A = np.array([[T**3, T**4, T**5],
					  [3*T**2, 4*T**3, 5*T** 4],
					  [6*T, 12*T**2, 20*T**3]])
		b = np.array([xf - self.a0 - self.a1*T - self.a2*T**2,
					  vf - self.a1 - 2*self.a2*T,
					  af - 2*self.a2])
		x = np.linalg.solve(A, b)

		self.a3 = x[0]
		self.a4 = x[1]
		self.a5 = x[2]

	# calculate postition info.
	def calc_pos(self, t):
		x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4 + self.a5 * t ** 5
		return x

	# calculate velocity info.
	def calc_vel(self, t):
		v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3 + 5*self.a5*t**4
		return v

	# calculate acceleration info.
	def calc_acc(self, t):
		a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2 + 20*self.a5*t**3
		return a

	# calculate jerk info.
	def calc_jerk(self, t):
		j = 6*self.a3 + 24*self.a4*t + 60*self.a5*t**2
		return j

class QuarticPolynomial:

	def __init__(self, xi, vi, ai, vf, af, T):
		# calculate coefficient of quartic polynomial
		# used for longitudinal trajectory
		self.a0 = xi
		self.a1 = vi
		self.a2 = 0.5*ai

		A = np.array([[3*T**2, 4*T**3],
							 [6*T, 12*T**2]])
		b = np.array([vf - self.a1 - 2*self.a2*T,
							 af - 2*self.a2])

		x = np.linalg.solve(A, b)

		self.a3 = x[0]
		self.a4 = x[1]

	# calculate postition info.
	def calc_pos(self, t):
		x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4
		return x

	# calculate velocity info.
	def calc_vel(self, t):
		v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3
		return v

	# calculate acceleration info.
	def calc_acc(self, t):
		a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2
		return a

	# calculate jerk info.
	def calc_jerk(self, t):
		j = 6*self.a3 + 24*self.a4*t
		return j

class FrenetPath:

	def __init__(self):
		# time
		self.t = []

		# lateral traj in Frenet frame
		self.d = []
		self.d_d = []
		self.d_dd = []
		self.d_ddd = []

		# longitudinal traj in Frenet frame
		self.s = []
		self.s_d = []
		self.s_dd = []
		self.s_ddd = []

		# cost
		self.c_lat = 0.0
		self.c_lon = 0.0
		self.c_tot = 0.0

		# combined traj in global frame
		self.x = []
		self.y = []
		self.yaw = []
		self.ds = []
		self.kappa = []



def calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d):
	frenet_paths = []

	# generate path to each offset goal
	for df in DF_SET:

		# Lateral motion planning
		for T in np.arange(MIN_T, MAX_T+DT_T, DT_T):
			fp = FrenetPath()
			lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T)

			fp.t = [t for t in np.arange(0.0, T, DT)]
			fp.d = [lat_traj.calc_pos(t) for t in fp.t]
			fp.d_d = [lat_traj.calc_vel(t) for t in fp.t]
			fp.d_dd = [lat_traj.calc_acc(t) for t in fp.t]
			fp.d_ddd = [lat_traj.calc_jerk(t) for t in fp.t]

			# Longitudinal motion planning (velocity keeping)
			tfp = deepcopy(fp)
			lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T)

			tfp.s = [lon_traj.calc_pos(t) for t in fp.t]
			tfp.s_d = [lon_traj.calc_vel(t) for t in fp.t]
			tfp.s_dd = [lon_traj.calc_acc(t) for t in fp.t]
			tfp.s_ddd = [lon_traj.calc_jerk(t) for t in fp.t]

			# ê²½ë¡œ ëŠ˜ë ¤ì£¼ê¸° (In case T < MAX_T)
			for _t in np.arange(T, MAX_T, DT):
				tfp.t.append(_t)
				tfp.d.append(tfp.d[-1])
				_s = tfp.s[-1] + tfp.s_d[-1] * DT
				tfp.s.append(_s)

				tfp.s_d.append(tfp.s_d[-1])
				tfp.s_dd.append(tfp.s_dd[-1])
				tfp.s_ddd.append(tfp.s_ddd[-1])

				tfp.d_d.append(tfp.d_d[-1])
				tfp.d_dd.append(tfp.d_dd[-1])
				tfp.d_ddd.append(tfp.d_ddd[-1])

			J_lat = sum(np.power(tfp.d_ddd, 2))  # lateral jerk
			J_lon = sum(np.power(tfp.s_ddd, 2))  # longitudinal jerk

			# cost for consistency
			d_diff = (tfp.d[-1] - opt_d) ** 2
			# cost for target speed
			v_diff = (TARGET_SPEED - tfp.s_d[-1]) ** 2
			#cost for global path traking
			d_global_diff = (tfp.d[-1]) ** 2
			# lateral cost
			tfp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff + K_D2 * d_global_diff
			# logitudinal cost
			tfp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff

			# total cost combined
			tfp.c_tot = K_LAT * tfp.c_lat + K_LON * tfp.c_lon

			frenet_paths.append(tfp)

	return frenet_paths


def calc_global_paths(fplist, mapx, mapy, maps):

	# transform trajectory from Frenet to Global
	for fp in fplist:
		for i in range(len(fp.s)):
			_s = fp.s[i]
			_d = fp.d[i]
			_x, _y, _ = get_cartesian(_s, _d, mapx, mapy, maps)
			fp.x.append(_x)
			fp.y.append(_y)

		for i in range(len(fp.x) - 1):
			dx = fp.x[i + 1] - fp.x[i]
			dy = fp.y[i + 1] - fp.y[i]
			fp.yaw.append(np.arctan2(dy, dx))
			fp.ds.append(np.hypot(dx, dy))

		fp.yaw.append(fp.yaw[-1])
		fp.ds.append(fp.ds[-1])

		# calc curvature
		for i in range(len(fp.yaw) - 1):
			yaw_diff = fp.yaw[i + 1] - fp.yaw[i]
			yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
			fp.kappa.append(yaw_diff / fp.ds[i])

	return fplist

def check_path(fplist, mapx, mapy, maps):
	ok_ind = []
	for i, _path in enumerate(fplist):
		acc_squared = [(abs(a_s**2 + a_d**2)) for (a_s, a_d) in zip(_path.s_dd, _path.d_dd)]
		if any([v > V_MAX for v in _path.s_d]):  # Max speed check
			continue
		elif any([acc > ACC_MAX**2 for acc in acc_squared]):
			continue
		if any([abs(kappa) > K_MAX for kappa in fplist[i].kappa]):  # Max curvature check
			continue
		

		ok_ind.append(i)
	if len(ok_ind) == 0:
		return [fplist[i] for i in range(len(fplist))]
	return [fplist[i] for i in ok_ind]

def frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, mapx, mapy, maps, opt_d):
	fplist = calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
	fplist = calc_global_paths(fplist, mapx, mapy, maps)

	fplist = check_path(fplist, mapx, mapy, maps)
	# find minimum cost path
	min_cost = float("inf")
	opt_traj = None
	opt_ind = 0
	for fp in fplist:
		if min_cost >= fp.c_tot:
			min_cost = fp.c_tot
			opt_traj = fp
			_opt_ind = opt_ind
		opt_ind += 1

	try:
		_opt_ind
	except NameError:
		print(" No solution ! ")
		_opt_ind = -1

	return fplist, _opt_ind

def callback(msg):
    global map_x,map_y,map_yaw,maps
	map_x = []
	map_y = []
	map_yaw = []
	maps = []

	for i in range(len(msg.poses)-1):
		map_x.append(msg.poses[i].pose.position.x)
		map_y.append(msg.poses[i].pose.position.y)
		map_yaw.append(np.arctan2(msg.poses[i+1].pose.position.x-msg.poses[i].pose.position.x,
                        msg.poses[i+1].pose.position.y-msg.poses[i].pose.position.y))

	# get maps
	maps = np.zeros(map_x.shape)
	for i in range(len(map_x)-1):
		x = map_x[i]
		y = map_y[i]
		sd = get_frenet(x, y, map_x, map_y)
		maps[i] = sd[0]
	


def callback2(msg):

	# initial condition
	x = map_x[0]
	y = map_y[0]
	yaw = 90 * np.pi/180
	v = 0.1
	a = 0

	ind = 0
	s, d = get_frenet(x, y, map_x, map_y)
	x, y, yaw_road = get_cartesian(s, d, map_x, map_y, maps)
	yawi = yaw - yaw_road

	target_speed = 20 / 3.6
	
	state = State(x=map_x[ind], y=map_y[ind], yaw=map_yaw[ind], v=0.1, dt=0.1)
	prev_v = state.v
	error_ia = 0

	# s ë°©í–¥ ì´ˆê¸°ì¡°ê±´
	si = s
	si_d = state.v * np.cos(yawi)
	si_dd = 0
	sf_d = target_speed
	sf_dd = 0

	# d ë°©í–¥ ì´ˆê¸°ì¡°ê±´
	di = d
	di_d = state.v * np.sin(yawi)
	di_dd = 0
	df_d = 0
	df_dd = 0

	opt_d = d
	prev_opt_d = d

	
	while(True):
		# optimal planning ìˆ˜í–‰ (output : valid path & optimal path index)
		path, opt_ind = frenet_optimal_planning(si, si_d, si_dd,
												sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, map_x, map_y, maps, opt_d)

		if opt_ind == -1:
			s, d = get_frenet(state.x, state.y, map_x, map_y)
			x, y, road_yaw = get_cartesian(s, d, map_x, map_y, map_s)
			steer = road_yaw - state.yaw
			a = 0
			opt_d = prev_opt_d
		else:
			error_pa = target_speed - state.v
			error_da = state.v - prev_v
			error_ia += target_speed - state.v
			kp_a = 0.5
			kd_a = 0.7
			ki_a = 0.01
			a = kp_a * error_pa + kd_a * error_da + ki_a * error_ia
			steer = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, state.WB)

			opt_d = path[opt_ind].d[-1]
			prev_opt_d = opt_d

		
		# x y yaw ds kappa
		map_xs = path[opt_ind].x
		map_ys = path[opt_ind].y
		map_yaws = path[opt_ind].yaw

		steer = stanley_control(state.x, state.y, state.yaw, state.v, map_xs, map_ys, map_yaws, WB)
		pub.publish(steer)

		#longitudinal controller
		error_pa = target_speed - state.v
		error_da = prev_v - state.v
		error_ia += target_speed - state.v
		print("speed = " + str(state.v))
		kp_a = 0.5
		kd_a = 0.5
		ki_a = 0.01

		a = kp_a * error_pa + kd_a * error_da + ki_a * error_ia

		# update state with acc, delta
		prev_v = state.v
		state.x = msg.pose.pose.position.x
		state.y = msg.pose.pose.position.y
		state.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
		state.update(a, steer)

		s, d = get_frenet(state.x, state.y, map_x, map_y)
		x, y, yaw_road = get_cartesian(s, d, map_x, map_y, maps)
		yaw_diff = state.yaw - yaw_road

		si = s
		si_d = state.v * math.cos(yaw_diff)
		si_dd = a * math.cos(yaw_diff)
		sf_d = target_speed
		sf_dd = 0

		di_d = state.v * math.sin(yaw_diff)
		di_dd = a * math.sin(yaw_diff)
		df_d = 0
		df_dd = 0

		# consistency costë¥¼ ìœ„í•´ update


		if SHOW_ANIMATION:  # pragma: no cover

			plt.cla()
			# for stopping simulation with the esc key.
			plt.gcf().canvas.mpl_connect(
				'key_release_event',
				lambda event: [exit(0) if event.key == 'escape' else None])

			plt.plot(mapx[:], mapy[:], 'k', linewidth=2)

			# plot obstacle
			for ob in obs_global:
				plt.plot(ob[0], ob[1], "s", color="crimson", MarkerSize=15, alpha=0.6)

			for i in range(len(path)):
					plt.plot(path[i].x, path[i].y, "-", color="crimson", linewidth=1.5, alpha=0.6)

			plt.plot(path[opt_ind].x, path[opt_ind].y, "o-", color="dodgerblue", linewidth=3)

			# plot car
			plot_car(state.x, state.y, state.yaw, steer = di)

			plt.axis([state.x-30, state.x+30,state.y-30,state.y+30])
			plt.title("[Simulation] v : " + str(si_d)[0:4] + " m/s")
			plt.grid(True)
			plt.xlabel("X [m]")
			plt.ylabel("Y [m]")
			
			plt.pause(0.01)
			# input("Press enter to continue...")

if __name__ == '__main__':
    rospy.init_node('stanley_control')
    rospy.Subscriber('/loop_path',Path,callback)
    rospy.Subscriber('/odom',Odometry,callback2)

    pub = rospy.Publisher('/steer',Float32,queue_size=1)
    rospy.spin()
