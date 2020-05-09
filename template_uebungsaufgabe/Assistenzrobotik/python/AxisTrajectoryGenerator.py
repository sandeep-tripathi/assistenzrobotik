#!/usr/bin/env python2
import math

import matplotlib.pyplot as plt
import numpy as np


class AxisTrajectoryGenerator:
    def __init__(self, max_velocitys, max_accels):
        self.max_velocitys = max_velocitys
        self.max_accels = max_accels

    def genSyncMove(self, current_configuration, target_configuration, dt, speed):
        delta_move = np.asarray(target_configuration) - np.asarray(current_configuration)
        axis_sign = []
        times = []
        for i in range(delta_move.shape[0]):
            if delta_move[i] < 0:
                axis_sign.append(-1)
            else:
                axis_sign.append(1)

            t0, t1, t2, t3, v0, v1, v2, v3, s0, s1, s2, s3, a0, a1, a2, a3 = self.calc_trajectory(abs(delta_move[i]),
                                                                                                  self.max_velocitys[
                                                                                                      i] * speed,
                                                                                                  self.max_accels[i], 0, 0)
            times.append(t3)

        movement_time = np.max(times)

        axis_trajectorys = []
        time_array = None

        for i in range(delta_move.shape[0]):
            t0, t1, t2, t3, s0, s3, a0 = self.calc_trajectory_timegiven(abs(delta_move[i]), self.max_accels[i], 0, 0,
                                                                        movement_time)
            time_array, pose_array = self.sample_trajectory_new(dt, t0, t1, t2, t3, s0, s3, a0, axis_sign[i])
            pose_array += current_configuration[i]
            axis_trajectorys.append(pose_array)

        return time_array, axis_trajectorys

    def _speed_reached_time(self, speed, target_speed, accel):
        dv = target_speed - speed
        return abs(dv / accel)

    def _get_pose(self, accel, speed, time):
        s = 0.5 * accel * time ** 2 + speed * time
        return s

    def _get_time(self, accel, speed, pose):
        time = (-speed + math.sqrt(speed ** 2 - (2 * accel * -abs(pose)))) / accel
        return time

    def calc_trajectory_timegiven(self, target_pose, target_acceleration, s0, t0, t3):
        # check for numerical glitchessqrt()
        root = (t3 ** 2 * target_acceleration - 4 * (target_pose - s0)) / target_acceleration
        if root < 0:
            root = 0
        t0 = t0
        t1 = 0.5 * (t3 - math.sqrt(root))
        t2 = t3 - t1
        t3 = t3

        a0 = target_acceleration
        a1 = 0
        a2 = -target_acceleration
        a3 = 0

        v0 = 0
        v1 = t1 * a0
        v2 = v1
        v3 = v0

        s3 = target_pose

        return t0, t1, t2, t3, s0, s3, a0

    def calc_trajectory(self, target_pose, max_speed, target_acceleration, s0, t0):
        target_pose = float(target_pose)
        max_speed = float(max_speed)
        target_acceleration = float(target_acceleration)
        t0 = float(t0)
        a0 = float(target_acceleration)
        v0 = 0.0
        s0 = float(s0)

        dt1 = self._speed_reached_time(v0, max_speed, target_acceleration)

        v1 = max_speed
        ds1 = self._get_pose(a0, v0, dt1)

        s1 = s0 + ds1
        a1 = 0

        dt3 = self._speed_reached_time(max_speed, 0, -target_acceleration)
        v3 = 0
        ds3 = self._get_pose(-target_acceleration, v1, dt3)
        a3 = 0

        const_speed_distance = target_pose - (ds1 + ds3)

        # check is const_speed_distance > 0

        if const_speed_distance > 0:
            dt2 = const_speed_distance / v1
            v2 = v1
            s2 = const_speed_distance + s1
            a2 = -target_acceleration

            s3 = s2 + ds3

            t1 = t0 + dt1
            t2 = t1 + dt2
            t3 = t2 + dt3

        else:
            dt1 = self._get_time(target_acceleration, v0, target_pose / 2)
            dt2 = 0
            dt3 = + self._get_time(target_acceleration, 0, target_pose / 2)

            t1 = t0 + dt1
            t2 = t1 + dt2
            t3 = t2 + dt3

            v1 = target_acceleration * dt1 + v0
            v2 = v1
            v3 = 0

            s1 = target_pose / 2 + s0
            s2 = s1
            s3 = s1 + target_pose / 2

            a1 = -target_acceleration
            a2 = -target_acceleration
            a3 = 0

        return t0, t1, t2, t3, v0, v1, v2, v3, s0, s1, s2, s3, a0, a1, a2, a3

    def sample_trajectory_new(self, dt, t0, t1, t2, t3, s0, s3, a0, sign):

        def _accel_phase(t, s0, a0):
            return sign * (s0 + 0.5 * a0 * t ** 2)

        def _speed_phase(t, s0, a0, t1):
            return sign * (s0 + a0 * t1 * (t - t1 / 2.0))

        def _brake_phase(t, s3, a0, t3):
            return sign * (s3 - 0.5 * a0 * (t3 - t) ** 2)

        time_array = np.arange(t0, t3, dt)

        accel_array = [_accel_phase(xi, s0, a0) for xi in time_array[:int(t1 / dt)]]
        speed_array = [_speed_phase(xi, s0, a0, t1) for xi in time_array[int(t1 / dt):int(t2 / dt)]]
        brake_array = [_brake_phase(xi, s3, a0, t3) for xi in time_array[int(t2 / dt):]]
        return time_array, np.asarray(accel_array + speed_array + brake_array)

    def sample_trajectory(self, t0, t1, t2, t3, v0, v1, v2, v3, s0, s1, s2, s3, a0, a1, a2, a3, dt):
        poses = []
        speeds = []
        accelerations = []
        times = []

        time = t0
        acceleration = a0
        speed = v0
        pose = s0

        run = True
        while run:
            speeds.append(speed)
            poses.append(pose)
            accelerations.append(acceleration)
            times.append(time)

            if time < t1:
                acceleration = a0
            elif time < t2:
                acceleration = a1
            elif time < t3:
                acceleration = a2
            else:
                acceleration = a3

            if time > t3 + dt:
                run = False

            speed += acceleration * dt
            pose += speed * dt
            time += dt

        return times, poses, speeds, accelerations


if __name__ == "__main__":
    max_speeds = np.deg2rad(np.array([85, 85, 100, 75, 130, 135, 135]))
    max_accels = np.array([5, 4, 5, 5, 25, 12, 25])
    generator = AxisTrajectoryGenerator(max_speeds, max_accels)
    time, poses = generator.genSyncMove([0, 5, 0, 2, 0, 5, 0], [-2, 1, 1, 1, 1, 1, 1], 0.001, 1.0)

    i = 1
    for trajectory in poses:
        plt.subplot(len(poses), 1, i)
        plt.plot(time, trajectory)
        plt.ylabel("axis " + str(i))
        i += 1

    plt.show()
