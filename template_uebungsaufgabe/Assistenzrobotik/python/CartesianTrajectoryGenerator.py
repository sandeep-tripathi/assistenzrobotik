#!/usr/bin/env python2
import math

import matplotlib.pyplot as plt
import numpy as np


class CartesianTrajectoryGenerator:
    def __init__(self, max_velocity, max_accel):
        self.max_velocity = max_velocity
        self.max_accel = max_accel

    def genMove(self, current_configuration, target_configuration, dt, speed):
        current_configuration = np.asarray(current_configuration)
        target_configuration = np.asarray(target_configuration)
        delta_move = target_configuration - current_configuration

        distance = np.linalg.norm(delta_move)
        t0, t1, t2, t3, v0, v1, v2, v3, s0, s1, s2, s3, a0, a1, a2, a3 = self.calc_trajectory(distance,
                                                                                              self.max_velocity,
                                                                                              self.max_accel, 0, 0)

        time_array, pose_array = self.sample_trajectory_new(dt, t0, t1, t2, t3, s0, s3, a0, 1)

        pose_array_3d = []
        # apply 1D move to 3d move:
        for pose in pose_array:
            # calculate 3d move proportional to 1d move
            prop = pose / distance
            pose_array_3d.append(current_configuration + delta_move * prop)

        return time_array, pose_array_3d

    def _speed_reached_time(self, speed, target_speed, accel):
        dv = target_speed - speed
        return abs(dv / accel)

    def _get_pose(self, accel, speed, time):
        s = 0.5 * accel * time ** 2 + speed * time
        return s

    def _get_time(self, accel, speed, pose):
        time = (-speed + math.sqrt(speed ** 2 - (2 * accel * -abs(pose)))) / accel
        return time

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
    generator = CartesianTrajectoryGenerator(1, 1)
    time, pose = generator.genMove([1,2,3], [1,2,4], 0.1, 1.0)

    print(len(pose))
    print(len(time))

    i = 1
    for trajectory in poses:
        plt.subplot(len(poses), 1, i)
        plt.plot(time, trajectory)
        plt.ylabel("axis " + str(i))
        i += 1

    plt.show()
