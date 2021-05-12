import numpy as np
import RobotDART as rd
import dartpy  # OSX breaks if this is imported before RobotDART
# import helpers
from init_tower_disks import init_disks, init_tower
from symbolic import next_move
from utils import damped_pseudoinverse, AdT


class PITask:
    def __init__(self, target, dt, Kp=10., Ki=0.1):
        self._target = target
        self._dt = dt
        self._Kp = Kp
        self._Ki = Ki
        self._sum_error = 0

    def set_target(self, target):
        self._target = target

    # function to compute error
    def error(self, tf):
        # 2 ways of computing rotation error in world frame
        # # 1st way: compute error in body frame
        # error_in_body_frame = rd.math.logMap(tf.rotation().T @ self._target.rotation())
        # # transform it in world frame
        # error_in_world_frame = error_in_body_frame @ tf.rotation().T
        # 2nd way: compute error directly in world frame
        rot_error = rd.math.logMap(self._target.rotation() @ tf.rotation().T)
        lin_error = self._target.translation() - tf.translation()
        return np.r_[rot_error, lin_error]

    def update(self, current):
        error_in_world_frame = self.error(current)

        self._sum_error = self._sum_error + error_in_world_frame * self._dt

        return self._Kp * error_in_world_frame + self._Ki * self._sum_error


def error(tf, tf_desired):
    return np.linalg.norm(rd.math.logMap(tf.inverse().multiply(tf_desired)))


# Create simulator object
dt = 0.004  # we want a timestep of 0.004
simu = rd.RobotDARTSimu(dt)
simu.set_collision_detector("fcl")  # fcl as the collision detector

# Create graphics
gconfig = rd.gui.GraphicsConfiguration(1024, 768)
graphics = rd.gui.Graphics(gconfig)
simu.set_graphics(graphics)
graphics.look_at([2., 0., 1.])

# index of init positions
index = 1

# load tower
packages = [["tower_of_hanoi", "tower_description"]]
tower = rd.Robot("tower.urdf", packages, "tower_base")
tower.set_positions(init_tower(index))
tower.fix_to_world()  # fix to world

# add tower to simulation
simu.add_robot(tower)

# get disk init states
disk_positions = init_disks(index)

current_state = [[], [], []]

# create/load disks
disks = []
for i in range(3):
    disk = rd.Robot("disk" + str(2-i) + ".urdf", packages, "disk" + str(i))
    disk.set_positions(disk_positions[i])
    disks.append(disk)
    # add disk to simulation
    simu.add_robot(disk)

    # creating the current state list representation
    disk_y = disk.body_pose("base_link").matrix()[1][3]
    pole_index = int(round(disk_y * 10) / 2 + 1)
    current_state[pole_index].append(2 - i)

current_move = next_move(current_state)

# load/position Franka
packages = [["franka_description", "franka/franka_description"]]
franka = rd.Robot("franka/franka.urdf", packages, "franka")
franka.fix_to_world()
franka.set_color_mode("material")
franka.set_actuator_types("servo")

# set initial joint positions
positions = franka.positions()
positions[5] = np.pi / 2
franka.set_positions(positions)

simu.add_robot(franka)

# we can add a floor
# simu.add_checkerboard_floor()
simu.add_floor()

# HERE ADDED
Kp = 2.
Ki = 0.01

current_disk_pose = disks[2 - current_move[0]].body_pose("tip")
franka_body_pose = franka.body_pose("panda_hand")
franka_body_pose.set_translation([current_disk_pose.matrix()[0][3], current_disk_pose.matrix()[1][3],
                                  current_disk_pose.matrix()[2][3] + 0.117])

controller = PITask(franka_body_pose, dt, Kp, Ki)

# run simulator
positions = franka.positions()
positions[7] = 0.07
positions[8] = 0.07
franka.set_positions(positions)

finger_counter = 0
# action
# 0 - approach the current handle
# 1 - close fingers to grab the handle/disk
# 2 - raise the disk
# 3 - set target pole
# 4 - move disk
# 5 - release disk, get next state, reset action counter (go to 0)
# 7 - do nothing (the game is over)
action = 0
action_error = [0.001, 0.004, 0.001]
while True:
    if simu.step_world():
        break

    tf = franka.body_pose("panda_hand")

    rrrr = error(tf, franka_body_pose)
    if action % 2 == 0 and rrrr < action_error[action // 2]:
        # stop moving and proceed to the next action
        franka.reset_commands()
        action += 1

    if action == 0 or action == 2 or action == 4:
        vel = controller.update(tf)
        jac = franka.jacobian("panda_hand")  # this is in world frame
        jac_pinv = damped_pseudoinverse(jac)  # np.linalg.pinv(jac) # get pseudo-inverse
        cmd = jac_pinv @ vel
        franka.set_commands(cmd)
    elif action == 1:
        # close fingers
        figs = 250
        if finger_counter < figs:
            positions = franka.positions()

            positions[7] -= 0.00025
            positions[8] -= 0.00025
            franka.set_positions(positions)

            finger_counter += 1
        elif finger_counter == figs:
            action = 2

            # raise target above the pole
            franka_body_pose = franka.body_pose("panda_hand")
            franka_body_pose.set_translation([franka_body_pose.matrix()[0][3], franka_body_pose.matrix()[1][3],
                                              franka_body_pose.matrix()[2][3] + 0.23])

            controller.set_target(franka_body_pose)
    elif action == 3:
        # move target to the next pole
        poles = ["poleA", "poleB", "poleC"]
        moving_disk, next_pole = current_move

        pole_x = tower.body_pose(poles[next_pole]).matrix()[0][3]
        pole_y = tower.body_pose(poles[next_pole]).matrix()[1][3]
        disk_base_x = disks[2 - moving_disk].body_pose("base_link").matrix()[0][3]
        disk_base_y = disks[2 - moving_disk].body_pose("base_link").matrix()[1][3]

        franka_body_pose = franka.body_pose("panda_hand")

        translation_matrix = franka_body_pose.translation()
        translation_matrix[0] += pole_x - disk_base_x
        translation_matrix[1] += pole_y - disk_base_y
        franka_body_pose.set_translation(translation_matrix)

        controller.set_target(franka_body_pose)

        action = 4
    elif action == 5:
        # release
        if finger_counter > 0:
            positions = franka.positions()

            positions[7] += 0.00025
            positions[8] += 0.00025
            franka.set_positions(positions)

            finger_counter -= 1
        # find next state/disk
        elif finger_counter == 0:
            for disks_in_pole in current_state:
                if current_move[0] in disks_in_pole:
                    disks_in_pole.remove(current_move[0])
                    break

            current_state[current_move[1]].append(current_move[0])
            if current_state == [[2, 1, 0], [], []]:
                action = 7
            # else
            else:
                current_move = next_move(current_state)

                current_disk_pose = disks[2 - current_move[0]].body_pose("tip")

                franka_body_pose = franka.body_pose("panda_hand")
                franka_body_pose.set_translation([current_disk_pose.matrix()[0][3], current_disk_pose.matrix()[1][3],
                                                  current_disk_pose.matrix()[2][3] + 0.117])
                controller = PITask(franka_body_pose, dt, Kp, Ki)

                action = 0
