import numpy as np
import RobotDART as rd
import dartpy  # OSX breaks if this is imported before RobotDART
# import helpers
from init_tower_disks import init_disks, init_tower
from symbolic import next_move
from utils import damped_pseudoinverse


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


def create_target_pose(robot, link, translation_vector_offset, base_translation_vector=None):
    target_body_pose = robot.body_pose(link)

    if base_translation_vector is None:
        base_translation_vector = target_body_pose.translation()

    for i in range(len(translation_vector_offset)):
        base_translation_vector[i] += translation_vector_offset[i]

    target_body_pose.set_translation(base_translation_vector)

    return target_body_pose


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
index = 0

# load tower
packages = [["tower_of_hanoi", "tower_description"]]
tower = rd.Robot("urdf/tower.urdf", packages, "tower_base")
tower.set_positions(init_tower(index))
tower.fix_to_world()  # fix to world

# add tower to simulation
simu.add_robot(tower)

# get disk init states
disk_positions = init_disks(index)

# initialize current state's list
current_state = [[], [], []]

# create/load disks
disks = []
for i in range(3):
    disk = rd.Robot("urdf/disk" + str(2-i) + ".urdf", packages, "disk" + str(i))
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

# add franka robot to the simulation
simu.add_robot(franka)

# add a floor
simu.add_floor()

# create and initialize controller
franka_body_pose = franka.body_pose("panda_hand")
Kp = 2.
Ki = 0.01
controller = PITask(franka_body_pose, dt, Kp, Ki)

# start with the fingers open
positions = franka.positions()
positions[7] = 0.07
positions[8] = 0.07
franka.set_positions(positions)

finger_counter = 0
# action
# 0-1 - go above the next handle to grab it
# 2-3 - approach the current handle
# 4-5 - close fingers to grab the handle/disk and raise the disk
# 6 - set target pole
# 7 - move disk
# 8 - release disk, get next state, go a bit higher than the next disk, reset action counter (go to 0)
# 10 - do nothing (the game is over)

# actions 1,3,5,7 are moving the whole robot
action = 0
# how far from the absolute coordinates we allow it to be
action_error = {1: 0.0035, 3: 0.00285, 5: 0.00245, 7: 0.0012}
# run simulator
while True:
    if simu.step_world():
        break

    # get the body pose of the franka hand in each iteration
    tf = franka.body_pose("panda_hand")
    # get the error of the current position to the desired position
    current_action_error = error(tf, franka_body_pose)

    # if moving and we've gotten close enough
    if action % 2 == 1 and current_action_error < action_error[action]:
        # stop moving and proceed to the next action
        franka.reset_commands()
        action += 1

    if action % 2 == 1:  # if still moving and not close enough (current_action_error >= action_error[action // 2])
        vel = controller.update(tf)
        jac = franka.jacobian("panda_hand")  # this is in world frame
        jac_pinv = damped_pseudoinverse(jac)  # np.linalg.pinv(jac) # get pseudo-inverse
        cmd = jac_pinv @ vel
        franka.set_commands(cmd)
    elif action == 0:
        current_disk_pose = disks[2 - current_move[0]].body_pose("tip")
        # get directly above the handle to grab it in next step
        franka_body_pose = create_target_pose(franka, "panda_hand", [0, 0, 0.23],
                                              [current_disk_pose.matrix()[i][3] for i in range(3)])
        controller.set_target(franka_body_pose)

        action = 1
    elif action == 2:
        current_disk_pose = disks[2 - current_move[0]].body_pose("tip")
        # get directly above the handle to grab it in next step
        franka_body_pose = create_target_pose(franka, "panda_hand", [0, 0, 0.115],
                                              [current_disk_pose.matrix()[i][3] for i in range(3)])
        controller.set_target(franka_body_pose)

        action = 3
    elif action == 4:
        # close fingers
        finger_steps = 122
        if finger_counter < finger_steps:  # while still closing fingers
            positions = franka.positions()

            positions[7] -= 0.0005
            positions[8] -= 0.0005
            franka.set_positions(positions)

            finger_counter += 1
        elif finger_counter == finger_steps:  # when fingers fully close
            # set the target of the handle to a bit more than the height of the tower
            franka_body_pose = create_target_pose(franka, "panda_hand", [0, 0, 0.24])
            controller.set_target(franka_body_pose)

            action = 5  # raise target above the pole in next iteration
    elif action == 6:
        # move target to the next pole
        poles = ["poleA", "poleB", "poleC"]
        moving_disk, next_pole = current_move

        pole_x = tower.body_pose(poles[next_pole]).matrix()[0][3]
        pole_y = tower.body_pose(poles[next_pole]).matrix()[1][3]
        disk_base_x = disks[2 - moving_disk].body_pose("base_link").matrix()[0][3]
        disk_base_y = disks[2 - moving_disk].body_pose("base_link").matrix()[1][3]

        franka_body_pose = create_target_pose(franka, "panda_hand", [pole_x - disk_base_x, pole_y - disk_base_y, 0])
        controller.set_target(franka_body_pose)

        action = 7
    elif action == 8:
        # 1. release
        if finger_counter > 0:
            positions = franka.positions()

            positions[7] += 0.0005
            positions[8] += 0.0005
            franka.set_positions(positions)

            finger_counter -= 1
        # 2. find next state/disk
        elif finger_counter == 0:
            # remove the disk we just changed position from the current state's towers
            for disks_in_pole in current_state:
                if current_move[0] in disks_in_pole:
                    disks_in_pole.remove(current_move[0])
                    break

            # update the current_state by adding the disk pointed by the current_move, 
            # to the tower pointed by the current_move            
            current_state[current_move[1]].append(current_move[0])

            if current_state == [[2, 1, 0], [], []]:  # if we have reached the final state
                action = 10  # end
            # if there are still moves to do move above the next disk to avoid collision from diagonal movement
            else:
                # get the next move
                current_move = next_move(current_state)
                # get the pose of the disk that we need to move now
                current_disk_pose = disks[2 - current_move[0]].body_pose("tip")

                franka_body_pose = create_target_pose(franka, "panda_hand", [0, 0, 0.23],
                                                      [current_disk_pose.matrix()[i][3] for i in range(3)])
                controller.set_target(franka_body_pose)

                action = 0
