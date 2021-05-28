<!-- PROJECT -->
## Hanoi Tower in RobotDART


<!-- ABOUT THE PROJECT -->
## About The Project

This project is a python implementation of solving the famous problem of Hanoi's Tower in the RobotDART aenvironment.

It is developed as a semester project for the course of Smart System Technologies and Robotics of the Computer Engineering and Informatics Department, University of Patras.
The supervisor was Konstantinos Hatzilygeroudis, a core contributor to the RobotDART library.



# Getting Started

The files `symbolic.py`, `utils.py`, `init_tower_disks.py` as well as all the urdf files, were given by [Constantinos Hatzilygeroudis](https://github.com/costashatz) . 

Here will be analyzed the code in the file `runme.py`, as it is the only part that was developed by the contributors of this repo.

## runme.py

### class PITask

This class contains the necessary methods for a controller that will move the robot.

**def error** computes the error, meaning the absolute distance of the robot from the desired end state.

I can't thoroughly explain the rest. These were given. 


The robot used is Franka, a standard robot urdf that is contained in the RobotDART installation.

### Main Code


`index = 0,1,2,3+`
The index variable indicates one of the four starting states, given by the file symbolic, when spawning the tower and the disks. 
It can take the values 0,1,2 or 3 and above. 

The urdf files for the tower and the disks are given in the repository. 
The disks have a handle that makes the process easier. 

### def create_target_pose

This is an auxilliary function that cleans up the main code when creating the target body pose is needed. 
***

The main part is the actions that are controlled by good ol' if statements. They are the following: 

0. Approach the handle that needs to be moved next
1. Close fingers to grab the handle/disk
2. Raise the disk
3. Set target pole
4. Move disk directly above the target pole
5. Release the disk, get the next state, go a bit higher than the next disk, reset action counter (go to 0)
7. Do nothing (game over)
9. Go above the next handle to grab it.


**Actions 0,2 and 4** are actions where the whole robot is moving. For each of the three movements, there is a corresponding acceptable error threshold in the `action_error` list.

**Action 1**
In action 1, where Franka's fingers close, it happens gradually in a 250 iteration fashion (it could be faster, we just chose this speed) instead of all at once: 

```
positions[7] -= 0.0625
positions[8] -= 0.0625
franka.set_positions(positions)
```
This happens in order to avoid any potential force that will move the handle out of balance. 

**Action 5** 
In action 5 where the fingers of Franka's handle open, we tried a faster pace of moving the fingers, but it seems to interfere with the error of the next movement, so it should stay at 250 steps. 


After opening the fingers, it first goes diagonally towards the next handle, but stops a bit higher, so that there is only a vertical movement left to go grab the handle. This is a work-around to avoid collisions between handles and the robot. 

When it has reached this state, it goes to the special action 9. 

**Action 9**

Here, the body pose is the same as in action 5, just a bit lower on the Z axis, therefore it only moves vertically. 



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contributors - Contact Info

Nikolaos Maragkos

Leonidas Karagiannis - [@LinkedIn](https://www.linkedin.com/in/leonidas-karagiannis-4304a8171/) - leonidask914@gmail.com


Project Link: [https://github.com/github_username/repo_name](https://github.com/github_username/repo_name)


