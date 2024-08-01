# GROUP BESTeam : MRS Summer School 2024: multi-robot inspection and monitoring

# Team members:

1. Boris KIEMA
2. Marco Luna
3. Amath Sow

# Tast: 
In this Summer School task, we will focus on the cooperation of a group of two UAVs (Unmanned Aerial Vehicles) in a 3D environment with obstacles.
The task is to plan collision-free trajectories of the UAVs so that cameras onboard the UAVs inspect a set of *N* unique inspection points.
Both UAVs have a predefined starting position and a limit on maximal velocity and acceleration.
The objective of the task is to minimize the time of inspection while capturing all the inspection points and not colliding the UAVs with the environment or with each other.
An already working solution is provided as a part of the assignment.
However, this example solution has poor performance and can be improved significantly.

## Installation

The Summer School 2024 will use the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) contained in a [Apptainer](https://apptainer.org/) image (previously called Singularity).
A set of scripts is provided to create a layer of abstraction above the Apptainer system, so the participants only need to know how to call a shell script, e.g.,
```bash
./script.sh
```
The following steps will download the main repository, install Apptainer (only on Ubuntu-compatible OS), and download the pre-built Apptainer image.
No further changes are made to the host operating system.

Compatible platforms:

* Linux OS,
* Windows 11 + WSL 2.0 with Linux OS,
* Windows with virtualized Linux OS,
* Mac OS X with virtualized Linux OS.

For a non-Ubuntu Linux OS, please, install Apptainer on your own.

Requirements:

* [Apptainer](https://apptainer.org/) on Linux OS,
* approx. 6 GB of disk space.

### Installation procedure

1. If you are a `git` veteran, you would think about `fork`-ing the repository, but because you are a veteran, you will know that a fork of a public repository cannot be made private, and you don't want your team's solutions to be public. So, we recommend to tap the plus sign in the top right corner, and then select the Import repository option. Add `https://github.com/ctu-mrs/summer-school-2024.git` as the link and while completing this form/page, you will find an option to make your new repository private.

2. Clone your new repository to, e.g., `~/git`:
```bash
mkdir -p ${HOME}/git
cd ${HOME}/git && git clone <your new repository's link>
```

3. Run the installation script that will install dependencies, download the MRS Apptainer image containing [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system), and compile the workspace:
```bash
cd ${HOME}/git/summer-school-2024 && ./install.sh
```

## Task overview

You are given two UAVs (Red 🟥 and Blue 🟦) required to inspect a set of **inspection points (IPs)** as fast as possible in a 3D environment with obstacles.
The two UAVs are equipped with the [MRS control pipeline](https://github.com/ctu-mrs/uav_core) [1], allowing precise trajectory tracking.
Your task is to assign the IPs to the UAVs and to generate multi-goal paths visiting **viewpoints (VPs)** (poses in which the particular IPs are inspected with onboard cameras) of each IP while keeping a safe distance from the obstacles and between the two UAVs.
Furthermore, you shall convert paths to collision-free time-parametrized trajectories that respect the UAVs' dynamic [constraints](#constraints).
The IPs are defined by their position and inspection angle and are divided into three subsets:

1. 🔴 red locations: inspectable by 🟥 UAV only,
2. 🔵 blue locations: inspectable by 🟦 UAV only,
3. 🟣 purple locations: inspectable by both (🟥 or 🟦)  UAVs.

![TASK](.fig/task.png)

To inspect an IP, you have to visit its attached VP with a correct UAV within a radius of 0.3 m and with a maximum deviation in inspection heading and pitch of 0.2 rad.
**Each successfully inspected point increments your score by 1.**
The overall objective is to maximize the score while minimizing the flight time of both UAVs.

The trajectories are required to begin and end at predefined starting locations.
The mission starts when the trajectory following is started and ends once the UAVs stop at their starting locations.
The motion blur effect during imaging is neglected; thus, the UAVs are not required to stop at particular VPs.

## Our Solution
1. We use k-means for clustering, we have tested with density based clustering OPTICS but it's very hyperparameters sensitive, so we need to do grid search or bayesian optimization.
2. We implemente the heading interpolation between samples 
3. We use A* for planning  with euclidean heuristic and add also the path straithtening with halveAndTest function
4. We apply path smoothing and continuous trajectory sampling (no stops at waypoints) to speed up the flight with toppra.
5. We implemented delay_till_no_collisions_occur for collision avoidance
6. Finally, we did some hyperparameters tuning

### Constraints

Your solution to both the challenges has to conform to constraints summarized in the following table:

| Constraint                                          | Virtual challenge | Real-world challenge |
| :---                                                | :---:             | :---:                |
| Maximum solution time (soft) - $T_s$                | 80 s              | 80 s                 |
| Maximum solution time (hard)                        | 200 s             | 150 s                 |
| Maximum mission time                                | 200 s             | 240 s                |
| Maximum velocity per x and y axes                   | 5 m/s             | 3 m/s                |
| Maximum velocity in z axis                          | 2 m/s             | 1 m/s                |
| Maximum acceleration per x and y axes               | 5 m/s^2           | 3 m/s^2              |
| Maximum acceleration in z axis                      | 2 m/s^2           | 1 m/s^2              |
| Maximum heading rate                                | 1 rad/s           | 1 rad/s              |
| Maximum heading acceleration                        | 2 rad/s^2         | 2 rad/s^2            |
| Minimum obstacle distance                           | 1.5 m             | 2.0 m                |
| Minimum mutual distance                             | 2.5 m             | 4.0 m                |
| Dist. from starting position to stop the mission:\* | 1.0 m             | 1.0 m                |

\* The last point of the trajectory is expected to match the starting point with up to 1 m tolerance.


## Run your code

A set of scripts is provided in `simulation/`, allowing you to start and stop the simulation and evaluate your code.
The **bold** scripts are expected to be used directly by the user.

| Script                 | Description                                                                       |
| :---                   | :---:                                                                             |
| **pycharm.sh**         | runs PyCharm inside Apptainer                                                     |
| **run_offline.sh**     | runs your solution using a lightweight _replay_                                   |
| **run_simulation.sh**  | runs your solution inside a realtime dynamics simulation                          |
| **kill_simulation.sh** | kills the running simulation environment                                          |
| 01_install.sh          | install the Apptainer software (called within `install.sh`)                       |
| 02_download.sh         | downloads the Apptainer image (called within `install.sh`)                        |
| 03_compile.sh          | compiles the user's software (called within `install.sh`)                         |
| apptainer.sh           | entry point to the Apptainer's shell (only run if you intend to program in `vim`) |

**1) Offline: lightweight without simulating UAV flight**

We recommend starting _offline_ (without using the dynamics simulator) when approaching the task for the first time.
The script below will run a solution to the task while showing the problem and the trajectories.
```bash
./simulation/run_offline.sh
```

After running the `run_offline.sh` script, you should see a similar visualization once the trajectory generation process is completed.
The RViz (ROS visualization) shows an **example solution** to the task.

![GIF](.fig/rviz_visualization.gif)

The RViz window contains:

* Start/pause button in the left bottom corner. **(Use 'Send Topic' button and not the Rviz 'Pause' button)**
* overall trajectories information in the top left/right corners (background is green if every check is OK, red otherwise)
* current flight statistics right below
* information about the mission and the score centered in the top
* lines intersecting both paths which indicate collisions.

**2) Online: run simulation locally**

The script below will execute your solution to the task alongside the *MRS Multirotor Simulator* and the [MRS UAV system](https://github.com/ctu-mrs/mrs_uav_system) [1] simulating two UAVs.
```bash
./simulation/run_simulation.sh
```
Stopping the simulation is done by calling
```bash
./simulation/kill_simulation.sh
```
**Things to configure/change :**

* **Problem Type:** By default, the `run_simulation.sh` spawns you 2 UAVs in the `warehouse` world and launches the *warehouse_small* problem.
To change the problem type to `warehouse_large` or `warehouse_yard`, you have to

  * change the parameter `problem/name` in the `mrim_task/mrim_planner/config/virtual.yaml` to `warehouse_large.problem` or `warehouse_yard.problem` (see section [Testing](https://github.com/ctu-mrs/summer-school-2024#testing))

You may notice that your terminal opened multiple tabs.
Check the first page of the [MRS Cheatsheet](https://github.com/ctu-mrs/mrs_cheatsheet) if you need help navigating the tabs and panes.

The terminal window will contain the interface of the Tmux: the terminal multiplexer.
It allows us to execute multiple commands in multiple terminals in one terminal window.
The Tmux window will contain "tabs" (panes), which are listed at the bottom of the window.
Switching between the tabs is done by the key combinations **shift→** and **shift←**.
The important tabs are listed below:

| Tab            | Description                                                                    |
| :---           | :---:                                                                          |
| planner        | the output of your planner                                                     |
| state_machine  | this node queries the planners for trajectories and handles the experiment     |
| start_planning | here, a command is prepared in the shell's history to start the planning again |
| control        | The MRS UAV System control pipeline                                            |

**3) Online: prepare for real-world experiments**

The preparation for a real-world experiment does not require any actions on your side.
You are required only to provide functional code for trajectory planning contained in the `mrim_planner`.
If you created other ROS nodes, which shall be run separately to the `mrim_planner`, include their launching in `mrim_planner/launch/planner.launch`.

## Problem sets - Testing

You have two problems prepared for testing and evaluating your solution.
The problems are located in `mrim_resources/problems`: you can switch between them by changing the `problem/name` line in `mrim_planner/config/virtual.yaml` to:

  1. `warehouse_small.problem` is a simple problem with fewer IPs, good for clustering, improving TSP sequences, parametrizing the solution, and testing
  2. `warehouse_large.problem` is a complex problem with 31 IPs that will test your solution in full
  3. `warehouse_yard.problem` is a complex problem with 32 IPs that will test your solution in full, good for final tuning of parameters (**a similar problem will be used in the competitions** described below)

## Competition

There will be two competitions:

  1. In the virtual environment, and
  2. in the real world.

To participate in the competitions, you must send your solution in a single archive file by email until Thursday, August 1, 11:59 p.m.
Please, **email us** the code to `prochon4@fel.cvut.cz` with the subject **SUMMER SCHOOL TASK** till Thursday, August 1, 11:59 p.m., zipped with your *Team's name* as:
```bash
zip -r my_team_name.zip mrim_planner
```
The submitted archive has to contain the whole package `mrim_planner`, including two config files (`real_world.yaml` and `virtual.yaml`) in the folder `mrim_planner/config`. **Please don't forget to modify the parameters in `real_world.yaml` according to your findings/configuration in virtual worlds.**
Please include the names of all team members in the email message.
**The late submissions will not be accepted for the competition**.

The evaluation of particular solutions in the real-world challenge will be performed on Friday, August 2nd, with the real-time score presentation.
The virtual challenge will be evaluated on Friday.
The results will be presented during an awards ceremony organized at the experimental site after the real-world challenge.
**The final score of the solution equals the sum of successfully inspected IPs.**

**Reasons to assign zero score (and thus to disqualify the solution):**

  1. violation of assigned dynamic constraints of UAVs (**in horizontal and vertical directions only**; violation of constraints on heading does not affect the score but beware that the heading rate/acceleration of the UAV controller will be limited by these constraints),
  2. violation of minimum allowed distance between obstacles and UAVs,
  3. violation of minimum allowed mutual distance between UAVs,
  4. violation of maximum distance of final trajectory point to the predefined starting location,
  5. exceeding the hard maximum available time for computing a solution (see the [constraints](#constraints) table).

In case of a tie, **secondary key** to determine the finishing order of the participating teams is given as $T_I + T_P$ (in seconds), where $T_I$ is the inspection time (start to end of both trajectories) and $T_P = max(0, T_C - T_s)$ is the time $T_C$ it took to compute the solution minus the soft limit $T_s$ for computing the solution (see the [constraints](#constraints) table).

### Virtual

The dimensions of the virtual environment and inspection problem will be similar to `warehouse_yard.problem`.
Your solution for the virtual environment has to conform to constraints summarized in the table above.

### Real-world

The dimensions of the real-world environment and inspection problem will be similar to `warehouse_yard.problem` but with significantly less obstacles.
The same code as the virtual challenge will be run onboard real UAVs during the real-world challenge.
No changes are required on your side.
However, note that the evaluation of inspected points will be based on the actual pose of the UAV in the world, not the reference trajectories.
Hence, the effect of trajectory tracking will not be negligible, and you should consider the challenges of the real-world environment.
Consider the challenges during parametrization and prepare your solution for deviations from the ideal conditions. E.g., introduce reserves for UAV-to-UAV and UAV-to-obstacles distances to prevent unfortunate zeroing of your score or lower the magnitude of allowed deviations from the reference trajectory.

## Explore another possible usage of the MRS UAV System

Based on the presentation of the MRS system, you can also try other capabilities of the system.
You selected a group of practicals based on your scientific interest.
Feel free to ask during the summer school and especially during the seminars how the system can be used for your area of interest.

## Troubleshooting

**Common error you might observe**
```
open terminal failed: missing or unsuitable terminal: rxvt-unicode-256color
```

*solution*: run the `./kill_simulation.sh` script.

**Updating the repository**

If there is an update in the repository, you can pull it to your local machine using git:
```
cd ${HOME}/git/summer-school-2024 && git pull
```

**Google**

Before asking for help, try to come up with the answer on your own or with the assistance of a Google search or ChatGPT.
Sometimes just writing the question down helps you to understand the problem.

**Contacts**

If you find a bug in the task, you need assistance, or you have any other questions, please contact by email one of (or all of):

* Ondřej Procházka `prochon4@fel.cvut.cz`
* Matej Novosad `novosma2@fel.cvut.cz`

We will try to help you as soon as possible.

## Disclaimer

During the week of the 2024 MRS Summer School, the organizers reserve the right to:

* to do fixes: to update the task in case of finding severe bugs in the code,
* to maintain fairness: to change the problems or the constraints for the challenges,
* to preserve safety: to discard provided trajectories for the real-world challenge if the flight would be unsafe in any possible way.

## References

* [1]  Baca, T., Petrlik, M., Vrba, M., Spurny, V., Penicka, R., Hert, D., and Saska, M., [The MRS UAV System: Pushing the Frontiers of Reproducible Research, Real-world Deployment, and Education with Autonomous Unmanned Aerial Vehicles](https://arxiv.org/pdf/2008.08050), _Journal of Intelligent & Robotic Systems 102(26):1–28, May 2021_, GitHub: https://github.com/ctu-mrs/mrs_uav_system.
* [2]  H. Pham, Q. C. Pham, [A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis](https://www.researchgate.net/publication/318671280_A_New_Approach_to_Time-Optimal_Path_Parameterization_Based_on_Reachability_Analysis), [Documentation](https://hungpham2511.github.io/toppra/index.html)
