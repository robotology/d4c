### Theoretical Background
The **Dynamic Force Field Controller** framework - DForC or D4C in short - is
based on the idea of employing virtual force field in order to perform a
reaching task. It takes inspiration from theories conceived by O. Khatib durig
80's (Oussama Khatib, **Real-time obstacle avoidance for manipulators and mobile
robots**, Int. J. Rob. Res., Vol. 5, No. 1. (1986), pp. 90-98) and revised by
P. Morasso and V. Mohan within the Passive Motion Paradigm (PMP) (Mohan. V.,
Morasso P.,Metta G., Sandini G. **A biomimetic, force-field based computational
model for motion planning and bimanual coordination in humanoid robots**,
Autonomous Robots, Volume 27, Issue 3, (2009) pp. 291-301). The novelty of the
D4C approach chiefly consists in replacing the use of the transposed Jacobian
(as proposed in PMP) with the [Cartesian Interface](http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html)
tool. Thereby, D4C automatically takes into account the robot's joints limits
and it makes the generated trajectory smooth. The D4C library allows describing
objects as obstacles or as targets, and to consequently build a virtual force
field to reach the target avoiding obstacles.

For the target a mass-spring-damper force field is generated:

![](https://github.com/robotology/d4c/blob/master/img/MassSpringDump.jpg)

Each obstacle is modeled as a Gaussian repulsive force field. It is possible to
choose between a canonical multivariate Gaussian force field:

![](https://github.com/robotology/d4c/blob/master/img/GaussianForceField.jpg)

and a Gaussian force field without tails:

![](https://github.com/robotology/d4c/blob/master/img/GaussianWithoutTails.jpg)

Since the tuning of the parameters is a critical point, we’re going to implement
some further methods in order to avoid local minima.


### MATLAB modeling
Many tests with the following Simulink model have been performed.
![](https://github.com/robotology/d4c/blob/master/img/SimulinkModel.jpg)

It allows you to modify parameters and to compute easily the virtual trajectory
generated from the combination of the virtual force fields associated to target
and obstacles. It is also possible to add a Gaussian attractor on the target
which will be summed to the mass-spring-damper force field.
![](https://github.com/robotology/d4c/blob/master/img/WithAndWithoutTails.jpg)
Obstacles modeled as Gaussians with tails in the first image, and without tails
in the second.

As it is possible to notice from the figure, local minima can be reached.
These situations can be avoided with the method explained in "_Randazzo M.,
Sgorbissa A. and Zaccaria R. µNav: Navigation without localization_".
![](https://github.com/robotology/d4c/blob/master/img/LocalMinima.jpg)
In the first case the trajectory stops in a local minima, in the second case
the trajectory bypass the obstacles correctly.

It is possible to download the two MATLAB models (the one without methods for
local minima, and the other one with methods for local minima) from [here](http://wiki.icub.org/images/8/83/PmpModels.zip).

### The Cartesian I/F Implementation
The D4C library exploits the [Cartesian Interface](http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html),
and it is built as follows:
<p align="center">
  <img src="https://github.com/robotology/d4c/blob/master/img/D4C_architecture.jpg"/>
</p>

There is a server, with the aim to instantiate, modify or delete objects, generate
force fields and compute the trajectory, which is transmitted to the Cartesian Interface.
Then there is a client which is automatically connected to the server and that
can send a request in order to have information on objects, or can ask to add or
delete objects.

Notably, the D4C server is capable of sending proper information
to the [iCubGui](http://wiki.icub.org/iCub/main/dox/html/group__icub__gui.html)
in order to display a pictorial representation of the set of targets, obstacles
and generated trajectories as depicted below.
<p align="center">
  <img src="https://github.com/robotology/d4c/blob/master/img/icubgui.jpg"/>
</p>


### Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)


### Code Snippets and Documentation
In order to use the D4C Library it is required to add the Cartesian Interface
to the _CMakeLists.txt_ file. It is possible to do it by following the Cartesian
Interface [tutorial](http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html).
It is also required to include in the _CMakeLists.txt_ the following line:

```CMakeLists
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} d4c)
```

In order to open a `d4c_server`:
```cpp
...
#include <iCub/d4c/d4c_server.h>
...
D4CServer server;
Property options;
options.put("verbosity",0); //print log messages
options.put("period",20); //period of the RateThread
options.put("device","cartesiancontrollerclient");
options.put("name","d4c_server");
options.put("robot","icub");
options.put("part","right_arm");
server.open(options);
```

In order to open a `d4c_client`:
```cpp
...
#include <iCub/d4c/d4c_client.h>
...
D4CClient client;
Property options;
options.put("verbosity",0);
options.put("remote","/d4c_server");
options.put("local","/d4c_client");
client.open(options);
```

In order to add an obstacle:
```cpp
Value centerOb; centerOb.fromString(("(0.2 0.2 0.2)");
Value radiusOb; radiusOb.fromString("(0.05 0.1 0.05)");
Property obstacleOpt;
obstacleOpt.put("type","obstacle_gaussian");
obstacleOpt.put("active","on");
obstacleOpt.put("G",5.0);
obstacleOpt.put("name","obstacle");
obstacleOpt.put("center",centerOb);
obstacleOpt.put("radius",radiusOb);
int obstacle; client.addItem(obstacleOpt,obstacle);
```

In order to add a target:
```cpp
Value centerTg; centerTg.fromString(("(0.4 0.4 0.4)");
Value radiusTg; radiusTg.fromString("(0.05 0.05 0.05)");
Property targetOpt;
targetOpt.put("type","target_msd");
targetOpt.put("active","on");
targetOpt.put("K",1.5);
targetOpt.put("D",3.0);
targetOpt.put("name","target");
targetOpt.put("center",centerTg);
targetOpt.put("radius",radiusTg);
int target; client.addItem(targetOpt,target);
```

In order to make the trajectory start from a specific point with a given initial
velocity it is possible to use the method
```cpp
client.setPointState(x,o,xdot,odot);
```
where `x` represents the position and `o` the orientation, so as `xdot` and
`odot` the corresponding velocities.

The user might want to enable the force field generation without enabling robot movements:
```cpp
client.enableField();
```

In order to enable also robot movements:
```cpp
client.enableControl();
```

For online documentation of all the methods composing the D4C Library,
please refer to the [Doxygen Documentation](http://robotology.github.com/d4c).


### Tutorials
In [`modules/d4cServer`](https://github.com/robotology/d4c/blob/master/modules/d4cServer/main.cpp)
and [`modules/d4cExample`](https://github.com/robotology/d4c/blob/master/modules/d4cExample/main.cpp)
the user will find further info on how to open a `d4c_server` component and a
`d4c_client` component, respectively, and how to write the corresponding
_CMakeLists.txt_.


### Dissemination
Gori I., Pattacini U., Nori F., Metta G. & Sandini G., [DForC: a Real-Time Method for Reaching,
Tracking and Obstacle Avoidance in Humanoid Robots](http://dx.doi.org/10.1109/HUMANOIDS.2012.6651573),
IEEE-RAS International Conference on Humanoid Robots, Osaka, Japan, November 29 - December 1, 2012.


### Videos
- [EFAA 1st Year Review Demo](http://www.youtube.com/watch?v=npBugYmf59U): the
  library at work within the EFAA EU project.
- [Humanoids2012 Video](http://www.youtube.com/watch?v=QR30jnW_bvY): the official
  DForC video recorded for Humanoids2012 conference.
