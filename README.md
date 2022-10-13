![Source Build](https://github.com/gstavrinos/ros2_kart_racing/actions/workflows/source-build.yml/badge.svg)

# ros2_kart_racing
ROS2 packages for simulated kart racing

<img src="https://raw.githubusercontent.com/gstavrinos/ros2_kart_racing/master/media/kart.gif">

<img src="https://raw.githubusercontent.com/gstavrinos/ros2_kart_racing/master/media/kart_closeup.png">

# Info
The `master` branch includes all the required packages to simulate the kart, along with some tracks. The `leaderboards` branch includes some markdown files with the best times of all users that have participated.

In order to participate in the `leaderboards`, you will need to use the template from [the kart_navigation-template repo](https://github.com/gstavrinos/kart_navigation-template). You will find more info on the template in its repo. In short, the `kart_navigation-template` repo includes a ROS2 package with all the expected files. You just need to integrate your own stuff inside its files, and you are good to go. It's also based on the [ROS2 Kart Racing Entry action](https://github.com/gstavrinos/ros2_kart_racing_action)

# Tracks
Currently there are two tracks included in the `race_tracks` package. They are based on real-world tracks, and their satellite image can be seen below.

### [kartland](https://kartland.gr):
<img src="https://raw.githubusercontent.com/gstavrinos/ros2_kart_racing/master/media/kartland.png">


### [speedpark](https://www.speedpark.gr):
<img src="https://raw.githubusercontent.com/gstavrinos/ros2_kart_racing/master/media/speedpark.png">

# Using the repo
The simplest way to use the packages in this repo is to use the complementary [ros-ez-docker-gui repo](https://github.com/gstavrinos/ros-ez-docker-gui). This package aims to simplify the process of installing and using ros into a single command. Keep in mind that it is compatible only with Linux. For example, in order to run a "ready-for-leaderboards" implementation you would simply run `ros2ez fi ros2 launch kart_simulation kart_speedpark_simulation.launch.py competition_mode:=true gui:=true`. Note that `gui` is set to `true`, assuming that you are running it on a local machine. More info on that repo.

Another way to use this repo is to clone it in your workspace and follow the normal process of dealing with ROS2 packages.

Finally, you could entirely base your experiments on github, without any local development. The `kart_navigation-template` includes an action that runs newly pushed code on the `competition` branch against all tracks. It is recommended to not use this method, since even though is the most hassle-free in terms of handling with software stacks, it is also the most time consuming, requiring "blind" testing entirely through github actions.

