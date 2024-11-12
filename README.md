# Magnetic Pheromone Follower Robot

Welcome to the **Magnetic-Pheromone-Follower-Robot** repository! This project is part of the PROE research initiative at the Costa Rica Institute of Technology. It extends the functionality of the Atta mobile robots—previously developed and powered by Feather M0 and STM32 microcontroller.

## Introduction

Imagine a robot navigating the outdoors, not by GPS or typical sensors, but by detecting tiny ferrofluid drops left as a trail. This project achieves that by using magnetometers on the front of the robot, enabling it to sense these "pheromone" markers on the ground. This approach bypasses traditional localization challenges, especially those of conventional odometry, which can be inaccurate outdoors.

<img src="https://github.com/user-attachments/assets/d8432523-a157-4f4d-b8d2-a0f013c6618c" width="400px" alt="intro">

### System Overview

#### Arc-Shaped Movement

To detect ferrofluid drops, the robot performs an arc-shaped sweep with its front sensors, moving like a seeker honing in on a hidden target. This arc movement is controlled by precise position feedback from the odometry system, allowing the robot to scan an area effectively for magnetic markers. 

<img src="https://github.com/user-attachments/assets/d1413afa-a714-4946-a986-31d5a0ce0072" width="300px" alt="move_gif">

- Look at these moves!


We carefully tuned the parameters of the navigation system, as well as the distance between the drops, to ensure consistent and accurate detection during these sweeps.

<img src="https://github.com/user-attachments/assets/bd6c3735-f213-42b7-9a83-987c3bc29254" width="200px" alt="av_gif">


#### Detection 

To enhance detection reliability, we also tuned the filtering parameters of the magnetometer readings. A low-pass filter is used to reduce random noise, while a high-pass filter eliminates gradual, low-frequency changes in the environmental magnetic field. This ensures the magnetometers primarily respond to ferrofluid drops, ignoring unrelated magnetic shifts.

Check out the detection of a pheromone!

<img src="https://github.com/user-attachments/assets/674bb6be-70cc-4dd1-9083-33e7f67da246" width="400px" alt="detect_gif">

 - There's something here!


Note: The filter design was carried out by our supervisor Juan Carlos Brenes Torres

### Path Following

Following a trail isn’t easy—it’s like a treasure hunt in slow motion! We equipped the robot with two magnetometers that allow it to "bounce" between signals on either side, just like ants do when following a pheromone trail. After some trial and error, the robot learns which way to go, navigating its path as if lured by an invisible cookie!

<img src="https://github.com/user-attachments/assets/f112c751-5fc2-4599-8f9d-767c642348ca" width="400px" alt="follow_gif">


### Room for Growth

To enhance this behavior further, we are working toward:
- **Better detection sensitivity** by refining filter settings.
- **Optimized sensor spacing** to create a seamless "bouncing" effect.
- **Enhanced navigation logic** for more robust trail following.

## Latest Publication

Curious to dive into the science and results? Check out our Lab's latest publication for detailed insights:
- [Published Paper](https://iopscience.iop.org/article/10.1088/1748-3190/ad8d28)



Are you intrigued? Feel free to explore the code, experiment with different configurations, and join us in developing this unique navigation method. Let’s make robots follow trails like ants!
