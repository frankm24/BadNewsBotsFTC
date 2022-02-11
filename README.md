## Bad News Bots
This repo currently contains an FTCRobotController build with our code from the **Ultimate Goal 2020-2021 and upcoming 2021-2022 seasons**. For organizational purposes, we may move the old stuff to a different project.

## Update from Frank: 
I will soon (mid-winter or spring break?) be making a video on how to code FTC op modes and go into detail on how it all works. I'm hoping that next year there will be more interest in robotics and people will be able to pick up the coding quickly.

Here are some notes of things that are on my mind at the moment:

-Devin is doing an independent study on Fusion 360. We can eventually model our prototype robot in that software and run physics simulations on it to ensure it has a centered center of mass.
-We need to think about 3D printing side panels and other parts for the robot more frequently. 
-After the state tournament, we need to work on designing a chassis with the goal of ease of assembly and disassembly, ease of cable management, integration of odometry wheels ("dead wheels"), and structural rigidity/strength. 
-We should definetly make a fun show-off/experimental chassis with monster truck wheels like Macy started to work on. This would be great for impressing people and recruiting and can be used for computer vision experiments and driving tests.
-Computer vision: More specifically, I would like to try using two cameras to create a disparity map, which estimates the distance away from the robot for each pixel of the image using trigonometry. It is also possible to use opencv canny and other functions to try to map out corners of walls, and where the walls and the floor touch, but none of these methods will work well without the disparity map. Without it, there is no basis on which to estimate distance. 
(Deep neural nets would be best, but that is what companies with access to large datasets and computing power beyond the REV Control Hub would do.)

## My final goals for the off-season:
-Use vision and odometry to make a system which can navigate autonomously beyond the robotics field. It would be great if it was able to drive itself around an open area in the school without running into walls, humans/moving objects, or static objects. This is extremely difficult, but possible, and I am going to attempt it. 
-Design an innovative chassis in Fusion 360 which meets the aforementioned requirements and build it.

