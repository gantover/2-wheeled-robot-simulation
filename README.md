# 2-wheeled-robot-simulation
A cinematic simulation involving a 2 wheeled robot to determine optimal value for its PID (P in this case)

### Input

- geometry of the vehicle (defined for the following shape)
- a range of expected values of Kp
![expected shape](/docs/shape.png)

### Output

- The optimal value of Kp for 2 criteria:
    - The one that went farthest
    - The one that used less correction
It turns out that the second criteria fits more to the real robot since inertia isn't taken into account in this cinematic simulation but has dramatic effect on osciliation of the robot and
therefore on its ability to go the farthest. Because minimizing correction minimizes inertia effects it makes sense that kp values returned by this criteria matches real life experiment better.

### Results
![Correction Intensity](/src/omega.png)
![Followed Path](/src/path.png)
