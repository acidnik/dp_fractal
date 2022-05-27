# Explanation
Coordinates of the pixel on the screen sets the initial state of the double pendulum.
X coordinate is the angle of first arm, mapped to 0 .. TAU (where TAU=2*PI) 
Y coordinate is the angle of second arm, mapped to 0 .. PI


When the second arm flips, i.e. passes over the upper dead center, the pendulum stops, and the color of the rect is set as the time it took to flip.
If it had not flipped after some time, it is stopped and its rect is drawn gray

We start with one pendulum, that fits into the entire screen. When it stops, we split it into 4 smaller pendulums

After stop we check all the adjacent stopped pendulums and compare their time. If the difference is less than 0.9, we split the pendulum and all of its neighbors into 4 smaller pendulums. The physical params of all pendulums (mass, length) are the same, they are only drawn smaller

While the pendulum is running, the color of the pendulum is set from the angle of second arm