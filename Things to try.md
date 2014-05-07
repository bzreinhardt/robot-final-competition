Things to try:

- making the integrate odom multiply u by 1.1 or so to take into account communication delay



- if there is a wall between the particle and the last good position, weight it lower

Things Tried
- averaging the particle the correct way [CHECK This Makes things work better]
- changing estimate method based on localization [Check, this localizes quickly]
- adjust particle weight so that they can't jump [CHECK]

Predictive position estimates - 

t_tot = t_odom_2 - t_odom_1 - time between the two odometry measurements
t_PF_start - time at beginning of particle filter
t_PF_end - time at the end of the particle filter

Things that need to be fixed - 
Response to hitting a wall
How close to a roadmap point do you have to be to consider it visited?