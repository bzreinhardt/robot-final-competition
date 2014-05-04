Things to try:

- making the integrate odom multiply u by 1.1 or so to take into account communication delay



- if there is a wall between the particle and the last good position, weight it lower

Things Tried
- averaging the particle the correct way [CHECK This Makes things work better]
- changing estimate method based on localization [Check, this localizes quickly]
- adjust particle weight so that they can't jump [CHECK]
