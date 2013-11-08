ct="/tmp/check_traj.dat"
fct="/tmp/foot_check_traj.dat"

set key right bottom Left title 'Legend'
set terminal postscript
set output "debug_trajectories.ps"
plot fct u 1 w l t "Left Foot X", fct u 5 w l t "Right Foot X", fct u 9 w l t "CoM X", fct u 12 w l t "ZMP X", ct u 16 w l t "ZMP X - tv"
set terminal X11
replot

