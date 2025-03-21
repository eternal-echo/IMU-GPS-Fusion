set terminal png size 1200,800
set output '../results/particle_filter_results.png'
set multiplot layout 2,1
set title 'Particle Filter Trajectory'
plot '../results/trajectory.dat' using 1:2 with lines title 'Estimated Path', \
     '../results/trajectory.dat' using 3:4 with points pt 7 title 'GPS Measurements'
set title 'Position Uncertainty'
plot '../results/error.dat' using 1:2 with lines title 'X StdDev', \
     '../results/error.dat' using 1:3 with lines title 'Y StdDev'
unset multiplot
