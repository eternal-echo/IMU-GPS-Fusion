set terminal png size 1600,1200 enhanced font 'Arial,12'
set output '../results/particle_filter_results.png'
set multiplot layout 2,1 spacing 0.15
set size 1,0.6
set origin 0,0.4
set title 'Particle Filter Trajectory' font 'Arial,14'
set xlabel 'Longitude (degrees)' font 'Arial,12'
set ylabel 'Latitude (degrees)' font 'Arial,12'
set grid
set key outside right
set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5
set style line 2 lc rgb '#dd181f' lt 1 lw 1 pt 5 ps 1.0
plot '../results/trajectory.dat' using 1:2 with linespoints ls 1 title 'Estimated Path',\
     '../results/trajectory.dat' using 3:4 with points ls 2 title 'GPS Measurements'
set size 1,0.4
set origin 0,0
set title 'Position Uncertainty Over Time' font 'Arial,14'
set xlabel 'Frame' font 'Arial,12'
set ylabel 'Standard Deviation (m)' font 'Arial,12'
set style line 3 lc rgb '#008000' lt 1 lw 2
set style line 4 lc rgb '#800080' lt 1 lw 2
plot '../results/error.dat' using 1:2 with lines ls 3 title 'X StdDev',\
     '../results/error.dat' using 1:3 with lines ls 4 title 'Y StdDev'
unset multiplot
