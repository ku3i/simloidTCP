


for f in *.ppm; do convert -quality 100 $f  `basename $f .ppm`.jpg; rm $f; done
mencoder "mf://*.jpg" -mf fps=25 -o movie.avi -ovc lavc -lavcopts vcodec=mjpeg
rm *.jpg

