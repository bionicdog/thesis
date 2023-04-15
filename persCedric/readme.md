#cone detection with opencv

##modules

###image

####functions

#####readimage(<image path>)
returns the RGB-image on that path

#####Rgb2HsvImage(<RGB image>)
returns HSV-image

#####showImage(<RGB image>)
plots the image with matplotlib
returns nothing

###edge_detection

####functions

#####blueFilter(<RGB image>), yellowFilter(<RGB image>), orangeFilter(<RGB image>)
Filters to contain only blue, yellow or orange from the image and makes all the other pixels black
returns an RGB image that contains only the colors that are in between the tresholds

#####morphOpen(<RGB image>)
morphing open means first eroding and then dilating
Using this you get rid of noise
returns an RGB image

#####getContours(<RGB image>)
calculates contours after edge detection and approximates those contours
returns the contours after edge-detection

#####contourIsCone(<contour>)
checks if a contour is pointing up (like a cone), to delete some backgroundforms
returns bool, [x, y, width, height]