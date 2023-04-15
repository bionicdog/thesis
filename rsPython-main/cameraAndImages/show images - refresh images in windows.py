
import cv2
import glob
from time import sleep


images = glob.glob('*.jpg')

print ('read ', len(images), ' images')

for fname in images:
    img = cv2.imread(fname)
    gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    cv2.imshow("original image", img)
    cv2.imshow("grayed image", gray_img)
    cv2.waitKey(500)
    #sleep(1)

cv2.waitKey(500)
cv2.destroyAllWindows()


