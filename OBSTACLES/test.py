from obstacle import process_depth
import cv2
import numpy as np

while 1:
    obstacles, image = process_depth()
    print(obstacles)
    ob = np.array(obstacles)
    print(ob)
    obstacles.clear()
    cv2.imshow( "abs", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()