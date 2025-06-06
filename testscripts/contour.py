import cv2 as cv#read the image



def is_point_in_polygon(x, y, polygon):
    """
    Determine if a point is inside a polygon using the ray casting algorithm.

    Args:
        x, y: Coordinates of the point.
        polygon: List of (x, y) tuples representing the polygon's vertices.

    Returns:
        True if the point is inside the polygon, False otherwise.
    """
    num = len(polygon)
    j = num - 1
    inside = False

    for i in range(num):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and \
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
            inside = not inside
        j = i

    return inside



img = cv.imread("/tmp/house.png")
height, width, channels = img.shape

#convert the image to grayscale
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#blur image to reduce the noise in the image while thresholding. #This smoothens the sharp edges in the image.
blur = cv.blur(gray, (10,10))
#Apply thresholding to the image
ret, thresh = cv.threshold(blur, 1, 255, cv.THRESH_OTSU)
#find the contours in the image
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
polygon = 0
print(f"Length Contours: {len(contours)}")
for i in range(len(contours)):
    if is_point_in_polygon(5/0.007229358,-5/0.007229358, contours[i].reshape(-1, 2)):
        polygon = i
        break

new = (contours[polygon].reshape(-1, 2)*0.007229358).tolist()
originx = -7.14
originy = -7.83
flipped = [[x+originx, -y+height*0.0072293588+originy] for x, y in new]


field = flipped 
field.append(field[0])

#navigator.navigateCoverage(field)


#draw the obtained contour lines(or the set of coordinates forming a line) on the original image
cv.drawContours(img, contours, polygon, (0,255,0), 20)
#show the image
cv.namedWindow('Contours',cv.WINDOW_NORMAL)
cv.imshow('Contours', img)
if cv.waitKey(0):
    cv.destroyAllWindows()

new = contours[5].reshape(-1, 2).tolist()
#flip
new2 = [[x, -y] for x, y in new] 
# create a closed loop as poligon
new.append(new[0])
