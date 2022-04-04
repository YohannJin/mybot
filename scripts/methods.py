import cv2
import numpy as np

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.depth = None

    def set_dep(self, value):
        self.depth = value

def select_blob(blobsA, blobsB): # blobs_raw, blobs_tuned
    selected_blobs = []
    for blobB in blobsB:
        centerY, centerX = np.round(np.average(blobB, axis = 0))
        centerY, centerX = (int(centerY), int(centerX))
        for blobA in blobsA:
            num = np.asarray(np.where(blobA[np.where(blobA[:, 0]==centerY)][:, 1] ==centerX)).shape[1]
            if num > 0:
                selected_blobs.append(blobA)
                break
    return selected_blobs

def sim(im, target):
    coors = np.where(target > 0)
    matached = np.where(im[coors] > 0)
    return matached[0].shape[0] / coors[0].shape[0]

def red_segm(bgr_im):
    h, w, _ = bgr_im.shape
    gray_im = cv2.cvtColor(bgr_im, cv2.COLOR_BGR2GRAY)
    # global detection
    hsv_im = cv2.cvtColor(bgr_im, cv2.COLOR_BGR2HSV)
    # Red color
    low_red = np.array([160, 155, 0])
    high_red = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv_im, low_red, high_red)
    red = cv2.bitwise_and(bgr_im, bgr_im, mask=red_mask)
    red = cv2.dilate(red, kernel=(5,5), iterations=5)
    red2gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    corrs = np.where(red2gray > 0)
    y_min = np.min(corrs[0])-20
    y_max = np.max(corrs[0])+20
    x_min = np.min(corrs[1])-20
    x_max = np.max(corrs[1])+20
    if x_min < 0:
        x_min = 0
    if y_min < 0:
        y_min = 0
    if x_max > w:
        x_max = w
    if y_max > h:
        y_max = h
    mask = np.zeros((h, w), np.uint8)
    for m_h in range(y_min, y_max):
        for m_w in range(x_min, x_max):
            mask[m_h, m_w] = 1
    detected_im = cv2.bitwise_and(bgr_im, bgr_im, mask=mask)

    # local segmentation
    hsv_im = cv2.cvtColor(detected_im, cv2.COLOR_BGR2HSV)
    low_red = np.array([0, 155, 0])
    high_red = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv_im, low_red, high_red)
    red_gray = cv2.bitwise_and(gray_im, gray_im, mask=red_mask)

    return red_gray
    #red = cv2.dilate(red, kernel=(7, 7), iterations=10)
    #detected_im_gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    #line_detect(detected_im_gray)
    #red_rect = cv2.rectangle(red, (x_min, y_min), (x_max, y_max), color=(0, 255, 0))

def red_segm_bom(bgr_im):
    h, w, _ = bgr_im.shape

    # global detection
    hsv_im = cv2.cvtColor(bgr_im, cv2.COLOR_BGR2HSV)
    # Red color
    low_red = np.array([0, 100, 0])
    high_red = np.array([255, 255, 255])
    red_mask = cv2.inRange(hsv_im, low_red, high_red)
    red = cv2.bitwise_and(bgr_im, bgr_im, mask=red_mask)

    #red = cv2.dilate(red, kernel=(7, 7), iterations=5)
    #red = cv2.medianBlur(red, 5)
    red2gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(red2gray, 1, 255, 0)
    # get the raw area
    median = cv2.medianBlur(binary, 3)
    ret, labels = cv2.connectedComponents(median)
    blobs_raw = []
    for label in range(1, ret):
        coordinate = np.asarray(np.where(labels == label)).transpose()
        blobs_raw.append(coordinate)
    # get the area after erosion
    kernel = np.ones((5, 5), np.uint8)
    binary = cv2.morphologyEx(median, cv2.MORPH_CLOSE, kernel)
    erosion = cv2.erode(binary, kernel=(3, 3), iterations=15)
    #cv2.imshow("result", binary)
    #cv2.waitKey(0)


    ret, labels = cv2.connectedComponents(erosion)
    blobs_tuned = []
    for label in range(1, ret):
        coordinate = np.asarray(np.where(labels == label)).transpose()
        if coordinate.shape[0] > 10:
            blobs_tuned.append(coordinate)
    print(len(blobs_tuned))
    final_blobs = select_blob(blobs_raw, blobs_tuned)
    return final_blobs
    """
    tuned_binary = np.zeros(erosion.shape, np.uint8)
    for fblob in final_blobs:
        y_min = np.min(fblob[:, 0])
        y_max = np.max(fblob[:, 0])
        x_min = np.min(fblob[:, 1]) + 1
        x_max = np.max(fblob[:, 1]) + 1
        tuned_binary[y_min:y_max, x_min:x_max] = gray_im[y_min:y_max, x_min:x_max]
        patch = gray_im[y_min:y_max, x_min:x_max]
        cv2.imshow("result", patch)
        cv2.waitKey(0)
    _, binary = cv2.threshold(tuned_binary, 20, 255, 0)
    binary = cv2.medianBlur(binary, 5)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    target = np.zeros(bgr_im.shape, dtype=np.uint8)
    target.fill(255)
    for c in contours:
        mask = np.zeros(bgr_im.shape, dtype=np.uint8)
        cv2.drawContours(mask, [c], -1, (255), thickness=cv2.FILLED)

        # copy the relevant part into a new image
        # (you might want to use bounding box here for more efficiency)
        single = cv2.bitwise_and(bgr_im, bgr_im, mask=mask)

        # then apply your rotation operations both on the mask and the result
        single = cv2.doContourSpecificOperation(single)
        mask = cv2.doContourSpecificOperation(mask)

        # then, put the result into your target image (which was originally white)
        target = cv2.bitwise_and(target, single, mask=mask)
    cv2.imshow("result", target)
    cv2.waitKey(0)
    """

def area_classify(gray, ori):
    _, binary = cv2.threshold(gray, 1, 255, 0)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    centers = np.zeros((len(contours), 2))
    widths = []
    for i, countour in enumerate(contours):
        #countour = countour[0]
        print(countour.shape)
        centers[i, :] = np.average(countour)
        width = np.max(countour[:, 0, 0]) - np.min(countour[:, 0, 0])
        widths.append(width)
    selected_contour = contours[np.argmax(widths)]
    y_min = np.min(selected_contour[:, 0, 1])
    x_min = np.min(selected_contour[:, 0, 0])
    x_max = np.max(selected_contour[:, 0, 0])
    print(len(contours), y_min, x_min, x_max)
    #cv2.drawContours(ori, contours, np.argmax(widths), (0, 255, 0), 3)
    #red_rect = cv2.rectangle(ori, (x_min, y_min), (x_min+10, y_min+10), color=(0, 255, 0))
    #red_rect = cv2.rectangle(red_rect, (x_max, y_min), (x_max + 10, y_min + 10), color=(0, 255, 0))
    #cv2.imshow('linesDetected', red_rect)
    #cv2.waitKey(0)
    pt1 = Point(x_min, y_min)
    pt2 = Point(x_max, y_min)
    return (pt1, pt2)

def area_detect(gray, ori):
    _, binary = cv2.threshold(gray, 1, 255, 0)
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    centers = np.zeros((len(contours), 2))
    widths = []
    for i, countour in enumerate(contours):
        #countour = countour[0]
        print(countour.shape)
        centers[i, :] = np.average(countour)
        width = np.max(countour[:, 0, 0]) - np.min(countour[:, 0, 0])
        widths.append(width)
    selected_contour = contours[np.argmax(widths)]
    y_min = np.min(selected_contour[:, 0, 1])
    x_min = np.min(selected_contour[:, 0, 0])
    x_max = np.max(selected_contour[:, 0, 0])
    print(len(contours), y_min, x_min, x_max)
    #cv2.drawContours(ori, contours, np.argmax(widths), (0, 255, 0), 3)
    #red_rect = cv2.rectangle(ori, (x_min, y_min), (x_min+10, y_min+10), color=(0, 255, 0))
    #red_rect = cv2.rectangle(red_rect, (x_max, y_min), (x_max + 10, y_min + 10), color=(0, 255, 0))
    #cv2.imshow('linesDetected', red_rect)
    #cv2.waitKey(0)
    pt1 = Point(x_min, y_min)
    pt2 = Point(x_max, y_min)
    return (pt1, pt2)

def get_headP(bgr_im):
    red_gray = red_segm(bgr_im)
    return area_detect(red_gray, bgr_im)



if __name__ == "__main__":
    bgr = cv2.imread("./test_images/rgb/color_1.jpg")
    #cv2.imshow("bgr", bgr)
    #cv2.waitKey(0)
    red = get_headP(bgr)
    cv2.imshow("red", red)
    cv2.waitKey(0)

