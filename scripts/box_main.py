import cv2
from methods import get_headP

def get_diff(bgr, d):
    head_points = get_headP(bgr)
    head_points[0].set_dep(d[head_points[0].y, head_points[0].x, 0])
    head_points[1].set_dep(d[head_points[1].y, head_points[1].x, 0])

    rbgr_rect = cv2.rectangle(bgr, (head_points[0].x, head_points[0].y), (head_points[0].x + 4, head_points[0].y + 4), color=(0, 255, 0))
    rbgr_rect = cv2.rectangle(rbgr_rect, (head_points[1].x, head_points[1].y), (head_points[1].x + 4, head_points[1].y + 4),
                              color=(0, 255, 0))
    rbgr_rect = cv2.putText(rbgr_rect, org=(head_points[0].x-10,
                            head_points[0].y-10), text="(" + str(head_points[0].x) + "," +
                                                    str(head_points[0].y) + "," + str(head_points[0].depth) + ")",
                            color=(0, 255, 0), fontScale=0.3, thickness=1, fontFace = cv2.FONT_HERSHEY_DUPLEX)
    rbgr_rect = cv2.putText(rbgr_rect, org=(head_points[1].x,
                                            head_points[1].y),
                            text="(" + str(head_points[1].x) + "," + str(head_points[1].y) + "," + str(
                                head_points[1].depth) + ")",
                            color=(0, 255, 0), fontScale=0.3, thickness=1, fontFace = cv2.FONT_HERSHEY_DUPLEX)
    cv2.imshow("result", rbgr_rect)
    cv2.waitKey(0)


def main():
    #read in TODO ros subscriber
    bgr = cv2.imread("./test_images/rgb/color_2.jpg")
    d = cv2.imread("./test_images/depth/dep_2.jpg")
    diff = get_diff(bgr, d)

if __name__ == "__main__":
    main()