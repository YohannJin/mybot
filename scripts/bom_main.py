import cv2
from methods import red_segm_bom, sim
import os
import numpy as np
from sklearn.datasets import load_iris
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process.kernels import RBF

def get_pattern_list(bgr, templates, t_shape, classify_model):
    pattern_list = []
    blobs = red_segm_bom(bgr)
    gray_im = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    t_h, t_w = t_shape
    for b in blobs:
        y_min = np.min(b[:, 0])
        y_max = np.max(b[:, 0])
        x_min = np.min(b[:, 1]) + 1
        x_max = np.max(b[:, 1]) + 1
        patch = gray_im[y_min:y_max, x_min:x_max]
        _, binary = cv2.threshold(patch, 50, 255, 0)
        binary = cv2.resize(binary, t_shape)
        #binary = cv2.erode(binary, kernel=(3,3), iterations=5)
        #ret, result, neighbours, dist = classify_model.findNearest(np.reshape(binary, (-1, 2500)).astype(np.float32), k=1)
        binary_input = np.reshape(binary, (-1, 2500)).astype(np.float32) / 155.0
        prob = classify_model.predict_proba(binary_input)
        print(prob)
        """
        sims = []
        for i, t in enumerate(templates):
            sims.append(sim(binary, t))
        if np.max(sims) > 0.4:
            pattern_list.append(np.argmax(sims)+1)
        """
        cv2.imshow("result", binary)
        cv2.waitKey(0)
    """
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
    """
    print(pattern_list)
    #cv2.imshow("result", red_segmentation)
    #cv2.waitKey(0)

def make_templates():
    im = cv2.imread("./test_images/template/5.png")
    im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(im, 200, 255, 0)
    binary = cv2.resize(binary, (50, 50))
    cv2.imshow("result", binary)
    cv2.waitKey(0)
    cv2.imwrite("./test_images/template/5_bi.png", binary)

def fill(img, h, w):
    img = cv2.resize(img, (h, w), cv2.INTER_CUBIC)
    return img

def horizontal_shift(img, ratio=0.0):
    if ratio > 1 or ratio < 0:
        print('Value should be less than 1 and greater than 0')
        return img
    ratio = np.random.uniform(-ratio, ratio)
    h, w = img.shape[:2]
    to_shift = w * ratio
    if ratio > 0:
        img = img[:, :int(w - to_shift)]
    if ratio < 0:
        img = img[:, int(-1 * to_shift):]
    img = fill(img, h, w)
    return img

def augmentation(im, num):
    images = []
    for r in range(num):
        images.append(horizontal_shift(im, r/num))

    return images

def read_templates(path):
    templates = []
    im_files = os.listdir(path)
    for i_f in im_files:
        im = cv2.imread(path+i_f)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(im, 1, 255, 0)
        #cv2.imshow("result", binary)
        #cv2.waitKey(0)
        templates.append(binary)

    train_images = []
    labels = []
    for i, t in enumerate(templates):
        train_images += augmentation(t, 100)
        for l in range(100):
             labels.append(i)
    train = np.array(train_images)

    train = train.reshape(-1, 2500).astype(np.float32) / 255.0  # Size = (2500,400)
    #labels = np.arange(len(templates))  # Size = (2500,400)
    # Initiate kNN, train it on the training data, then test it with the test data with k=1
    #knn = cv2.ml.KNearest_create()
    #knn.train(train, cv2.ml.ROW_SAMPLE, labels)
    #ret, result, neighbours, dist = knn.findNearest(train, k=1)
    #print("------------------------")
    #print(ret, result, neighbours, dist)
    #print("------------------------")
    X, y = load_iris(return_X_y=True)
    #print(X.shape, train.shape, y, labels)
    kernel = 1.0 * RBF(1.0)
    gpc = GaussianProcessClassifier(kernel=kernel, random_state = 0).fit(train, labels)
    print(gpc.predict_proba(train))

    return templates, binary.shape, gpc


def main():
    bgr = cv2.imread("./test_images/bom/born_c.jpg")
    templates, t_s, model = read_templates(path="./test_images/template/")
    get_pattern_list(bgr, templates, t_s, model)
    #make_templates()

if __name__ == "__main__":
    main()