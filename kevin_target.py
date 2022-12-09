import cv2 as cv
import numpy as np
from threading import Thread
import math

cap = cv.VideoCapture(0)

_, frame = cap.read()
WIDTH, HEIGHT = frame.shape[:2]


def find_blue(frame):
    condition1 = frame[..., 0] > 80  # blue
    condition2 = frame[..., 1] > 80  # green
    condition3 = frame[..., 1] < 150
    condition4 = frame[..., 2] < 90  # red
    return np.logical_and(np.logical_and(condition1, condition2),
                          np.logical_and(condition3, condition4))


def find_red(frame):
    condition1 = frame[..., 0] < 80
    condition2 = frame[..., 1] > 40
    condition3 = frame[..., 1] < 100
    condition4 = frame[..., 2] > 130
    return np.logical_and(np.logical_and(condition1, condition2),
                          np.logical_and(condition3, condition4))


def find_yellow(frame):
    condition1 = frame[..., 0] < 130
    condition2 = frame[..., 1] > 120
    condition3 = frame[..., 2] > 120
    return np.logical_and(np.logical_and(condition1, condition2),
                          condition3)


def find_white(frame):
    condition1 = frame[..., 0] > 200
    condition2 = frame[..., 1] > 200
    condition3 = frame[..., 2] > 200
    return np.logical_and(np.logical_and(condition1, condition2),
                          condition3)


def det_circle(frame, res):
    res.append(cv.HoughCircles(frame, cv.HOUGH_GRADIENT,
                               1, 50, param1=80, param2=20, minRadius=10, maxRadius=100))  # EDIT THESE PARAMS


def get_area(x, y):
    area = 0.5*((x[0]*(y[1]-y[2])) + (x[1]*(y[2]-y[0])) + (x[2]*(y[0]-y[1])))
    return int(abs(area))


def dilate_erode(frame, d_iter, e_iter):
    kernel = np.ones((3, 3), np.uint8)
    frame = cv.dilate(frame, kernel, d_iter)
    frame = cv.dilate(frame, kernel, e_iter)
    return frame


while True:
    _, frame = cap.read()

    b_circles = r_circles = y_circles = []

    mask_white = np.zeros(shape=(WIDTH, HEIGHT))
    mask_blue = np.zeros(shape=(WIDTH, HEIGHT))
    mask_yellow = np.zeros(shape=(WIDTH, HEIGHT))
    mask_red = np.zeros(shape=(WIDTH, HEIGHT))

    r_res = find_red(frame)
    b_res = find_blue(frame)
    y_res = find_yellow(frame)
    w_res = find_white(frame)

    mask_blue[b_res] = 255
    mask_yellow[y_res] = 255
    mask_red[r_res] = 255
    mask_white[w_res] = 255

    # NOT VERY IMP BUT TRY EDITING THESE TOO
    mask_blue = dilate_erode(np.array(mask_blue, dtype=np.uint8), 3, 3)
    mask_red = dilate_erode(np.array(mask_red, dtype=np.uint8), 2, 2)
    mask_yellow = dilate_erode(np.array(mask_yellow, dtype=np.uint8), 4, 1)
    mask_white = dilate_erode(np.array(mask_white, dtype=np.uint8), 7, 5)

    contours, _ = cv.findContours(
        mask_white, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # print(contours)
    sel_points = []
    if(len(contours) > 0):
        for contour in contours:
            approx = cv.approxPolyDP(
                contour, 0.085*cv.arcLength(contour, True), True)
            if(len(approx) == 4):
                x, y, w, h = cv.boundingRect(contour)
                if((w/h <= 2 or h/w <= 2) and w > 20 and h > 20):
                    sel_points.append([x, y, w, h])
                    # frame = cv.rectangle(
                    #     frame, (x, y), (x+w, y+h), (0, 0, 255), 4)

    blue_det = Thread(target=det_circle, args=(mask_blue, b_circles))
    yellow_det = Thread(target=det_circle, args=(mask_yellow, y_circles))
    red_det = Thread(target=det_circle, args=(mask_red, r_circles))

    blue_det.start()
    yellow_det.start()
    red_det.start()

    blue_det.join()
    yellow_det.join()
    red_det.join()

    # print(b_circles)
    # print(y_circles)
    # print(r_circles)
    # try:
    b_circles = np.uint16(
        np.around(b_circles[0])) if b_circles[0] is not None else np.array([[]])
    r_circles = np.uint16(
        np.around(r_circles[0])) if r_circles[0] is not None else np.array([[]])
    y_circles = np.uint16(
        np.around(y_circles[0])) if y_circles[0] is not None else np.array([[]])

    # if len(b_circles[0]) > 0 and len(r_circles[0]) > 0 and len(y_circles[0]) > 0:
    #     for b in b_circles[0, :]:
    #         for r in r_circles[0, :]:
    #             for y in y_circles[0, :]:
    #                 # EDIT THESE PARAMS (< 10)
    #                 if get_area([b[0], r[0], y[0]], [b[1], r[1], y[1]]) < 10 and b[2] >= y[2]:
    #                     cv.circle(frame, (b[0], b[1]), 10, (0, 0, 255), -1)

    possible_points = []
    possible_semi = []
    possible_final = []

    dist_thresh = 30

    for b in b_circles[0, :]:
        for r in r_circles[0, :]:
            if math.dist(b[:2], r[:2]) < dist_thresh:
                possible_points.append((b[:2]+r[:2])/2)

    for r in r_circles[0, :]:
        for y in y_circles[0, :]:
            if math.dist(r[:2], y[:2]) < dist_thresh:
                possible_semi.append((r[:2]+y[:2])/2)
                for p in possible_points:
                    if math.dist((r[:2]+y[:2])/2, p) < dist_thresh:
                        possible_final.append(((r[:2]+y[:2])/2+p)/2)

    if(len(possible_final) > 0):
        for point in possible_final:
            for bound in sel_points:
                if(point[0] >= bound[0] and point[1] >= bound[1] and point[0] <= bound[0]+bound[2] and point[1] <= bound[1]+bound[3]):
                    cv.circle(frame, (int(point[0]), int(
                        point[1])), 10, (0, 0, 255), -1)
    else:
        for point in possible_semi:
            for bound in sel_points:
                if(point[0] >= bound[0] and point[1] >= bound[1] and point[0] <= bound[0]+bound[2] and point[1] <= bound[1]+bound[3]):
                    cv.circle(frame, (int(point[0]), int(
                        point[1])), 10, (0, 0, 255), -1)

    cv.imshow("blue", mask_blue)
    cv.imshow("red", mask_red)
    cv.imshow("yellow", mask_yellow)
    cv.imshow("white", mask_white)
    # except:
    #     print("Error ")

    cv.imshow("frame", frame)

    if cv.waitKey(10) & 0xFF == 27:
        break

cv.destroyAllWindows()
cap.release()
