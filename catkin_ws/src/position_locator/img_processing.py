#!/usr/bin/python3
import cv2
import numpy as np


def get_marker_position(img):
    img_mean = img.mean(axis=2).astype(np.uint8)
    bin_img = np.where(img_mean>200, 1, 0)
    im_bw = cv2.threshold(img_mean, 200, 255, cv2.THRESH_BINARY)[1]
    # Find the contours in the image.
    _, contours, _ = cv2.findContours(im_bw, cv2.RETR_TREE,
                                      cv2.CHAIN_APPROX_SIMPLE)
    # Find the biggest contour, corresponding to the square.
    max_area = 0
    square_cnt = None
    for cnt in contours:
        cnt_area = cv2.contourArea(cnt)
        if cnt_area > max_area:
            square_cnt = cnt
            max_area = cnt_area
    return (im_bw, square_cnt)

def get_marker_centre(contour):
    cnt_moments = cv2.moments(contour)
    centre_x = int(cnt_moments["m10"] / cnt_moments["m00"])
    centre_y = int(cnt_moments["m01"] / cnt_moments["m00"])
    return (centre_x, centre_y)
