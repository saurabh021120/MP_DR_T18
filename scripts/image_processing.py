#!/usr/bin/env python
import sys
import cv2
import numpy as np
import random as rng

import rospy
rng.seed(12345)

class image_processing:
    def __init__(self):
        self.road_depth = None
        self.road_center = 320
        self.avg_road_depth = None

    def get_centroid(self, contour, shift=(0,0)):
        M = cv2.moments(contour)
        cx, cy = None, None
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            rospy.logerr('Centroid could not be found!')
        return cx+shift[0], cy+shift[1]

    def get_angle(self, centroid1, centroid2):
        if None in centroid1 or None in centroid2:
            return None
        angle = np.arctan2(centroid2[1]-centroid1[1], centroid2[0]-centroid1[0])
        if angle < 0:
            angle = angle + 2*np.pi
        return angle/np.pi*180-90

    def get_contours(self, cv_image, depth, debug=False):
        if (self.road_depth-self.avg_road_depth):
            k=5
            margin = max(k*(self.road_depth-self.avg_road_depth),1.3)
        else:
            margin = 1.3
        rospy.loginfo('Margin: %f', margin)
        mask = cv2.inRange(cv_image, (depth-margin, depth-margin, depth-margin), (depth+margin, depth+margin, depth+margin))
        # mask = cv2.inRange(cv_image, (17,17,17), (19,19,19))
        if debug:
            cv2.imshow('input', cv_image)
            cv2.imshow('mask', mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=5)
        # if debug:
        #     cv2.imshow('erroded', mask)
        # mask = cv2.dilate(mask, kernel, iterations=3)
        # if debug:
        #     cv2.imshow('dilated', mask)
        # mask = cv2.erode(mask, kernel, iterations=15)
        # if debug:
        #     cv2.imshow('erroded2', mask)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if debug:
            ri = np.copy(cv_image)
            cv2.drawContours(ri, contours, -1, (0,255,0), 3)
            cv2.imshow('contours', ri)
            # cv2.waitKey(0)
        if len(contours) == 0:
            rospy.logerr('No contours found!')
        return contours[0]
    
    def get_car_error(self, cv_image, cam, debug=False):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        height, width, channels = cv_image.shape
        road_height = []
        for i in range(channels):
            road_height.append(np.average(cv_image[210:270, 280:360]))

        contours = self.get_contours(cv_image, road_height)
        contour1 = self.get_contours(cv_image[:240][:], road_height)
        contour2 = self.get_contours(cv_image[240:][:], road_height)
        centroid1 = self.get_centroid(contour1)
        centroid2 = self.get_centroid(contour2,(0,240))
        angle = self.get_angle(centroid1, centroid2)
        # print('Angle: ', angle)
        # if debug:
        rgb_img = np.copy(cam.get_rgb_image())
        # cv2.imshow("RGB",rgb_img)
        # rgb = cv2.circle(rgb, centroid1, radius=2, color=(0, 0, 255), thickness=-1)
        # rgb = cv2.circle(rgb, centroid2, radius=2, color=(0, 0, 255), thickness=-1)
        # cv2.drawContours(rgb, [contours], -1, (0,255,0), 3)
        # print(contours.shape)
        (x_1, y_1, w_1, h_1) = cv2.boundingRect(contours)
        # cv2.rectangle(rgb,(x_1,y_1),(x_1+w_1,y_1+h_1),(255,0,0),3)

        ################################
        gray_image = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
        _,thresh = cv2.threshold(gray_image, 50, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # cv2.drawContours(rgb_img,contours,-1,(255,255,0),3)
        ##
        rgb = rgb_img
        error=[]
        error_x=[]
        error_y=[]
        centroid = []
        for c in contours:
            if cv2.contourArea(c) < 250:
                    continue
            (x, y, w, h) = cv2.boundingRect(c)
            # cv2.rectangle(rgb_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # cv2.line(rgb,(x_1,0),(x_1,479),(0,255,255),3)
            # cv2.line(rgb,(x_1+w_1,0),(x_1+w_1,479),(0,255,255),3)
            cen_rec = (int((2 * x + w) / 2), int((2 * y + h) / 2)) 
            if cen_rec[0]<=x_1 or cen_rec[0]>=x_1+w_1 or cen_rec[1]<=100:
                continue
            centroid.append(cen_rec)
            # cv2.line(rgb,((x_1+w_1/2),0),((x_1+w_1/2),479),(0,255,0),3)
            err = abs(cen_rec[0] - (x_1+w_1/2))
            if err > 50:
                continue
            err_x = (cen_rec[0] - (x_1+w_1/2))
            err_y = (cen_rec[1] - (y_1+h_1/2))
            error.append(err)
            error_x.append(err_x)
            error_y.append(err_y)
            # cv2.circle(rgb, cen_rec, 6, (255, 0, 0), -1)
            # cv2.putText(rgb_img, "centroid", (cen_rec[0] - 25, cen_rec[1] - 25),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.circle(rgb,(int(x_1+w_1/2),int(y_1+h_1/2)),5,(0,255,0),-1)
        err_x = 0
        err_y = 0
        if error:
            error = np.array(error)
            cen = np.argmin(error)
            cv2.circle(rgb,centroid[cen],10,(0,0,255),-1)
            print(error_x[cen],error_y[cen])
            err_x = error_x[cen]
            err_y = error_y[cen]
        #########################
        if debug:
            # cv2.imshow('Thresh', thresh)
            cv2.imshow('Car-detection', rgb_img)
            # cv2.imshow("thresh",thresh)
            cv2.waitKey(1)
        return err_x,err_y

    def get_road_angle(self, depth_image, cam, debug=False):
        cv_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
        height, width, channels = cv_image.shape
        contours = self.get_contours(cv_image, (self.road_depth+self.avg_road_depth)/2)
        contour1 = self.get_contours(cv_image[:240][:], self.road_depth)
        contour2 = self.get_contours(cv_image[240:][:], self.avg_road_depth)
        centroid1 = self.get_centroid(contour1)
        centroid2 = self.get_centroid(contour2,(0,240))
        self.road_center = (centroid1[0]+centroid2[0])/2
        angle = self.get_angle(centroid1, centroid2)
        # print('Angle: ', angle)
        if debug:
            rgb = cam.get_rgb_image()
            rgb = cv2.circle(rgb, centroid1, radius=2, color=(0, 0, 255), thickness=-1)
            rgb = cv2.circle(rgb, centroid2, radius=2, color=(0, 0, 255), thickness=-1)
            cv2.drawContours(rgb, [contours], -1, (0,255,0), 3)
            cv2.imshow('contours', rgb)
            cv2.waitKey(1)
            # rect = cv2.minAreaRect(contours[0])
            # (x_min, y_min), (w_min, h_min), angle = rect
            # if angle > 0:
            #     angle = angle-90
            # else:
            #     angle = angle+90
            # if angle < -45:
            #     angle = 90 + angle
            # elif angle > 45:
            #     angle = -90 + angle

        # Update road depth
        self.update_road_depth(depth_image, contour1)
        self.update_avg_road_depth(depth_image, contours)  
        return angle

    def update_road_depth(self, depth_img, contour):
        mask = np.zeros(depth_img.shape, np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        # cv2.imshow('mask', mask)
        # cv2.waitKey(0)
        self.road_depth = np.average(cv2.mean(depth_img, mask=mask)[0])
        rospy.loginfo('Depth: %f', self.road_depth)

    def update_avg_road_depth(self, depth_img, contour):
        mask = np.zeros(depth_img.shape, np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        # cv2.imshow('mask', mask)
        # cv2.waitKey(0)
        self.avg_road_depth = np.average(cv2.mean(depth_img, mask=mask)[0])

    def update_drone_depth(self, depth_img):
        self.road_depth = np.average(depth_img[210:270, 280:360])
        self.avg_road_depth = np.average(depth_img[210:270, 280:360])
        rospy.loginfo('Depth: %f', self.road_depth)
    
    def get_road_depth(self):
        return self.road_depth

    def get_road_center(self):
        return self.road_center

def main(args):
    ic = image_processing()
    for i in range(43):
        image = cv2.imread(sys.argv[1]+'depth-road-' + str(i) + '.png')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ic.get_road_angle(image,True)

if __name__ == '__main__':
    main(sys.argv)