#!/usr/bin/env python
import sys
import cv2
import rospy
import numpy as np
import image_geometry
import message_filters

from std_msgs.msg import Header, String
from geometry_msgs.msg import PointStamped, Point

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Gate:
    def __init__(self):
        # Throwback to the good ol' days (We'll have to fine tune for the situation)
        self.yellow_min = np.array([0, 50, 50], np.uint8)
        self.yellow_max = np.array([29, 250, 250], np.uint8)

        self.dims = (2.4, 1.2)  # dimensions in metes (w,l)

        # Orientation information
        self.point_publisher = rospy.Publisher("gate_points", PointStamped, queue_size=4)
        self.gate_pose = None
        self.approx_gate_distance = None


class GateFinder:
    # TODO: Make scale invariant!!!!
    def __init__(self, gate):
        self.gate = gate
        self.last_image = None
        self.last_image_time_stamp = None
        self.last_draw_image = None

        self.lcam_info, self.rcam_info = None, None
        self.lP, self.rP = None, None

        self.lcam_info_sub = rospy.Subscriber('/stereo/left/camera_info', CameraInfo, self.cinfo_cb, callback_args='l')
        self.rcam_info_sub = rospy.Subscriber('/stereo/right/camera_info', CameraInfo, self.cinfo_cb, callback_args='r')

        # Hacky shit to get the camera info and unsubscribe
        # TODO: Implement service that gathers camera_info
        # TODO: Reimplement this w/ message filters
        while self.lcam_info is None or self.rcam_info is None:
            pass

        rospy.loginfo('Left Camera Info: %s', self.lcam_info)
        rospy.loginfo('Right Camera Info: %s', self.rcam_info)

        self.lcam_info_sub.unregister()
        self.rcam_info_sub.unregister()

        # Begin stereo camera dirty work
        self.bridge = CvBridge()
        self.lcam_image_sub = message_filters.Subscriber('/stereo/left/image_rect_color', Image)
        self.rcam_image_sub = message_filters.Subscriber('/stereo/right/image_rect_color', Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.lcam_image_sub, self.rcam_image_sub], 5, 1)
        self.ts.registerCallback(self.image_cb)

    def cinfo_cb(self, cam_msg, cam):
        if cam == 'l':
            self.lcam_info = cam_msg
        elif cam == 'r':
            self.rcam_info = cam_msg
        else:
            pass

    def gen_gate_kernel(self, scale=15):
        if scale <= 3:
            vert_kern = np.ones((3,3))*-5
            horz_kern = np.ones((3,3))*-5
            midpoint = (1,1)
        else:
            vert_kern = np.ones((scale, scale))*-1
            horz_kern = np.ones((scale, scale))*-1
            midpoint = (scale // 2, scale // 2)

        cv2.rectangle(vert_kern, (midpoint[0]-1, 0), (midpoint[0]+1, scale-1), 1, -1)
        cv2.rectangle(horz_kern, (0, midpoint[1]-1), (scale-1, midpoint[1]+1), 1, -1)
        return vert_kern, horz_kern

    def ncc(self, image, scale=10):
        # kernel = np.ones((scale, scale)) * -1
        # midpoint = (scale // 2, scale // 2)
        # cv2.circle(kernel, midpoint, midpoint[0], 1, -1)
        img = None
        v, h = self.gen_gate_kernel()
        mean, std_dev = cv2.meanStdDev(image)
        
        #return cv2.filter2D((image - mean) / std_dev, -1, kernel)
        v_f = cv2.filter2D((image-mean) / std_dev, -1, v)
        h_f = cv2.filter2D((image-mean) / std_dev, -1, h)

        #img = cv2.bitwise_and(v_f, h_f)
        #cv2.imshow('B_masked', img)
        #cv2.waitKey(0)
        #return v_f
        return cv2.bitwise_and(v_f, h_f)

    def find_contours(self, image, cam):
        # TODO: Resize image to process faster
        # TODO: Limit this to the largest three contours
        # TODO: PCA for orientation (Differentiating b/w long and tall tubes)
        # TODO: Work on discriminating reflections (Should not be a problem at the transdec)
        # TODO: Convolve gate-type kernel @ different resolutions

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        ghsv = cv2.GaussianBlur(hsv,(9,9),0)
        mask = cv2.inRange(ghsv, self.gate.yellow_min, self.gate.yellow_max)

        h, s, v = cv2.split(ghsv)
        v = cv2.bitwise_and(v, v, mask=mask)
        v = self.ncc(v)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        gate_moments, largest_contours, edge_pts = [], [], []

        if contours is None:
            print 'No contours found'
        else:
            for cnt in contours:
                if cv2.contourArea(cnt) > 600 and cv2.contourArea(cnt) < 1400:
                    largest_contours.append(cnt)
                    moment = cv2.moments(cnt)
                    gate_moments.append((int(moment['m10']/moment['m00']), int(moment['m01']/moment['m00'])))   # Pixel location

        # Using bounding box to approximate the shape of the start gate tubes
        plane_pts = []
        for cnt in largest_contours:
            rect = cv2.minAreaRect(cnt)
            box_points = cv2.cv.BoxPoints(rect)

            box = [np.int0(cv2.cv.BoxPoints(rect))]
            cv2.drawContours(image, box, 0, (0, 0, 255), 2)

            # Let's get some box midpoints
            pts = [list(point) for point in box_points]

            # Here we would need to check the orientation of the contour
            plane_pts.extend([tuple((np.add(pts[0][:], pts[3][:]) // 2).astype(int)),
                              tuple((np.add(pts[2][:], pts[1][:]) // 2).astype(int))])

        for i, point in enumerate(plane_pts):
            cv2.circle(image, point, 4, (255, 0, 0), -1)
            cv2.putText(image, str(i), point, cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 4, (255, 255, 255), 2, cv2.CV_AA)

        if len(plane_pts) == 4:
            # If we have enough points for reconstruction
            dplane_pts = np.asarray([list(pt) for pt in plane_pts])

            # Camera Shenanigans
            if cam == 'l':
                self.lP = np.asarray(self.lcam_info.P).reshape((3, 4))
                P = self.lP
                return plane_pts
            elif cam == 'r':
                self.rP = np.asarray(self.rcam_info.P).reshape((3, 4))
                P = self.rP
                return plane_pts
            else:
                rospy.logerr('Invalid camera parameter provided')
                return None
        else:
            return None

    def visualize_pts(self, p_points):
        # Accepts a list of four points in order to generate a plane
        # Returns dims, and angles
        pass

    def triangulate_Linear_LS(self, lpoint, rpoint, lP, rP):
        # From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997

        # lpoint -> Left camera point (homogenous)
        # rpoint -> Right camera point (homogenous)
        # lP -> Left camera projection matrix
        # rP -> Right camera projection matrix

        # Build matrix A for homogenous equation system Ax = 0
        # Assume X = (x,y,z,1), for Linear-LS method
        # Which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1

        # !!! No need to provide homogenous coordinates, (u, v) suffice !!!

        print 'Left Point: ', lpoint
        print 'Right Point: ', rpoint, '\n'

        A = np.zeros((4, 3))
        B = np.zeros((4, 1))
        X = np.zeros((3, 1))

        A[0][0] = lpoint[0]*lP[2][0] - lP[0][0]
        A[0][1] = lpoint[0]*lP[2][1] - lP[0][1]
        A[0][2] = lpoint[0]*lP[2][2] - lP[0][2]

        A[1][0] = lpoint[1]*lP[2][0] - lP[1][0]
        A[1][1] = lpoint[1]*lP[2][1] - lP[1][1]
        A[1][2] = lpoint[1]*lP[2][2] - lP[1][2]

        A[2][0] = rpoint[0]*rP[2][0] - rP[0][0]
        A[2][1] = rpoint[0]*rP[2][1] - rP[0][1]
        A[2][2] = rpoint[0]*rP[2][2] - rP[0][2]

        A[3][0] = rpoint[1]*rP[2][0] - rP[1][0]
        A[3][1] = rpoint[1]*rP[2][1] - rP[1][1]
        A[3][2] = rpoint[1]*rP[2][2] - rP[1][2]

        B[0][0] = -(lpoint[0]*lP[2][3] - lP[0][3])
        B[1][0] = -(lpoint[1]*lP[2][3] - lP[1][3])
        B[2][0] = -(rpoint[0]*rP[2][3] - rP[0][3])
        B[3][0] = -(rpoint[1]*rP[2][3] - rP[1][3])

        print 'A-Matrix: \n', A, '\n'
        print 'B-Matrix: \n', B, '\n'
        cv2.solve(A, B, X, cv2.DECOMP_SVD)
        print 'Solution: \n', X, '\n'
        return X

    def image_cb(self, limage, rimage):
        try:
            x_sol = []
            l_img = self.bridge.imgmsg_to_cv2(limage, "bgr8")
            r_img = self.bridge.imgmsg_to_cv2(rimage, "bgr8")

            l_pts = self.find_contours(l_img, 'l')
            r_pts = self.find_contours(r_img, 'r')

            if l_pts is not None and r_pts is not None:
                assert len(l_pts) == len(r_pts), 'Lists have different dimensions'

                for i, pt in enumerate(l_pts):
                    print 'Attempting to triangulate points:'
                    x_sol.append(self.triangulate_Linear_LS(l_pts[i], r_pts[i], self.lP, self.rP))

                for i, sol in enumerate(x_sol):
                    if sol[2][0] < 0:
                        print sol
                        rospy.logwarn('Invalid Point')
                        break    # Invalid point
                    else:
                        print 'Plotting: ', sol[0], sol[1], sol[2]
                        point = PointStamped(header=Header(stamp=rospy.Time.now(),
                                                 frame_id='/stereo_front/left'), point=Point(sol[0],sol[1],sol[2]))
                        self.gate.point_publisher.publish(point)
                print '\n'
            else:
                pass
        except CvBridgeError as e:
            print e

if __name__ =='__main__':
    rospy.init_node('gate_finder', anonymous=False)
    sg = Gate()
    gf = GateFinder(sg)
    gf.gen_gate_kernel()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"
    cv2.destroyAllWindows()
