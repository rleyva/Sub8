import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import feature_drawing

FLANN_INDEX_KDTREE = 0

class FeatureHandler():
    def __init__(self, image_path):
        self.detector = cv2.SIFT()
        self.trained_image = cv2.imread(image_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        self.flann = cv2.FlannBasedMatcher(dict(algorithm = FLANN_INDEX_KDTREE, trees = 5), {})
        self.kp, self.des =  self.detector.detectAndCompute(self.trained_image, None)

    def flann_query(self, img):
        kp2, des2 = self.detector.detectAndCompute(img, None)
        self.matches = self.flann.match(self.des, des2)
        self.good_matches = []

        for m in self.matches:
            if m.distance < 3 * min(self.matches):
                self.good_matches.append(m)
        return feature_drawing.drawMatches(self.trained_image, self.kp, img, kp2, self.good_matches)


class SURFTester():
    def __init__(self, surf_handler):
        self.surf_handler = surf_handler
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/down/left/image_rect_color', Image, self.image_cb)

    def image_cb(self, img):
        try:
            img_1 = self.bridge.imgmsg_to_cv2(img, "bgr8")
            img_1 = cv2.cvtColor(img_1, cv2.COLOR_BGR2GRAY)
            match_img = self.surf_handler.flann_query(img_1)
            cv2.imshow('Matches', match_img)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node('SURF_tester', anonymous=True)
    f = FeatureHandler('templates/anchor_danforth_black.png')
    st = SURFTester(f)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")