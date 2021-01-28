import open3d as o3d
import numpy as np
import sys
import cv2
import tuberz
from enum import IntEnum

try:
    sys.path.append('/usr/local/python')
    from openpose import pyopenpose as op
except ImportError as e:
    print(
        'Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
    raise e

class Joint(IntEnum):
    NOSE = 0
    NECK = 1
    RSHOULDER = 2
    RELBOW = 3
    RWRIST = 4
    LSHOULDER = 5
    LELBOW = 6
    LWRIST = 7
    MIDHIP = 8
    RHIP = 9
    RKNEE = 10
    RANKLE = 11
    LHIP = 12
    LKNEE = 13
    LANKLE = 14
    REYE = 15
    LEYE = 16
    REAR = 17
    LEAR = 18
    LBIGTOE = 19
    LSMALLTOE = 20
    LHEEL = 21
    RBIGTOE = 22
    RSMALLTOE = 23
    RHEEL = 24


class PoseEstimator:
    def __init__(self):
        # parameters for pose estimation
        self.params = dict()
        self.params["model_folder"] = "models/"
        self.params["face"] = False
        self.params["hand"] = True
        self.params["number_people_max"] = 1
        self.params["num_gpu"] = 1

        # Starting OpenPose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()

    def process_image(self, image):
        datum = op.Datum()
        datum.cvInputData = image
        self.opWrapper.emplaceAndPop([datum])
        return datum

    def display_pose(self, datum):
        cv2.imshow("OpenPose 1.6.0 - Tutorial Python API", datum.cvOutputData)

    def tube_worthy(self, keypoints):
        try:
            lwrist = keypoints.poseKeypoints[0,Joint.LWRIST,1]
            rwrist = keypoints.poseKeypoints[0, Joint.RWRIST, 1]
            nose = keypoints.poseKeypoints[0, Joint.NOSE, 1]
            return lwrist < nose and rwrist <nose
        except:
            return False



config = o3d.io.AzureKinectSensorConfig()

device = 0

align_depth_to_color = True

sensor = o3d.io.AzureKinectSensor(config)

pose_estimator = PoseEstimator()

if not sensor.connect(device):
    raise RuntimeError('Failed to connect to sensor')

while True:
    rgbd = sensor.capture_frame(True)
    if rgbd is not None:
        depth = np.asarray(rgbd.depth)
        color = np.asarray(rgbd.color)

        keypoints = pose_estimator.process_image(color)
        if len(keypoints.poseKeypoints.shape) > 0 and pose_estimator.tube_worthy(keypoints):
            tuberz.tube()


