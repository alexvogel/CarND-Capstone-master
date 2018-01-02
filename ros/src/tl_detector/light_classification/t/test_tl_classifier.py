import cv2

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from tl_classifier import TLClassifier

# path to the frozen tensorflow model (protobuf)
model_file = currentdir + "/../../models/frozen_classifier_model.pb"

# test image is a cutout of a detected traffic light
img_cutout = cv2.imread(currentdir + "/sample_tl_cutout.png")
print ("shape of input image: " + str(img_cutout.shape))

# create classifier
light_classifier = TLClassifier(model_file)

# classify cutout image
detected_color_id = light_classifier.get_classification(img_cutout)

# print result
print("detected color-id is: " + str(detected_color_id))

