import cv2

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from tl_classifier import TLClassifier

# path to the frozen tensorflow model (protobuf)
model_file = currentdir + "/../../models/frozen_classifier_model.pb"

# create classifier by reading the frozen model file
light_classifier = TLClassifier(model_file)

# test images (cutout of a detected traffic light)
image_files = ["sample_tl_cutout_yellow_1.png", "traffic_light_red_1.png", "traffic_light_green_1.png", "traffic_light_red_2.png"]

# classify every sample image and print the results
for image_file in image_files:
    
    image = cv2.imread(currentdir + "/" + image_file)
    print("----")
    print ("classifying image " + image_file)
    print ("shape of input image: " + str(image.shape))

    # classify cutout image
    detected_color_id = light_classifier.get_classification(image)

    # print result
    print("detected color-id is: " + str(detected_color_id))

