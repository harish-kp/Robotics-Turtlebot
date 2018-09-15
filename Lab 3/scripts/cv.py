def get_image(camera):
# read is the easiest way to get a full image out of a VideoCapture object.
    retval, im = camera.read()
    return im


camera = cv2.VideoCapture(0)

for i in range(ramp_frames):
    temp = get_image(camera)
print("Taking image...")

camera_capture = get_image(camera)
#camera_capture = cv2.resize(camera_capture,(368,500))
file = "D:\\New folder\\projrough\\test_image.jpg"