import cv2

class Camera:
    def __init__(self, COM) -> None:
        self.COM = COM
        self.cam = cv2.VideoCapture(COM)

    def get_img(self):
        ret, frame = self.cam.read()
        if ret:
            return frame
        else:
            return None
