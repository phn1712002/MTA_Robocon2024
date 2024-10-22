# python auto_fire.py --weights ./model_fine_tuning/model_robocon24.pt --camera 0 --port COM1 --buad 9600 --windows True --print True 
# python auto_fire.py --weights ./model_robocon24.pt --camera 0 --port /dev/ttyACMA --buad 9600
# python ./yolov5/detect.py --weights ./model_fine_tuning/model_robocon24.pt --source 1 

import argparse
import torch
import cv2
import serial
import os

parser = argparse.ArgumentParser()
parser.add_argument("--weights", type=str, default= "./model_robocon24.pt")
parser.add_argument("--camera", type=int, default= 0)
parser.add_argument("--port", type=str, default= "/dev/ttyUSB0")
parser.add_argument("--buad", type=int, default= 9600)
parser.add_argument("--windows", type=bool, default= False)
parser.add_argument("--print", type=bool, default= False)
args = parser.parse_args()

# Clear
if args.windows:
  os.system('cls')
else:
  os.system('clear')

# Fix error windows
if args.windows:
  import pathlib  
  temp = pathlib.PosixPath
  pathlib.PosixPath = pathlib.WindowsPath

# Load model
path_model = args.weights
model = torch.hub.load('ultralytics/yolov5', 'custom', path=path_model, force_reload=True)
prediction_threshold = 0.8
print("Load model complete!")

# Load camera
com_camera = args.camera
cam = cv2.VideoCapture(com_camera)
print("Load camera complete!")

# Serial
port = args.port
buad = args.buad
ser = serial.Serial(port, buad, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
print("Load serial complete!")

# Fire
area_fire_max = 500
area_fire_min = 400
key_fire = "1"
key_no_fire = "0"
fire_check = False


while True:
    ret, frame = cam.read()
    if ret: 
      result = model(frame)
      if args.print: print(result)
      predictions = result.pandas().xyxy[0]
      
      if(predictions.shape[0] > 0):
        # Get confidence max
        id = predictions['confidence'].idxmax()
        confidence = max(predictions['confidence'])
        
        if confidence > prediction_threshold:
          print("-" * 100)
          row_with_max_confidence = predictions.loc[id]

          # Get xmin, ymin, xmax, ymax from object confidence max
          xmin = row_with_max_confidence['xmin']
          ymin = row_with_max_confidence['ymin']
          xmax = row_with_max_confidence['xmax']
          ymax = row_with_max_confidence['ymax']

          # Calc Area
          width = xmax - xmin
          height = ymax - ymin
          area = width * height
          print(f"Area: {area}")
          print(f"Confidence: {confidence}")
          
          #if area < area_fire_max and area > area_fire_min:
          # print(f"Fire !")
          # ser.write(bytes(key_fire, 'utf-8'))
          # fire = True
          # fire_check = True
          # print("-" * 100)
          
    # if not fire_check: 
    #   print(f"No Fire !")
    #   ser.write(bytes(key_no_fire, 'utf-8'))