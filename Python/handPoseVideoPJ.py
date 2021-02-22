import cv2
import time
import numpy as np
import serial
import sys

# Pompilio - Comunicação com o Arduino
#f:\02_MundoTudoFacil\01_Projetos\011_Deep_Learning\opencv_GPU_Intel\opencvGpuIntel\build\setup_vars_opencv4.cmd
#cd "F:\02_MundoTudoFacil\01_Projetos\011_Deep_Learning\001_HandPose\HandPose"
#python handPoseVideoPJ.py
port = 'COM12' # Porta seria do Arduino
temArduino = True

try:
    ard = serial.Serial(port,9600,timeout=.1)
except serial.serialutil.SerialException:
    temArduino = False
    print("O Arduino não está conectado")


protoFile = "hand/pose_deploy.prototxt"
weightsFile = "hand/pose_iter_102000.caffemodel"
nPoints = 22
POSE_PAIRS = [ [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[0,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],[15,16],[0,17],[17,18],[18,19],[19,20] ]
#nPoints = 9
#POSE_PAIRS = [ [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8] ]
showAllPoints = 0

threshold = 0.35

#input_source = "asl.mp4"
print(cv2.__version__)
print("Aguarde...")
cap = cv2.VideoCapture(1) # Dispositivo de caputra de vídeo
#cap = cv2.VideoCapture("asl.mp4") # Para teste em arquivo de vídeo

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

hasFrame, frame = cap.read()

frameWidth = frame.shape[1]
frameHeight = frame.shape[0]

aspect_ratio = frameWidth/frameHeight

#inHeight = 184 #368 - Aumenda a velocidade de diminui a qualidade
inHeight = 368
inWidth = int(((aspect_ratio*inHeight)*8)//8)

vid_writer = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (frame.shape[1],frame.shape[0]))
print("Carregando rede...")
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
#net = cv2.dnn.readNet(protoFile, weightsFile)


net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_HALIDE)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_INFERENCE_ENGINE)
#net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL_FP16)
#net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)



print("Iniciando captura")
k = 0
pFinger1 = []
pFinger2 = []

sys.stdout = open("output.txt", "w")

while 1:
    k+=1
    t = time.time()
    hasFrame, frame = cap.read()
    frameCopy = np.copy(frame)
    if not hasFrame:
        cv2.waitKey()
        break

    inpBlob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    net.setInput(inpBlob)

    output = net.forward()

    #print("forward = {}".format(time.time() - t))

    # Empty list to store the detected keypoints
    points = []
    
    for i in range(nPoints):
        # confidence map of corresponding body's part.
        probMap = output[0, i, :, :]
        probMap = cv2.resize(probMap, (frameWidth, frameHeight))

        # Find global maxima of the probMap.
        minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
        
        if prob > threshold :
            cv2.circle(frameCopy, (int(point[0]), int(point[1])), 6, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.putText(frameCopy, "{}".format(i), (int(point[0]), int(point[1])), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2, lineType=cv2.LINE_AA)

            # Add the point to the list if the probability is greater than the threshold
            points.append((int(point[0]), int(point[1])))

            # Pompilio - Suavização
            if (i == 5):
                if (pFinger1):
                    pFinger1 = (int(.3 * float(pFinger1[0]) + .7 * point[0]), int(.3 * float(pFinger1[1]) + .7 * point[1]))
                else:
                    pFinger1 = (int(point[0]), int(point[1]))
            if (i == 8):
                if (pFinger2):
                    pFinger2 = (int(.3 * float(pFinger2[0]) + .7 * point[0]), int(.3 * float(pFinger2[1]) + .7 * point[1]))
                else:
                    pFinger2 = (int(point[0]), int(point[1]))                    
                    
            
        else :
            points.append(None)

    # Pompilio calculo da distancia
    if points[5] and points[8] and points[0]:
        print(str(k) + ";" + str(int(cv2.norm(points[5], points[8],cv2.NORM_L2))) + ";" + str(int(cv2.norm(points[0], points[5],cv2.NORM_L2))))
        # Pompilio - Comunicação com o Arduino
        iValue = int(cv2.norm(points[5], points[8],cv2.NORM_L2))
        iValue = 180 - (iValue - 63) * 80 / 187
        strMSG = str(iValue)
        if (temArduino):
            ard.write(strMSG.encode())
    else:
        print(str(k) + ";-1;-1")



    # Draw Skeleton
    iPoint = 0
    for pair in POSE_PAIRS:
        partA = pair[0]
        partB = pair[1]
        #print(partA)
        iPoint += 1
        c = (0, 255, 255)
        if (iPoint > 4 and iPoint < 9): c = (255, 0, 0)
        if points[partA] and points[partB] and (iPoint > 4 or showAllPoints == 0):
            cv2.line(frame, points[partA], points[partB], c, 2, lineType=cv2.LINE_AA)
            cv2.circle(frame, points[partA], 5, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
            cv2.circle(frame, points[partB], 5, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)

            # Pompilio - Desenha Suavização
            if (partA == 5 or partB == 5):
                cv2.circle(frame, pFinger1, 5, (0, 255, 0), thickness=-1, lineType=cv2.FILLED)
                if (pFinger2): cv2.line(frame, pFinger1, pFinger2, (0, 255, 0), 1, lineType=cv2.LINE_AA)
            if (partA == 8 or partB == 8):
                cv2.circle(frame, pFinger2, 5, (0, 255, 0), thickness=-1, lineType=cv2.FILLED)
                if (pFinger1):
                    cv2.line(frame, pFinger1, pFinger2, (0, 255, 0), 1, lineType=cv2.LINE_AA)
                    dist = str(int(cv2.norm(pFinger1, pFinger2, cv2.NORM_L2)))
                    cv2.putText(frame, dist, (int((pFinger1[0] + pFinger2[0])/2)-5, int((pFinger1[1] + pFinger2[1])/2)-5), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 0), 1, lineType=cv2.LINE_AA)


        


    #print("Time Taken for frame = {}".format(time.time() - t))

    #cv2.putText(frame, "time taken = {:.2f} sec".format(time.time() - t), (50, 50), cv2.FONT_HERSHEY_COMPLEX, .8, (255, 50, 0), 2, lineType=cv2.LINE_AA)
    #cv2.putText(frame, "Hand Pose using OpenCV", (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 50, 0), 2, lineType=cv2.LINE_AA)
    cv2.imshow('Output-Skeleton', frame)
    # cv2.imwrite("video_output/{:03d}.jpg".format(k), frame)
    key = cv2.waitKey(1)
    if key == 32:
        if showAllPoints == 0:
            nPoints = 9
            POSE_PAIRS = [ [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8] ]
            showAllPoints = 1
        else:
            nPoints = 22
            POSE_PAIRS = [ [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[0,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],[15,16],[0,17],[17,18],[18,19],[19,20] ]
            showAllPoints = 0


    if key == 27:
        break

    #print("total = {}".format(time.time() - t))

    vid_writer.write(frame)

sys.stdout.close()

vid_writer.release()
