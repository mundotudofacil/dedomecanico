import cv2
import time
import numpy as np


protoFile = "hand/pose_deploy.prototxt"
weightsFile = "hand/pose_iter_102000.caffemodel"
#nPoints = 22
#POSE_PAIRS = [ [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[0,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],[15,16],[0,17],[17,18],[18,19],[19,20] ]
nPoints = 9
POSE_PAIRS = [ [0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8] ]
showAllPoints = 1


threshold = 0.2


input_source = "asl.mp4"
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


hasFrame, frame = cap.read()

frameWidth = frame.shape[1]
frameHeight = frame.shape[0]

aspect_ratio = frameWidth/frameHeight

#inHeight = 184 #368
inHeight = 368
inWidth = int(((aspect_ratio*inHeight)*8)//8)

vid_writer = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (frame.shape[1],frame.shape[0]))

net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
k = 0
pFinger1 = []
pFinger2 = []
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
    if points[5] and points[8]:
        print(cv2.norm(points[5], points[8],cv2.NORM_L2))


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
            if (partA == 5):
                cv2.circle(frame, pFinger1, 5, (0, 255, 0), thickness=-1, lineType=cv2.FILLED)
            if (partA == 8):
                cv2.circle(frame, pFinger1, 9, (0, 255, 0), thickness=-1, lineType=cv2.FILLED)


        


    #print("Time Taken for frame = {}".format(time.time() - t))

    # cv2.putText(frame, "time taken = {:.2f} sec".format(time.time() - t), (50, 50), cv2.FONT_HERSHEY_COMPLEX, .8, (255, 50, 0), 2, lineType=cv2.LINE_AA)
    # cv2.putText(frame, "Hand Pose using OpenCV", (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 50, 0), 2, lineType=cv2.LINE_AA)
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

vid_writer.release()
