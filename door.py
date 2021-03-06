# 点
import logging
import cv2
import numpy as np
class Point(object):

    def __init__(self, x, y):
        self.x, self.y = x, y

# 向量
class Vector(object):

    def __init__(self, start_point, end_point):
        self.start, self.end = start_point, end_point
        self.x = end_point.x - start_point.x
        self.y = end_point.y - start_point.y

ZERO = 1e-9

def negative(vector):
    return Vector(vector.end, vector.start)

def vector_product(vectorA, vectorB):
    return vectorA.x * vectorB.y - vectorB.x * vectorA.y

def isIntersected(A, B, C, D):
    AC = Vector(A, C)
    AD = Vector(A, D)
    BC = Vector(B, C)
    BD = Vector(B, D)
    CA = negative(AC)
    CB = negative(BC)
    DA = negative(AD)
    DB = negative(BD)

    return (vector_product(AC, AD) * vector_product(BC, BD) <= ZERO) \
        and (vector_product(CA, CB) * vector_product(DA, DB) <= ZERO)


class door:
    ifTakePhoto = 0
    stillAtDoor=[]# 防止老是拍照
    lastPoints = []
    LastNumPeople = 0
    # (x1,y1).------.(x2,y2)
    #(0,0)--(1,0)
    #(1,0)--(1,1)
    def initCanvas(self):
        if (not self.ifPlot):
            raise SystemError
        self.font = cv2.FONT_HERSHEY_SIMPLEX 
        self.window_x = 480
        self.window_y = int(self.window_x/6*10)
        self.pxPerMeter = int(self.window_x/6)
        self.image = np.zeros((self.window_x, self.window_y, 3), np.uint8)
        self.image.fill(255)
        cv2.circle(self.image,(int(self.window_y/2),0),5,(0,0,255),6)
        cv2.line(self.image, (int(self.window_y/2*(1+self.PointA.x/5)),int(self.PointA.y*self.pxPerMeter)), (int(self.window_y*(self.PointB.x+5)/10),int(self.PointB.y*self.pxPerMeter)), (0,255,0), 3)

    def __init__(self, x1:float,y1:float,x2:float,y2:float,ifPlot:bool):
        self.PointA = Point(x1,y1)
        self.PointB = Point(x2,y2)
        self.ifPlot = False
        if (ifPlot):
            self.ifPlot = True
            self.initCanvas()

    def takePhoto(self,x:list,y:list)->int:
        self.ifTakePhoto = 0
        if len(x) != self.LastNumPeople:
            self.LastNumPeople = len(x)
            self.stillAtDoor = [False] * len(x) # 填充
            self.lastPoints = [x,y]
            return False # drop frame if num people changed
        # 对每个人计算(没有去判断每个人的位置，可能会发生突变，就假装没有这个问题好了)
        for i in range(self.LastNumPeople):
            if (self.stillAtDoor[i]):#上次计算认为这个人还在门边
                # TODO:计算当前点到门的距离，超过阈值就释放stillAtDoor
                # 如果没超过，continue
                pass
            PointC = Point(self.lastPoints[0][i],self.lastPoints[1][i])
            PointD = Point(x[i],y[i])
            if (isIntersected(self.PointA,self.PointB,PointC,PointD)):# 前代码保证是第一轮拍照
                if (PointD.y < (self.PointA.y + self.PointB.y)/2):
                    # 进门ing
                    logging.warning("Person "+str(i)+" triggered Enter Door")
                    self.stillAtDoor[i] = True
                    self.ifTakePhoto = 1
                else:
                    # 出门ing
                    logging.warning("Person "+str(i)+" triggered Exiting Door")
                    self.stillAtDoor[i] = True
                    self.ifTakePhoto = -1
            pass
        for i in range(self.LastNumPeople):
            self.lastPoints[0][i] = x[i]
            self.lastPoints[1][i] = y[i]
            pass
        return self.ifTakePhoto
    def drawPeople(self,x:list,y:list):
        if (not self.ifPlot):
            raise SystemError
        if ((len(x)!=len(y)) or (len(x)==0)):
            logging.error("Called drawPeople with wrong data")
            return self.image
        self.initCanvas()
        for peopleNum in range(len(x)):
            cv2.circle(self.image,(int(self.window_y/2 + x[peopleNum]*self.pxPerMeter),int(y[peopleNum]*self.pxPerMeter)),5,(0,0,255),6)
            cv2.putText(self.image, str(f"Suspect:{peopleNum}"), (int(self.window_y/2 + x[peopleNum]*self.pxPerMeter)+5,int(y[peopleNum]*self.pxPerMeter)+15), self.font, 0.5, (159, 159, 255), 2)
        return self.image





