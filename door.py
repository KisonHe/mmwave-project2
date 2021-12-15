# 点
import logging
from sys import flags

from cv2 import FlannBasedMatcher
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

def is_intersected(A, B, C, D):
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
    if_take_photo = False
    still_at_door=[]# 防止老是拍照
    lastPoints = []
    LastNumPeople = 0
    # (x1,y1).------.(x2,y2)
    def __init__(self, x1:float,y1:float,x2:float,y2:float):
        self.PointA = Point(x1,y1)
        self.PointB = Point(x2,y2)

    def take_photo(self,x:list,y:list):
        self.if_take_photo = False
        if len(x) != self.LastNumPeople:
            self.LastNumPeople = len(x)
            self.still_at_door = [False] * len(x) # 填充
            self.lastPoints = [x,y]
            return False # drop frame if num people changed
        # 对每个人计算(没有去判断每个人的位置，可能会发生突变，就假装没有这个问题好了)
        for i in range(self.LastNumPeople):
            if (self.still_at_door[i]):#上次计算认为这个人还在门边
                # TODO:计算当前点到门的距离，超过阈值就释放still_at_door
                # 如果没超过，continue
                pass
            PointC = Point(self.lastPoints[0][i],self.lastPoints[1][i])
            PointD = Point(x[i],y[i])
            if (is_intersected(self.PointA,self.PointB,PointC,PointD)):# 前代码保证是第一轮拍照
                logging.warning("Person "+str(i)+" triggered")
                self.still_at_door[i] = True
                self.if_take_photo = True
            pass
        for i in range(self.LastNumPeople):
            self.lastPoints[0][i] = x[i]
            self.lastPoints[1][i] = y[i]
            pass
        return self.if_take_photo





