import math
from math import atan2, degrees
from typing import Tuple
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP, ROBOT_NAMES
import utils


blueTeamNames = ['B1', 'B2', 'B3']
yellowTeamNames = ['Y1', 'Y2', 'Y3']


class MyRobot2(RCJSoccerRobot):

    left = 0
    right = 0
    role = 'shooter'
    attackerFlag = True
    lastBallPosition = None
    ballPostion = None
    ballLackOfProgressCounter=0
    ballLackOfProgress=False

    def newDataProccessing(self):
        self.rawData = self.get_new_data()
        self.position = self.rawData[self.name]['x'], self.rawData[self.name]['y']
        self.lastBallPosition=self.ballPostion
        self.ballPostion = self.rawData['ball']['x'], self.rawData['ball']['y']

        self.distances = {}
        for i in ROBOT_NAMES:
            self.distances[i] = utils.getDistance(
                (self.rawData[i]['x'], self.rawData[i]['y']), (self.rawData['ball']['x'], self.rawData['ball']['y']))

        self.orientations = {}
        for i in ROBOT_NAMES:
            self.orientations[i] = math.degrees(self.rawData[i]['orientation'])

        self.positions={}
        for i in ROBOT_NAMES:
            self.positions[i]=self.rawData[i]['x'], self.rawData[i]['y']

        self.distance = self.distances[self.name]

        self.orientation = utils.correctAngle(self.orientations[self.name])
        if self.team=='B':
            self.orientation -= 180
            self.orientation=utils.correctAngle(self.orientation)

        self.ballAngle = self.get_angles(
            self.rawData['ball'], self.rawData[self.name])[0]

    def move(self, position):
        x, y = position
        angle = utils.correctAngle(self.get_angles(
            {'x': x, 'y': y}, self.rawData[self.name])[0])
        direction = utils.getDirectionBy4(angle)
        value = self.distance/0.15*1.5
        if value < 1:
            value = 1
        if angle < 90 or angle > 270:
            if direction == 1:
                self.left = -10
                self.right = -value
            elif direction == 3:
                self.left = -value
                self.right = -10
            elif direction == 0:
                self.left = -10
                self.right = -10
        else:
            if direction == 1:
                self.left = 10
                self.right = value
            elif direction == 3:
                self.left = value
                self.right = 10
            elif direction == 2:
                self.left = 10
                self.right = 10
        if self.left>10:
            self.left=10
        if self.right>10:
            self.right=10
        if self.left<-10:
            self.left=-10
        if self.right<-10:
            self.right=-10

    def stop(self):
        self.left = 0
        self.right = 0

    def getRouteAngle(self, pos1: iter, pos2: iter) -> float:
        xDiff = pos2[0] - pos1[0]
        yDiff = pos2[1] - pos1[1]
        return utils.correctAngle(degrees(atan2(yDiff, xDiff)))

    def checkLackOfProgress(self):
        error=0.012
        if self.ballPostion!=None and self.lastBallPosition!=None:
            if abs(self.ballPostion[0]-self.lastBallPosition[0])<=error and abs(self.ballPostion[1]-self.lastBallPosition[1])<=error:
                self.ballLackOfProgressCounter+=1
            else:
                self.ballLackOfProgressCounter=0
                self.ballLackOfProgress=False
            if self.ballLackOfProgressCounter>=30:
                self.ballLackOfProgress=True

    def getSegment(self, position:Tuple[float, float]):
        return position[0]//0.35+2, position[1]//0.3+2

    def getCrowdedSegment(self):
        segmentCounter={}
        for i in ROBOT_NAMES:
            segment=str(self.getSegment(self.positions[i]))
            if segment in segmentCounter.keys():
                segmentCounter[segment]+=1
            else:
                segmentCounter[segment]=1
        segment=str(self.getSegment(self.ballPostion))
        if segment in segmentCounter.keys():
            segmentCounter[segment]+=1
        else:
            segmentCounter[segment]=1
        sorted(segmentCounter.items(), key=lambda x:x[1])

    def run(self):
        if self.team=='B':
            blueTeamNames.remove(self.name)
        else:
            yellowTeamNames.remove(self.name)

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():

                self.newDataProccessing()
                # print(self.name, self.role)
                # d={}
                # for i in blueTeamNames:
                #     d[i]=self.distances[i]
                # d[self.name]=self.distance
                # ls=sorted(d.items(), key=lambda x:x[1])

                self.checkLackOfProgress()


                if self.team=='B':
                    if self.role=='shooter':
                        self.checkLackOfProgress()
                        if self.ballLackOfProgress:
                            self.move((0.2, 0))
                            if 0.15<self.position[0]<0.25 and -0.05<self.position[1]<0.05:
                                if 100>self.orientation>80:
                                    self.stop()
                                elif 90>self.orientation>0 or 360>self.orientation>270:
                                    self.left=-5
                                    self.right=5
                                else:
                                    self.left=5
                                    self.right=-5
                        else:
                            if self.ballPostion[0]<0.55:
                                self.move(self.ballPostion)
                            else:
                                self.stop()
                    elif self.role=='attacker':
                        if self.ballPostion[0]<0.55:
                            self.move(self.ballPostion)
                        else:
                            self.stop()
                    elif self.role=='goalkeeper':
                        if self.position[0]<0.65:
                            self.move((0.7, 0))
                        else:
                            if self.orientation>170 and self.orientation<190:
                                if self.ballPostion[1]>self.position[1]:
                                    self.left=-10
                                    self.right=-10
                                elif self.ballPostion[1]<self.position[1]:
                                    self.left=10
                                    self.right=10
                                else:
                                    self.stop()
                                
                            else:
                                if self.orientation>180 and self.orientation<360:
                                    self.left=-5
                                    self.right=5
                                else:
                                    self.left=5
                                    self.right=-5
                                    

                            
                else:
                    if self.role=='shooter':
                        self.checkLackOfProgress()
                        if self.ballLackOfProgress:
                            self.move((-0.2, 0))
                            if -0.15>self.position[0]>-0.25 and -0.05<self.position[1]<0.05:
                                if 100>self.orientation>80:
                                    self.stop()
                                elif 90>self.orientation>0 or 360>self.orientation>270:
                                    self.left=-5
                                    self.right=5
                                else:
                                    self.left=5
                                    self.right=-5
                        else:
                            if self.ballPostion[0]>-0.55:
                                self.move(self.ballPostion)
                            else:
                                self.stop()
                    elif self.role=='attacker':
                        if self.ballPostion[0]>-0.55:
                            self.move(self.ballPostion)
                        else:
                            self.stop()
                    elif self.role=='goalkeeper':
                        if self.position[0]>-0.65:
                            self.move((-0.7, 0))
                        else:
                            if self.orientation>350 or self.orientation<10:
                                if self.ballPostion[1]>self.position[1]:
                                    self.left=-10
                                    self.right=-10
                                elif self.ballPostion[1]<self.position[1]:
                                    self.left=10
                                    self.right=10
                                else:
                                    self.stop()
                                
                            else:
                                if self.orientation>0 and self.orientation<180:
                                    self.left=-5
                                    self.right=5
                                else:
                                    self.left=5
                                    self.right=-5
                        

                self.left_motor.setVelocity(self.left)
                self.right_motor.setVelocity(self.right)