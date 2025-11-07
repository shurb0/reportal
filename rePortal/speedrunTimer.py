from direct.task.Timer import Timer
from panda3d.core import ClockObject
import os
import json

globalClock = ClockObject.getGlobalClock()


class SpeedrunTimer:
    """
    In game speedrun timer that can save run times
    """
    def __init__(self):
        self.currentSession = 0
        self.currentLevel = -1
        self.started = 0
        
        with open(os.path.join(os.path.dirname(__file__), "times.json"), "r") as file:
            timesJsonString = file.read()
            file.close()
        self.timesJson = json.loads(timesJsonString)
        
        for session in self.timesJson["sessions"]:
            if session["sessionID"] >= self.currentSession:
                self.currentSession = session["sessionID"] + 1
        
        self.sessionDict = {"session": self.currentSession,"times":[]}
    
    def start(self,t:int,level:str):
        pass
    
    def pause(self):
        pass
    
    def end(self,output=True):
        pass
    
    def getT(self,formatted=False):
        pass
    
    def setT(self):
        pass
