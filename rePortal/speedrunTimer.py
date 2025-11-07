from direct.task import Task
from panda3d.core import ClockObject
from math import floor
import os
import json

globalClock = ClockObject.getGlobalClock()


class SpeedrunTimer:
    """
    In game speedrun timer that can save run times
    """
    def __init__(self):
        self.currentSession = 0
        self.currentLevel = "none"
        self.running = False
        self.currentTime = 0
        
        with open(os.path.join(os.path.dirname(__file__), "times.json"), "r") as file:
            timesJsonString = file.read()
            file.close()
        self.timesJson = json.loads(timesJsonString)
        
        for session in self.timesJson["sessions"]:
            if session["sessionID"] >= self.currentSession:
                self.currentSession = session["sessionID"] + 1
        
        self.sessionDict = {"session": self.currentSession,"times":[]}
    
    def start(self,t:int = 0, level:str = "none"):
        self.running = True
        self.currentLevel = level
        self.currentTime = t
        taskMgr.add(self.updateTimer,"timer", sort=5) # noqa # type:ignore
    
    def pause(self):
        self.running = False
    
    def resume(self):
        self.running = True
        taskMgr.add(self.updateTimer,"timer", sort=5) # noqa # type:ignore
    
    def end(self, output=True):
        self.running = False
        if output:
            self.sessionDict["times"].append({"level":self.currentLevel,"time":self.currentTime})
        print(self.sessionDict)
    
    def getT(self, formatted=False):
        if not formatted:
            return str(self.currentTime)
        h = round(self.currentTime / 3600)
        m = round((self.currentTime % 3600) / 60)
        s = round((self.currentTime % 60),2)
        if h == 0:
            return f"{m}:{s:.2f}"
        return f"{h}:{m}:{s:.2f}"
    
    def setT(self,t):
        self.currentTime = t
    
    def updateTimer(self, task):
        if not self.running:
            return Task.done
        self.currentTime += globalClock.getDt()
        return Task.cont
