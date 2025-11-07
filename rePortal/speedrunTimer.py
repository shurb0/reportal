from direct.task.Timer import Timer
import os
import json


class SpeedrunTimer(Timer):
    """
    In game speedrun timer that can save run times
    """
    def __init__(self):
        super().__init__(Timer)
        self.currentSession = 0
        self.currentLevel = -1
        
        with open(os.path.join(os.path.dirname(__file__), "times.json"), "r") as file:
            timesJsonString = file.read()
            file.close()
        self.timesJson = json.loads(timesJsonString)
        
        for session in self.timesJson["sessions"]:
            if session["sessionID"] >= self.currentSession:
                self.currentSession = session["sessionID"] + 1
        
        self.sessionDict = {"session": self.currentSession,"times":[]}
    
    def startLevel(self, levelName:str):
        self.currentLevel = levelName
        self.restart()
    
    def endLevel(self):
        self.stop()
        self.sessionDict["times"].append({"level": self.currentLevel,"time": self.getT()})
        
