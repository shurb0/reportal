from direct.showbase.DirectObject import DirectObject
import os
import json


class InputWatcher(DirectObject):
    def __init__(self):
        with open(os.path.join(os.path.dirname(__file__), "binds.json"), "r") as file:
            self.bindJsonString = file.read()
            file.close()
        self.bindJson = json.loads(self.bindJsonString)
        self.bindDict = {}
        self.wheelFunctions = []
        for function in self.bindJson:
            if function[0] == "+":
                self.bindDict[function[1:]] = False
            for inputEvent in self.bindJson[function]:
                if inputEvent[:2] == "wh":
                    self.wheelFunctions.append(function)
                    self.bindDict[function[1:] + "-wheel"] = False
                    self.accept(inputEvent,self.process,[function + "-wheel"])
                else:
                    self.accept(inputEvent,self.process,[function])
    
    def process(self,function):
        if function[-6:] == "-wheel":
            self.bindDict[function[1:-6]] = True
            self.bindDict[function[1:] + "-wheel"] = True
        if function[0] == "+":
            self.bindDict[function[1:]] = True
        elif function[0] == "-":
            self.bindDict[function[1:]] = False
    
    def fixWheel(self):
        for function in self.wheelFunctions:
            if self.bindDict[function[1:] + "-wheel"]:
                self.bindDict[function[1:]] = False
                self.bindDict[function[1:] + "-wheel"] = False
    
    def isSet(self,function):
        return self.bindDict[function]
