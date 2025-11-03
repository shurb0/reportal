import json
import panda3d.bullet as bt
from panda3d.core import Vec3, BitMask32
import os
from entities import FloorButton, Cube, LevelEnd


class LevelLoader:
    """
    Load level model, collision, entities from .json level file
    """
    def __init__(self,level:str,world,worldNP,loader,entities:list = []):
        self.levelName = level
        self.world = world
        self.worldNP = worldNP
        self.loader = loader
        self.entities = entities
        
        self.unpackJson()
        self.loadModel()
        self.loadCollision(self.levelDict)
        self.loadEntities()
        self.playerStartPos = self.levelDict["playerStartPos"]
        self.playerStartPos = Vec3(self.playerStartPos[0],self.playerStartPos[1],self.playerStartPos[2])
        self.playerStartHpr = self.levelDict["playerStartHpr"]
        self.playerStartHpr = Vec3(self.playerStartHpr[0],self.playerStartHpr[1],self.playerStartHpr[2])
        self.nextLevel = self.levelDict["nextLevel"]
        self.title = self.levelDict["levelTitle"]
        
    def unpackJson(self):
        self.levelColliderPath = os.path.dirname(__file__) + rf"/Assets/levels/{self.levelName}.json"
        with open(self.levelColliderPath,"r") as levelFile:
            self.levelColliderFile = levelFile.read()
            levelFile.close()
        self.levelDict = json.loads(self.levelColliderFile)
        
    def loadModel(self):
        hpr = self.levelDict["modelHpr"]
        self.levelModelNP = self.worldNP.attachNewNode("level")
        self.levelModel = self.loader.loadModel(self.levelDict["modelPath"])
        self.levelModel.reparentTo(self.levelModelNP)
        self.levelModel.setHpr(hpr[0],hpr[1],hpr[2])
        # self.levelModel.hide()
        
    def loadCollision(self,levelDict):
        for collisionBox in levelDict["collisionBoxes"]:
            dimensions = collisionBox["dimensions"]
            pos = collisionBox["pos"]
            if "hpr" in collisionBox:
                hpr = collisionBox["hpr"]
            shape = bt.BulletBoxShape(Vec3((dimensions[0]),dimensions[1],dimensions[2]))
            box = bt.BulletRigidBodyNode(collisionBox["desc"])
            box.addShape(shape)
            boxNP = self.levelModelNP.attachNewNode(box)
            boxNP.setPos(pos[0],pos[1],pos[2])
            if "hpr" in collisionBox:
                boxNP.setHpr(hpr[0],hpr[1],hpr[2])
            self.world.attach(boxNP.node())
            if collisionBox["desc"] == "portal collider":
                box.setName("portal collider")
                boxNP.setCollideMask(BitMask32.bit(1))  # portalable collider
            elif collisionBox["desc"] == "nonportal collider":
                box.setName("nonportal collider")
                boxNP.setCollideMask(BitMask32.bit(3))
    
    def loadEntities(self):
        self.linkedEntities = {}
        for i in range(self.levelDict["numLinks"]):
            self.linkedEntities[i + 1] = []
        for entity in self.levelDict["entities"]:
            if entity["type"] == "box":
                pos = Vec3(entity["pos"][0], entity["pos"][1], entity["pos"][2])
                hpr = Vec3(entity["hpr"][0], entity["hpr"][1], entity["hpr"][2])
                cube = Cube(self.world, self.levelModelNP, self.loader, pos, hpr)
                self.entities.append([cube, cube.boxCollisionNode, cube.cloneCollisionNode])
            elif entity["type"] == "floorButton":
                pos = Vec3(entity["pos"][0], entity["pos"][1], entity["pos"][2])
                hpr = Vec3(entity["hpr"][0], entity["hpr"][1], entity["hpr"][2])
                linkID = entity["linkID"]
                floorButton = FloorButton(self.world, self.levelModelNP, self.loader, pos, hpr, linkID)
                self.entities.append([floorButton, floorButton.baseCollisionNode])
                self.linkedEntities[linkID].append(floorButton)
            elif entity["type"] == "levelEnd":
                pos = Vec3(entity["pos"][0], entity["pos"][1], entity["pos"][2])
                hpr = Vec3(entity["hpr"][0], entity["hpr"][1], entity["hpr"][2])
                linkID = entity["linkID"]
                levelEnd = LevelEnd(self.world, self.levelModelNP, self.loader, pos, hpr, linkID)
                self.entities.append([levelEnd, levelEnd.levelEndTriggerNode])
                if linkID != 0:
                    self.linkedEntities[linkID].append(levelEnd)


class ModelLoader:
    
    """
    model loader for non level colliders
    not expected to be used since collision with triangle meshes is unstable
    but keeping it just in case
    """
    
    def make_collision_from_model(self,input_model, world):  # stolen code
        # tristrip generation from static models
        # generic tri-strip collision generator begins
        geom_nodes = input_model.findAllMatches('**/+GeomNode')
        geom_nodes = geom_nodes.getPath(0).node()
        # print(geom_nodes)
        geom_target = geom_nodes.getGeom(0)
        # print(geom_target)
        output_bullet_mesh = bt.BulletTriangleMesh()
        output_bullet_mesh.addGeom(geom_target)
        tri_shape = bt.BulletTriangleMeshShape(output_bullet_mesh, dynamic=False)
        # print(output_bullet_mesh)

        body = bt.BulletRigidBodyNode('input_model_tri_mesh')
        np = self.render.attachNewNode(body)
        np.node().addShape(tri_shape)
        np.node().setMass(0)
        np.node().setFriction(0.5)
        # np.setPos(0, 0, 0)
        np.setScale(1)
        np.setCollideMask(BitMask32.allOn())
        world.attach(np.node())
