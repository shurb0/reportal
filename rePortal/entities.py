from direct.actor.Actor import Actor
from panda3d.core import (Vec3,
                          Point3,
                          BitMask32,
                          SamplerState,
                          LRotationf as Rot,
                          CardMaker,
                          TransparencyAttrib)
import panda3d.bullet as bt
from math import sin, cos, tan, radians


class FloorButton:
    """
    Static object with trigger to detect player and physics object and send output
    """
    def __init__(self,world,worldNP,loader,pos,hpr,linkID):
        self.world = world
        self.worldNP = worldNP
        self.loader = loader
        self.linkID = linkID
        
        baseHeight = 0.25
        baseRadius = .8
        baseTan60 = tan(radians(60)) * baseRadius
        baseTan30 = tan(radians(30)) * baseRadius
        baseTopRadius = .65
        baseTopTan60 = tan(radians(60)) * baseTopRadius
        baseTopTan30 = tan(radians(30)) * baseTopRadius
        self.baseShape = bt.BulletConvexHullShape()
        self.baseShape.addArray([
            Point3(baseRadius,0,0), Point3(-baseRadius,0,0),
            Point3(0,baseRadius,0), Point3(0,-baseRadius,0),
            Point3(baseTan30,baseTan60,0), Point3(-baseTan30,baseTan60,0),
            Point3(baseTan30,-baseTan60,0), Point3(-baseTan30,-baseTan60,0),
            Point3(baseTan60,baseTan30,0), Point3(-baseTan60,baseTan30,0),
            Point3(baseTan60,-baseTan30,0), Point3(-baseTan60,-baseTan30,0),
            Point3(baseTopRadius,0,baseHeight), Point3(-baseTopRadius,0,baseHeight),
            Point3(0,baseTopRadius,baseHeight), Point3(0,-baseTopRadius,baseHeight),
            Point3(baseTopTan30,baseTopTan60,baseHeight), Point3(-baseTopTan30,baseTopTan60,baseHeight),
            Point3(baseTopTan30,-baseTopTan60,baseHeight), Point3(-baseTopTan30,-baseTopTan60,baseHeight),
            Point3(baseTopTan60,baseTopTan30,baseHeight), Point3(-baseTopTan60,baseTopTan30,baseHeight),
            Point3(baseTopTan60,-baseTopTan30,baseHeight), Point3(-baseTopTan60,-baseTopTan30,baseHeight),
        ])  # have fun reading that
        
        self.baseCollisionNode = bt.BulletRigidBodyNode("floorButtonBase")
        self.baseCollisionNode.addShape(self.baseShape)
        self.baseNP = self.worldNP.attachNewNode(self.baseCollisionNode)
        self.baseNP.setPos(pos)
        self.baseNP.setHpr(hpr)
        self.baseNP.setCollideMask(BitMask32.bit(1))
        # self.baseNP.hide()
        self.base = self.loader.loadModel("Assets/models/floorButtonBase.glb")
        self.base.reparentTo(self.baseNP)
        
        triggerHeight = .1
        triggerRadius = 1
        self.triggerShape = bt.BulletCylinderShape(triggerRadius,triggerHeight,bt.ZUp)
        self.triggerGhostNode = bt.BulletGhostNode("floorButtonTrigger")
        self.triggerGhostNode.addShape(self.triggerShape)
        self.triggerNP = self.worldNP.attachNewNode(self.triggerGhostNode)
        self.triggerNP.setPos(pos + Vec3(0,0,baseHeight + triggerHeight / 2))
        self.triggerNP.setCollideMask(BitMask32.bit(4))
        self.world.attach(self.triggerGhostNode)
        
        self.button = Actor("Assets/models/floorButton.glb")
        self.button.reparentTo(self.baseNP)
        self.button.setPos(Vec3())
        self.world.attach(self.baseNP.node())
        
        self.actived = False  # flag for activating trigger
        self.isActivated = False  # flag for determining activation state
        self.contactList = []
    
    def update(self,linkList):
        self.updateTrigger()
        if self.activated and not self.isActivated:
            for entity in linkList:
                entity.activate()
        elif not self.activated and self.isActivated:
            for entity in linkList:
                entity.deactivate()
    
    def updateTrigger(self):
        result = self.world.contactTest(self.triggerGhostNode)
        self.contactList = []
        self.activated = False
        for contact in result.getContacts():
            contactNode = contact.getNode1()
            if contactNode in self.contactList:
                continue
            self.contactList.append(contactNode)
            
            if contactNode.getName() == "box" or contactNode.getName() == "player":
                self.activated = True
    
    def activate(self):
        self.isActivated = True
        self.button.play("+toggle")
    
    def deactivate(self):
        self.isActivated = False
        self.button.play("-toggle")
    
    def resetPos(self):
        pass


class Cube:
    """
    Dynamic/kinematic physics based interactable cube object
    """
    def __init__(self,world,worldNP,loader,pos,hpr):
        self.world = world
        self.worldNP = worldNP
        self.loader = loader
        self.startPos = pos
        
        self.boxCollisionShape = bt.BulletBoxShape(Vec3(.4,.4,.4))
        self.boxCollisionNode = bt.BulletRigidBodyNode("box")
        self.boxCollisionNode.addShape(self.boxCollisionShape)
        self.boxCollisionNode.setMass(1)
        self.boxCollisionNode.setFriction(self.boxCollisionNode.getFriction() * 5)
        self.boxCollisionNode.setDeactivationEnabled(False)
        self.boxCollisionNode.setKinematic(False)
        self.boxCollisionNode.setGravity(Vec3(0,0,-9.81))
        
        self.boxNP = self.worldNP.attachNewNode(self.boxCollisionNode)
        self.boxNP.setPos(self.startPos)
        self.boxNP.setHpr(hpr)
        self.boxNP.setCollideMask(BitMask32.allOn())
        
        self.box = self.loader.loadModel("Assets/models/cube.obj")
        self.box.setScale(.4,.4,.4)
        self.box.setClipPlaneOff()
        self.box.reparentTo(self.boxNP)
        self.world.attach(self.boxNP.node())
        
        self.boxDeactivatedTexture = self.loader.loadTexture("Assets/textures/cube.png")
        self.boxDeactivatedTexture.setMagfilter(SamplerState.FT_nearest)
        self.boxDeactivatedTexture.setMinfilter(SamplerState.FT_nearest)
        self.boxActivatedTexture = self.loader.loadTexture("Assets/textures/cubeActivated.png")
        self.boxActivatedTexture.setMagfilter(SamplerState.FT_nearest)
        self.boxActivatedTexture.setMinfilter(SamplerState.FT_nearest)
        self.box.setTexture(self.boxDeactivatedTexture,1)
        self.velocity = self.boxCollisionNode.getLinearVelocity()
        self.currentPos = self.boxNP.getPos()
        
        self.cloneCollisionNode = bt.BulletRigidBodyNode("clone")
        self.cloneCollisionNode.addShape(self.boxCollisionShape)
        self.cloneCollisionNode.setMass(1)
        self.cloneCollisionNode.setFriction(self.boxCollisionNode.getFriction() * 5)
        self.cloneCollisionNode.setDeactivationEnabled(False)
        self.cloneCollisionNode.setKinematic(False)
        self.cloneCollisionNode.setGravity(Vec3(0,0,-9.81))
        
        self.cloneNP = self.worldNP.attachNewNode(self.cloneCollisionNode)
        self.cloneNP.setPos(self.startPos)
        self.cloneNP.setHpr(hpr)
        mask = BitMask32.allOn()
        mask.setBitTo(1,False)
        self.cloneNP.setCollideMask(mask)
        
        self.clone = self.loader.loadModel("Assets/models/cube.obj")
        self.clone.reparentTo(self.cloneNP)
        self.clone.setScale(.4,.4,.4)
        self.clone.setClipPlaneOff()
        self.clone.setTexture(self.boxDeactivatedTexture,1)
        self.world.attach(self.cloneNP.node())
        
        self.inPortal = False
        self.forceInPortalState = False  # forces inportal to activate if the player is inportal
        self.isActivated = True
        self.isHeld = False
    
    def applyLinearVelocity(self,dt):
        """
        Moves player position by velocity vector and calls penetration prevention function
        """
        
        self.currentPos = self.boxNP.getPos()
        self.currentPos += self.velocity * dt
        self.boxNP.setPos(self.currentPos)
    
    def processCollisions(self,dt):
        """
        Prevents penetration into objects by moving current position by scaled manifold normal
        """
        
        result = self.world.contactTest(self.boxCollisionNode)
        contactList = []
        self.inPortal = self.forceInPortalState
        
        for contact in result.getContacts():
            if contact.getNode1().getName() == "inportal trigger":
                self.inPortal = True
        
        for contact in result.getContacts():
            contactNode = contact.getNode1()  # the collision node the player is in contact with
            contactName = contactNode.getName()
            if contactNode in contactList or contactName == "player" or type(contactNode) is bt.BulletGhostNode:
                continue
            contactList.append(contactNode)
            pairResult = self.world.contactTestPair(self.boxCollisionNode, contactNode)
            self.chastity(pairResult)

    def chastity(self,pairResult):  # prevent penetration
        normalList = []
        for pairContact in pairResult.getContacts():
            pulloutGame = Vec3()  # haha get it because its nothing but its meant to push you out of clip brushes
            pushoutGame = Vec3()
            mpoint = pairContact.getManifoldPoint()
            normal = mpoint.getNormalWorldOnB()
            
            if normal in normalList or (self.inPortal and pairContact.getNode1().getName() == "portal collider"):
                continue
            normalList.append(normal)
            
            if mpoint.getDistance() < 0:
                pulloutGame -= normal * mpoint.getDistance()
                pushoutGame -= self.velocity.project(normal)  # z velocity should be handled by groundSnap
            self.currentPos += pulloutGame
            if pushoutGame.length() < 30:  # so that it doesnt kill your computer...
                self.velocity += pushoutGame
            self.boxNP.setPos(self.currentPos)
    
    def resetPos(self):
        self.boxNP.setPos(self.startPos)
        self.boxCollisionNode.setLinearVelocity(Vec3())
        self.velocity = Vec3()
    
    def processDynamicCollision(self, pairResult, contactNode, dt):  # chat im cooked
        normalList = []
        for pairContact in pairResult.getContacts():
            mpoint = pairContact.getManifoldPoint()
            normal = mpoint.getNormalWorldOnB()
            
            if normal in normalList:
                continue
            normalList.append(normal)
            
            if mpoint.getDistance() < 0:
                contactNode.applyImpulse(self.velocity * dt, pairContact.getManifoldPoint().getLocalPointB())
    
    def detectTriggers(self,portals):
        self.inPortal = False
        self.inPortalPointer = None
        pastActivateState = self.isActivated
        self.isActivated = False
        result = self.world.contactTest(self.boxCollisionNode)
        for contact in result.getContacts():
            node = contact.getNode1()
            for portal in portals:
                if node == portal.inPortalTriggerNP.node():
                    self.inPortal = True
                    self.inPortalPointer = portal
                    mask = BitMask32.allOn()
                    mask.setBitTo(1,False)
                    self.boxNP.setCollideMask(mask)
            if node.getName() == "floorButtonTrigger":
                self.isActivated = True
        
        if not pastActivateState and self.isActivated:
            self.activate()
        elif pastActivateState and not self.isActivated:
            self.deactivate()
    
    def updateClone(self,portals):
        if self.inPortal:
            self.clone.show()
            if not self.isHeld:
                self.cloneCollisionNode.setKinematic(False)
            portalFrame = self.inPortalPointer.portalFrame
            for portal in portals:
                if portal is not self.inPortalPointer:
                    outPortal = portal
            
            # update position
            relativePos = self.boxNP.getPos() - portalFrame.getPos()
            relativePos = Rot(-portalFrame.getH(),portalFrame.getP(),portalFrame.getR()).xform(relativePos)
            outPortalRot = Rot(outPortal.portalFrame.getH(),outPortal.portalFrame.getP(),-outPortal.portalFrame.getR())
            relativePos = outPortalRot.xform(relativePos)
            relativePos.setZ(-relativePos.getZ())
            self.cloneNP.setPos(outPortal.portalFrame.getPos() - relativePos)
            
            # update rotation
            relativeRot = Rot(portalFrame.getH(),portalFrame.getP(),portalFrame.getR())
            relativeRot.invertInPlace()
            rotH = self.boxNP.getH()
            rotP = self.boxNP.getP() * cos(radians(portalFrame.getH()))
            rotR = self.boxNP.getP() * sin(radians(-portalFrame.getH()))
            relativeRot *= Rot(rotH,rotP,rotR)
            relativeRot *= Rot(180,0,0)
            relativeRot *= outPortal.portalFrame.getQuat()
            self.cloneNP.setQuat(relativeRot)
            
            if not self.isHeld and self.checkTeleport():
                self.teleport()
        else:
            self.clone.hide()
            self.cloneCollisionNode.setKinematic(True)
            self.cloneNP.setPos(0,0,-1000)
    
    def activate(self):
        self.box.setTexture(self.boxActivatedTexture)
        self.clone.setTexture(self.boxActivatedTexture)
    
    def deactivate(self):
        self.box.setTexture(self.boxDeactivatedTexture)
        self.clone.setTexture(self.boxDeactivatedTexture)
    
    def updatePos(self):
        self.currentPos = self.boxNP.getPos()
    
    def checkTeleport(self):
        pos = self.boxNP.getPos()
        inPortalPos = self.inPortalPointer.portalFrame.getPos()
        inPortalNormal = self.inPortalPointer.normal
        if inPortalNormal.getX() < 0:
            if pos.getX() > inPortalPos.getX():
                return True
        elif inPortalNormal.getX() > 0:
            if pos.getX() < inPortalPos.getX():
                return True
        elif inPortalNormal.getY() < 0:
            if pos.getY() > inPortalPos.getY():
                return True
        elif inPortalNormal.getX() > 0:
            if pos.getY() < inPortalPos.getY():
                return True
        return False
    
    def teleport(self):
        pos = self.boxNP.getPos()
        rot = self.boxNP.getQuat()
        vel = self.boxCollisionNode.getLinearVelocity()
        clonePos = self.cloneNP.getPos()
        cloneRot = self.cloneNP.getQuat()
        cloneVel = self.cloneCollisionNode.getLinearVelocity()
        
        self.boxNP.setPos(clonePos)
        self.boxNP.setQuat(cloneRot)
        self.boxCollisionNode.setLinearVelocity(cloneVel)
        self.cloneNP.setPos(pos)
        self.cloneNP.setQuat(rot)
        self.cloneCollisionNode.setLinearVelocity(vel)


class LevelEnd:
    def __init__(self,world,worldNP,loader,pos,hpr,linkID=None):
        self.linkID = linkID
        
        triggerShape = bt.BulletBoxShape(Vec3(0.05,1,1))
        self.levelEndTriggerNode = bt.BulletGhostNode("levelEnd")
        self.levelEndTriggerNode.addShape(triggerShape)
        
        self.levelEndNP = worldNP.attachNewNode(self.levelEndTriggerNode)
        self.levelEndNP.setPos(pos)
        self.levelEndNP.setHpr(hpr)
        world.attach(self.levelEndNP.node())
        cm = CardMaker("levelEndCard")
        cm.setFrame(-1.25,1.25,-1.25,1.25)
        
        self.flagTexture = loader.loadTexture("Assets/textures/flags.png")
        self.flagTexture.setMagfilter(SamplerState.FT_nearest)
        self.flagTexture.setMinfilter(SamplerState.FT_nearest)
        self.flagTextureDeactivated = loader.loadTexture("Assets/textures/flagsDeactivated.png")
        self.flagTextureDeactivated.setMagfilter(SamplerState.FT_nearest)
        self.flagTextureDeactivated.setMinfilter(SamplerState.FT_nearest)
        
        self.frontCard = self.levelEndNP.attachNewNode(cm.generate())
        self.frontCard.setH(90)
        self.frontCard.setTransparency(TransparencyAttrib.MAlpha)
        
        self.backCard = self.levelEndNP.attachNewNode(cm.generate())
        self.backCard.setH(-90)
        self.backCard.setTransparency(TransparencyAttrib.MAlpha)
        if linkID:
            self.deactivate()
        else:
            self.activate()
    
    def activate(self):
        self.isActivated = True
        self.frontCard.setTexture(self.flagTexture)
        self.backCard.setTexture(self.flagTexture)
    
    def deactivate(self):
        self.isActivated = False
        self.frontCard.setTexture(self.flagTextureDeactivated)
        self.backCard.setTexture(self.flagTextureDeactivated)
    
    def resetPos(self):
        pass
