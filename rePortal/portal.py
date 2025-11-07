from panda3d.core import (Vec3, LRotationf as Rot,
                          NodePath,
                          BitMask32,
                          Shader,
                          Plane, PlaneNode,
                          TextureStage)
import panda3d.bullet as bt
from math import sin, cos, radians


class Portal:
    """
    Portal entity class
    """
    def __init__(self,isOrange,world,worldNP,render,loader,win,cam,base):
        self.isOrange = isOrange
        self.world = world
        self.worldNP = worldNP
        self.render = render
        self.win = win
        self.pos = Vec3(0,0,-100)
        
        self.isPlaced = False
        
        if self.isOrange:
            self.colour = "orange"
        else:
            self.colour = "blue"
            
        self.portalFrame = loader.loadModel(f"Assets/models/{self.colour}Portal.glb")
        self.portalFrame.reparentTo(self.worldNP)
        self.portalFrame.setScale(.5,.5,.5)
        self.portalFrame.setPos(self.pos)
        
        if not self.isOrange:
            self.normal = Vec3(-1,0,0)
            self.portalFrame.setHpr(180,0,0)
            self.portalFrame.node().adjustDrawMask(BitMask32.allOff(),BitMask32(0x16),BitMask32.allOff())
        else:
            self.normal = Vec3(1,0,0)
            self.portalFrame.node().adjustDrawMask(BitMask32.allOff(),BitMask32(0x26),BitMask32.allOff())
        
        self.portal = loader.loadModel("Assets/models/portal2.obj")
        self.portal.reparentTo(self.portalFrame)
        self.portal.setTransparency(True)
        self.portal.setDepthOffset(5)  # to resolve z-fighting
        self.portal.setP(-90)
        
        self.blankTexture = loader.loadTexture("Assets/textures/almostWhite.png")
        self.portal.setTexture(self.blankTexture,1)
        
        self.buffer = self.win.makeTextureBuffer(self.colour, 0, 0)  # makes a texture buffer named colour and size of screen
        self.buffer.setSort(-100)
        self.camera = base.makeCamera(self.buffer)  # this camera renders behind the other portal
        self.camera.reparentTo(self.render)
        self.camLens = self.camera.node().getLens()
        self.camLens.setAspectRatio(16 / 9)
        self.camLens.setFov(90)
        self.camLens.setNear(0.1)
        
        self.shader = Shader.load(Shader.SL_GLSL,
                                  vertex="portal.vert",
                                  fragment="portal.frag")
        
        self.portal.setShader(self.shader)
        
        self.renderState = NodePath(self.colour)
        
        if not self.isOrange:
            clipPlane = Plane(1,0,0,6.99)
            clipPlaneNode = PlaneNode("clipPlane",clipPlane)
            self.clipPlaneNP = self.worldNP.attachNewNode(clipPlaneNode)
            
            self.camera.node().setCameraMask(BitMask32(0x26))
        else:
            clipPlane = Plane(-1,0,0,16.99)
            clipPlaneNode = PlaneNode("clipPlane",clipPlane)
            self.clipPlaneNP = self.worldNP.attachNewNode(clipPlaneNode)
            
            self.camera.node().setCameraMask(BitMask32(0x16))
            
        self.renderState.setClipPlane(self.clipPlaneNP)
        self.camera.node().setInitialState(self.renderState.getState())
        self.relativePos = Vec3()
        self.relativeRot = Rot()
        
        horizontalCollisionShape = bt.BulletBoxShape(Vec3(.1,1,.1))
        verticalCollisionShape = bt.BulletBoxShape(Vec3(.1,.1,2))
        bottomCollider = bt.BulletRigidBodyNode("portalB")
        bottomCollider.addShape(horizontalCollisionShape)
        self.bottomNP = self.portalFrame.attachNewNode(bottomCollider)
        self.bottomNP.setPos(-.1,0,-3.1)
        self.bottomNP.setCollideMask(BitMask32.bit(3))
        leftCollider = bt.BulletRigidBodyNode("portalL")
        leftCollider.addShape(verticalCollisionShape)
        self.leftNP = self.portalFrame.attachNewNode(leftCollider)
        self.leftNP.setPos(-.1,-2.1,0)
        self.leftNP.setCollideMask(BitMask32.bit(3))
        rightCollider = bt.BulletRigidBodyNode("portalR")
        rightCollider.addShape(verticalCollisionShape)
        self.rightNP = self.portalFrame.attachNewNode(rightCollider)
        self.rightNP.setPos(-.1,2.1,0)
        self.rightNP.setCollideMask(BitMask32.bit(3))
        
        self.colliderNodes = (bottomCollider,leftCollider,rightCollider)
        
        # inportal trigger detects if entity is inside portal to remove collisions
        
        inPortalTriggerShape = bt.BulletBoxShape(Vec3(1,1,3))
        inPortalTriggerNode = bt.BulletGhostNode("inportal trigger")
        inPortalTriggerNode.addShape(inPortalTriggerShape)
        self.inPortalTriggerNP = self.portalFrame.attachNewNode(inPortalTriggerNode)
        self.inPortalTriggerNP.setX(-0.75)
        self.inPortalTriggerNP.setCollideMask(BitMask32.bit(5))
        
        # outportal trigger is the teleport trigger for the player
        
        outPortalTriggerShape = bt.BulletBoxShape(Vec3(0.25,0.5,2))
        outPortalTriggerNode = bt.BulletGhostNode(f"{self.colour} outportal trigger")
        outPortalTriggerNode.addShape(outPortalTriggerShape)
        self.outPortalTriggerNP = self.portalFrame.attachNewNode(outPortalTriggerNode)
        self.outPortalTriggerNP.setPos(-1.245,0,0)
        self.outPortalTriggerNP.setCollideMask(BitMask32.bit(5))
        
        self.world.attach(self.bottomNP.node())
        self.world.attach(self.leftNP.node())
        self.world.attach(self.rightNP.node())
        self.world.attach(self.inPortalTriggerNP.node())
        self.world.attach(self.outPortalTriggerNP.node())
    
    def updatePos(self,pos):
        self.portalFrame.setPos(pos)
        self.bottomNP.setPos(Vec3())
        self.leftNP.setPos(Vec3())
        self.rightNP.setPos(Vec3())
        self.inPortalTriggerNP.setPos(Vec3(0.001,0,0))
        self.outPortalTriggerNP.setPos(Vec3())
        
        self.bottomNP.setPos(-.1,0,-3.1)
        self.leftNP.setPos(-.1,-2.1,0)
        self.rightNP.setPos(-.1,2.1,0)
        self.inPortalTriggerNP.setPos(Vec3())
        self.outPortalTriggerNP.setPos(-1.245,0,0)
    
    def renderInPortal(self,outPortal,player):
        self.camera.setPos(player.currentPos + Vec3(0,0,0.5))
        self.camera.setHpr(player.camera.getHpr())
        
        ts = TextureStage('ts')
        ts.setSort(-100)
        ts.setMode(TextureStage.MDecal)
        
        # portal camera position
        self.relativePos = self.camera.getPos() - self.portalFrame.getPos()
        self.relativePos = Rot(-self.portalFrame.getH(),self.portalFrame.getP(),self.portalFrame.getR()).xform(self.relativePos)
        outPortalRot = Rot(outPortal.portalFrame.getH(),outPortal.portalFrame.getP(),-outPortal.portalFrame.getR())
        self.relativePos = outPortalRot.xform(self.relativePos)
        self.relativePos.setZ(-self.relativePos.getZ())
        self.camera.setPos(outPortal.portalFrame.getPos() - self.relativePos)  # sets the camera behind the outPortal
        
        # portal camera rotation # this is complete bullshit i threw random spaghetti numbers to the wall till it landed
        self.relativeRot = Rot(self.portalFrame.getH(),self.portalFrame.getP(),self.portalFrame.getR())
        self.relativeRot.invertInPlace()
        camRotH = self.camera.getH()
        camRotP = self.camera.getP() * cos(radians(self.portalFrame.getH()))
        camRotR = self.camera.getP() * sin(radians(-self.portalFrame.getH()))
        self.relativeRot *= Rot(camRotH,camRotP,camRotR)
        self.relativeRot *= Rot(180,0,0)
        self.relativeRot *= outPortal.portalFrame.getQuat()
        self.camera.setQuat(self.relativeRot)
        
        # setting portal texture
        self.portal.setTexture(self.buffer.getTexture(),1)
