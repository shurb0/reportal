from direct.showbase.ShowBase import ShowBase
from direct.filter.CommonFilters import CommonFilters
from direct.gui.OnscreenText import OnscreenText
from direct.gui.OnscreenImage import OnscreenImage
from panda3d.core import (Vec3, Vec2,
                          ClockObject,
                          WindowProperties,
                          loadPrcFileData,
                          BitMask32,
                          TextNode,
                          TransparencyAttrib, AntialiasAttrib,
                          Shader)
import panda3d.bullet as bt
from math import sin, cos, radians
from player import Player
from level import LevelLoader
from entities import FloorButton, Cube
from inputWatcher import InputWatcher

configVars = """
show-frame-rate-meter 1
sync-video 0
window-title rePortal
interpolate-frames 1
framebuffer-multisample 0
framebuffer-srgb 0
multisamples 2
framebuffer-stencil false
audio-library-name p3openal_audio
"""

loadPrcFileData("", configVars)

type PandaVec2 = Vec2
type PandaVec3 = Vec3


class App(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
    # window properties
        
        self.wp = WindowProperties()
        self.wp.setSize(1920,1080)
        self.wp.setFullscreen(True)
        self.wp.setCursorHidden(True)
        self.wp.setMouseMode(WindowProperties.M_relative)
        self.wp.setRawMice(True)
        self.win.requestProperties(self.wp)

        self.render.setAntialias(AntialiasAttrib.MMultisample)
        
    # get refresh rate
        self.refreshRate = self.pipe.getDisplayInformation().getDisplayModeRefreshRate(2)
        
        globalClock.setMode(ClockObject.MLimited) # noqa # type: ignore
        globalClock.setFrameRate(300) # noqa # type: ignore
        
    # debug (wireframes)

        self.debugNode = bt.BulletDebugNode("Debug")
        self.debugNode.showBoundingBoxes(True)
        self.debugNode.showWireframe(True)
        self.debugNP = self.render.attachNewNode(self.debugNode)
        # self.debugNP.show()
        
    # bullet physics initialisation
        
        self.world = bt.BulletWorld()
        self.world.setGravity(Vec3(0,0,-9.81))
        self.world.setDebugNode(self.debugNP.node())
        self.worldNP = self.render.attachNewNode("World")
        
    # input detection
        
        self.inputWatcher = InputWatcher()
        
    # player
        
        self.player = Player(self.world, self.worldNP, self.render,
                             self.camera, self.loader, self.win,
                             self.inputWatcher, self)
        self.portals = (self.player.bluePortal,self.player.orangePortal)
        
    # entities
         
        self.entities = [[self.player, self.player.playerCollisionNode]]
        
    # map loading
        
        self.level = LevelLoader("lvl0",self.world,self.worldNP,self.loader,self.entities)
        self.player.startPos = self.level.playerStartPos
        self.player.startHpr = self.level.playerStartHpr
        self.player.resetPos()
        
        self.entities += self.level.entities
        
    # camera setup
        
        self.camera.reparentTo(self.player.playerNP)
        self.camera.setPos(0,0,0.5)
        self.camera.setHpr(0,0,0)
        self.camLens.setFov(90)
        self.camLens.setNear(.01)
        self.cam.node().setCameraMask(BitMask32(0x0A))
        
    # mouse set
        
        self.disableMouse()
        
    # colour correction
        
        self.filters = CommonFilters(self.win, self.cam)
        self.filters.setSrgbEncode(True)
        
        self.posText = 0
        self.imageObject = None
        
        self.shader = Shader.load(Shader.SL_GLSL,
                                  vertex="portal.vert",
                                  fragment="portal.frag")
        
        # base.messenger.toggleVerbose()
        self.bgm = self.loader.loadSfx("Assets/audio/empty-room.wav")
        self.bgm.setLoop(True)
        self.bgm.setVolume(0.4)
        self.bgm.play()
        
        # base.messenger.toggleVerbose()
        
        self.player.resetPos()
        self.pauseState = False
        self.pause()
        
    def update(self, task):
        
        if self.inputWatcher.isSet("pause"):
            self.pause()
            return task.done
        
        # if self.pauseState:
        #     self.showPos()
        #     self.win.movePointer(0, self.wp.getXSize() // 2, self.wp.getYSize() // 2)
        #     if self.inputWatcher.isSet("jump"):
        #         self.pauseState = False
        #         self.pauseText.destroy()
        #     return task.cont
        
        dt = globalClock.getDt() # noqa # type: ignore
        
        self.updateDynamicEntityPositions()
        
        self.moveCamera(dt)
        self.player.processInput(dt)
        
        if self.player.startGrab:
            self.player.grabState = True
            self.player.setInteractEntity(self.getEntityFromNode(self.player.interactNode))
            self.player.startGrab = False
        
        if self.inputWatcher.isSet("reset"):
            self.reset()
        
        self.showPos()
        self.showCrosshair()
        
        self.world.doPhysics(dt, 10, 1 / 300)
        
        if self.player.releaseKinematic:
            self.player.updateReleasedEntity()
        
        self.player.processCollisions(dt)
        self.processEntityUpdates()
        self.inputWatcher.fixWheel()
        
        self.renderPortals()
        
        if self.player.transitionNode is not None:
            self.tryLevelTransition()
            return task.done
            
        return task.cont
    
    def pauseUpdate(self, task):
        self.showPos()
        if self.inputWatcher.isSet("jump"):
            self.pauseState = False
            self.pauseText.destroy()
            self.taskMgr.doMethodLater(0.2, self.bufferUpdate, "bufferUpdate")
            self.taskMgr.remove("pauseUpdate")
            self.win.movePointer(0, self.wp.getXSize() // 2, self.wp.getYSize() // 2)
            return task.done
        return task.cont
    
    def updateDynamicEntityPositions(self):
        for entity in self.entities:
            if type(entity[0]) is Cube:
                entity[0].updatePos()
                entity[0].detectTriggers(self.portals)
                entity[0].updateClone(self.portals)
    
    def processEntityUpdates(self):
        for entity in self.entities:
            if type(entity[0]) is FloorButton:
                entity[0].update(self.level.linkedEntities[entity[0].linkID])
    
    def moveCamera(self,dt):
        
        # mouse turning
        
        x = 0
        y = 0
        midX = self.wp.getXSize() // 2
        midY = self.wp.getYSize() // 2
        
        if self.mouseWatcherNode.hasMouse():
            x = self.win.getPointer(0).getX()  # use getPointer(0) instead of mouseWatcherNode.getMouse() !!
            y = self.win.getPointer(0).getY()
            self.win.movePointer(0, midX, midY)
            
        # if x != midX:
        deltaX = (midX - x) * self.player.sensitivity
        newH = (self.camera.getH() + deltaX) % 360
        if newH > 180:
            newH -= 360
        self.camera.setH(newH)
        self.player.player.setH(newH + 180)
        self.player.portalGun.setH(newH - 170)
        
        # if y != midY:
        deltaY = (midY - y) * self.player.sensitivity
        newP = max(-90, min(self.camera.getP() + deltaY, 90))  # clamps y camera between -90 and 90
        self.camera.setP(newP)
        self.player.portalGun.setP(-newP - 7.5)
        
        # positioning viewmodel
        
        newH = radians(newH)
        newP = radians(newP)
        a1 = (.05 * cos(newH)) - (0.125 * sin(newH) * cos(newP)) - 0.09 * sin(newP) * sin(newH)
        a2 = (.05 * sin(newH)) + ((0.125 * cos(newH)) * cos(newP)) + 0.09 * sin(newP) * cos(newH)
        a3 = .5 + 0.125 * sin(newP) - 0.09 * cos(newP)
        
        self.player.portalGun.setPos(a1,a2,a3)
        
        # keyboard turning
        
        if self.inputWatcher.isSet('left'):
            self.camera.setH(self.camera.getH() + dt * 60)
            self.player.player.setH(self.player.player.getH() + dt * 60)
        if self.inputWatcher.isSet('right'):
            self.camera.setH(self.camera.getH() - dt * 60)
            self.player.player.setH(self.player.player.getH() - dt * 60)
        if self.inputWatcher.isSet('up'):
            self.camera.setP(min(self.camera.getP() + dt * 60,90))
        if self.inputWatcher.isSet('down'):
            self.camera.setP(max(self.camera.getP() - dt * 60,-90))
    
    def showPos(self):
        if self.posText != 0:
            self.posText.destroy()
        pos = self.player.playerNP.getPos()
        xpos = round(pos.getX(),2)
        ypos = round(pos.getY(),2)
        zpos = round(pos.getZ(),2)
        vel = self.player.velocity
        xvel = round(vel.getX(),2)
        yvel = round(vel.getY(),2)
        zvel = round(vel.getZ(),2)
        h = round(self.camera.getH(),2)
        p = round(self.camera.getP(),2)
        showPos = f"{self.level.title}\npos: {xpos,ypos,zpos}\nvel: {xvel,yvel,zvel}\nang: {h,p}"
        self.posText = OnscreenText(text=showPos, pos=(-1.75,.9), scale=0.07, align=TextNode.ALeft, mayChange=True)
    
    def showCrosshair(self):
        if self.imageObject is not None:
            self.imageObject.destroy()
        self.imageObject = OnscreenImage(image='Assets/textures/crosshair256.png', pos=(0,0,0),scale=.075)
        self.imageObject.setTransparency(TransparencyAttrib.MAlpha)
    
    def reset(self):
        for entity in self.entities:
            entity[0].resetPos()
        self.player.resetPos()
    
    def pause(self):
        if not self.pauseState:
            self.pauseState = True
            pauseText = "Game paused, press Jump to continue"
            self.pauseText = OnscreenText(text=pauseText, pos=(0,-0.8), scale=0.07,align=TextNode.ACenter,mayChange=False)
            self.taskMgr.add(self.pauseUpdate, "pause")
    
    def bufferUpdate(self,task):
        self.taskMgr.add(self.update, "update")
        return task.done
    
    def getEntityFromNode(self,node):
        for entity in self.entities:
            if entity[1] == node:
                return entity[0]
            elif type(entity[0]) is Cube:
                if entity[2] == node:  # its a clone
                    return entity[0]
    
    def renderPortals(self):
        if not self.player.bluePortal.isPlaced:
            self.player.bluePortal.portalFrame.hide()
        else:
            self.player.bluePortal.portalFrame.show()
        if not self.player.orangePortal.isPlaced:
            self.player.orangePortal.portalFrame.hide()
        else:
            self.player.orangePortal.portalFrame.show()
            
        if not self.player.bluePortal.isPlaced or not self.player.orangePortal.isPlaced:
            return
        
        # check if portal is visible
        lensBounds = self.camLens.makeBounds()
        bluePortalBounds = self.player.bluePortal.portal.getBounds()
        bluePortalBounds.xform(self.player.bluePortal.portal.getMat(self.camera))
        orangePortalBounds = self.player.orangePortal.portal.getBounds()
        orangePortalBounds.xform(self.player.orangePortal.portal.getMat(self.camera))
        
        # render portals
        if lensBounds.contains(bluePortalBounds):
            self.player.bluePortal.renderInPortal(self.player.orangePortal,self.player)
        
        if lensBounds.contains(orangePortalBounds):
            self.player.orangePortal.renderInPortal(self.player.bluePortal,self.player)
    
    def cleanupLevel(self):
        safeObjects = ("player",
                       "portalT","portalB","portalL","portalR",
                       "inportal trigger","blue outportal trigger", "orange outportal trigger")
        for entity in self.entities:
            if entity[0] == self.player:
                continue
            elif type(entity) is FloorButton:
                entity[0].button.delete()
            for i in range(len(entity)):
                entity[i] = None
        for rigidBody in self.world.getRigidBodies():
            if rigidBody.getName() not in safeObjects:
                self.world.remove(rigidBody)
        for ghost in self.world.getGhosts():
            if ghost.getName() not in safeObjects:
                self.world.remove(ghost)
        self.player.interactEntity = None
        self.player.interactNode = None
        self.player.interactNP = None
        self.entities = [[self.player, self.player.playerCollisionNode]]
        self.level.levelModelNP.removeNode()
    
    def tryLevelTransition(self):
        if self.getEntityFromNode(self.player.transitionNode).isActivated and self.level.nextLevel != "end":
            self.cleanupLevel()
            
            self.level = LevelLoader(self.level.nextLevel,self.world,self.worldNP,self.loader,self.entities)
            self.entities += self.level.entities
            self.player.startPos = self.level.playerStartPos
            self.player.startHpr = self.level.playerStartHpr
            self.player.resetPos()
            
            for portal in self.portals:
                portal.isPlaced = False
                portal.portal.setTexture(portal.blankTexture,1)
                portal.portalFrame.setPos(portal.pos)
                portal.updatePos(portal.pos)
            
            self.pause()
            self.taskMgr.remove("update")


app = App()
app.setFrameRateMeter(True)
app.run()
