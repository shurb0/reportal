from direct.actor.Actor import Actor
from panda3d.core import (Vec3, Vec2, Point3, LRotationf as Rot,
                          TransformState,
                          BitMask32,
                          StencilAttrib,
                          NodePath,
                          Plane, PlaneNode)
import panda3d.bullet as bt
from math import sin, cos, tan, asin, atan2, radians, degrees, log
from portal import Portal

type PandaVec2 = Vec2
type PandaVec3 = Vec3


class Player:
    """
    Kinematic player controller
    """
    def __init__(self, world, worldNP, render, camera, loader, win, inputWatcher, base):
        self.world = world
        self.worldNP = worldNP
        self.render = render
        self.camera = camera
        self.inputWatcher = inputWatcher
        
        self.playerCollisionShape = bt.BulletBoxShape(Vec3(.5,.5,1))
        self.playerCollisionNode = bt.BulletRigidBodyNode("player")
        self.playerCollisionNode.addShape(self.playerCollisionShape)
        self.playerCollisionNode.setKinematic(True)
        self.playerCollisionNode.setMass(1)
        self.playerCollisionNode.setFriction(1)
        
        self.playerNP = self.worldNP.attachNewNode(self.playerCollisionNode)
        self.playerNP.setPos(0,2,3)
        self.playerNP.node().setCcdMotionThreshold(1e-7)
        self.playerNP.node().setCcdSweptSphereRadius(0.50)
        
        self.playerNP.setCollideMask(BitMask32.bit(0))
        self.world.attach(self.playerNP.node())
        
        self.player = Actor("Assets/models/characterModel.glb")
        self.player.reparentTo(self.playerNP)
        self.player.setPos(0,0,-1)  # tiny offset because clipping
        self.player.setH(180)
        self.player.setScale(.6,.6,.6)
        
        self.player.makeSubpart("arms",["forearm.L","forearm.R","upper_arm.L","upper_arm.R"])
        self.player.makeSubpart("legs",["shin.L","shin.R","thigh.L","thigh.R"])
        self.player.play("a-pose", partName="arms")
        self.player.setClipPlaneOff()
        self.walkCycleController = self.player.getAnimControls("walk-cycle","legs")[0]
        
        self.playerHideNP = NodePath(self.player)
        self.player.node().adjustDrawMask(BitMask32.allOff(),BitMask32(0x0A),BitMask32.allOff())
        
        self.portalGunPos = Vec3(0.125,-0.05,0.41)
        self.portalGunHpr = Vec3(100,-7.5,0)
        self.portalGun = Actor("Assets/models/portalGun.glb")
        self.portalGun.reparentTo(self.playerNP)
        self.portalGun.setPos(self.portalGunPos)
        self.portalGun.setHpr(self.portalGunHpr)
        self.portalGun.setScale(.025,.025,.025)
        self.portalGun.node().adjustDrawMask(BitMask32(0x0A),BitMask32(0x36),BitMask32.allOff())
        
        self.portalGunSound = loader.loadSfx("Assets/audio/bad-portal-sound.wav")
        self.portalGunSound.setVolume(0.5)
        self.portalGunSoundFail = loader.loadSfx("Assets/audio/portal-fail.wav")
        self.portalGunSoundFail.setVolume(0.4)
        
        self.bluePortal = Portal(False,world,worldNP,render,loader,win,self.camera,base)
        self.orangePortal = Portal(True,world,worldNP,render,loader,win,self.camera,base)
        self.portals = (self.bluePortal,self.orangePortal)
        
        self.constantOneStencil = StencilAttrib.make(1,StencilAttrib.SCFAlways,
                                                     StencilAttrib.SOZero,StencilAttrib.SOReplace,StencilAttrib.SOReplace,
                                                     1,0,1)
        
        self.stencilReader = StencilAttrib.make(1,StencilAttrib.SCFEqual,
                                                StencilAttrib.SOKeep,StencilAttrib.SOKeep,StencilAttrib.SOKeep,
                                                1,1,0)
        
        self.halfExtentHeight = self.playerCollisionShape.getHalfExtentsWithoutMargin().getZ()
        self.onGround = False
        self.onSlope = False
        self.bonking = False
        
        self.noclip = False
        self.grabState = False
        self.startGrab = False
        
        # movement
        self.maxGroundSpeed = 250
        self.maxSlopeAngleDeg = 45
        self.maxSlopeAngleRad = radians(self.maxSlopeAngleDeg)
        self.terminalVelocity = -25
        self.currentPos = self.playerNP.getPos()
        self.gravity = Vec3(0,0,.19)
        self.velocity = Vec3()
        self.acceleration = 5
        self.airAcceleration = 5
        self.jumpImpulse = Vec3(0,0,4.5)
        self.friction = -10
        self.maxStepHeight = 0.25
        self.sensitivity = .01
        
        # interacts
        self.interactEntity = None
        self.interactNode = None
        self.interactNP = None
        self.interactLength = 2.25
        self.grabLength = 2.25
        self.maxPullSpeed = 25
        self.maxHoldLength = 5
        self.pullAccel = 15
        self.releaseState = False
        self.releaseKinematic = False
        
        self.fixedTimestep = 1 / 60
        self.count = 0
        
        # input delayers
        self.useState = False
        self.jumpState = False
        self.shootState = False
        
        self.snappy = False
        self.bottomTo = 0
        
        # portals
        self.maxShootLength = 250
        self.inPortal = False
        
        # misc
        self.startPos = Vec3()  # IMPORTANT: DO NOT USE x = self.startPos IF YOU AREN'T ACTUALLY CHANGING THE STARTPOS
        self.startHpr = Vec3()  # IMPORTANT: DO NOT USE x = self.startHpr IF YOU AREN'T ACTUALLY CHANGING THE STARTHPR
        self.transitionNode = None

# ====================
#
#   input processing
#
# ====================

    def processInput(self, dt: int):
        
        """
        Translates keyboard inputs into raw movement or function execution
        """
        
        self.processShootState()
        
        self.findGroundState(dt)
        
        wishVelocity, groundedWishSpeed = self.inputMovement(dt)
        
        heading = self.camera.getH()
        pitch = self.camera.getP()
        camPos = Vec3(self.currentPos.getX(), self.currentPos.getY(), self.currentPos.getZ() + self.camera.getZ())
        rayDir = Vec3(-sin(heading), cos(heading), tan(pitch))
        rayDir = Rot(heading,pitch,0).xform(Vec3.forward())
        rayVec = rayDir.normalized() * self.interactLength
        rayTo = rayVec + camPos
        
        self.processUseState(camPos, rayTo, dt)
        if self.grabState:
            self.boxPull(rayDir, camPos, dt)
        # gravity/groundsnap
        if self.onGround:
            self.velocity.setZ(0)
            self.groundAccelerate(dt, wishVelocity, groundedWishSpeed)
            
            if self.inputWatcher.isSet('jump'):
                if not self.jumpState:
                    if self.walkCycleController.isPlaying():
                        self.walkCycleController.pose(16)
                    self.jump()
            else:
                if self.jumpState:
                    self.player.play("land","legs")
                else:
                    if not self.walkCycleController.isPlaying() and self.velocity.getXy().length() != 0:
                        self.walkCycleController.loop(0,16,39)
                    elif self.walkCycleController.isPlaying() and self.velocity.getXy().length() == 0:
                        self.walkCycleController.pose(16)
                self.jumpState = False
                # self.groundSnap(dt)
        else:
            self.applyGravity(dt)
            self.airAccelerate(wishVelocity, groundedWishSpeed)
        
        # fps toggle (this is for dt debug)
        
        if self.inputWatcher.isSet("fps30"):
            globalClock.setFrameRate(30) # noqa # type: ignore
        if self.inputWatcher.isSet("fps60"):
            globalClock.setFrameRate(60) # noqa # type: ignore
        if self.inputWatcher.isSet("fps144"):
            globalClock.setFrameRate(144) # noqa # type: ignore
        if self.inputWatcher.isSet("fps300"):
            globalClock.setFrameRate(300) # noqa # type: ignore
        if self.inputWatcher.isSet("fps0"):
            globalClock.setFrameRate(-1) # noqa # type: ignore
        
    # sending output to character node
        
        self.applyLinearVelocity(dt)
        
        if self.grabState:
            self.interactEntity.applyLinearVelocity(dt)

# ==========================
#
#   movement
#   - ground acceleration
#   - air acceleration
#   - jumping
#   - friction
#   - gravity
#   - velocity application
#
# ==========================

    def inputMovement(self, dt):
        wishVelocity = Vec3(0,0,0)
        heading = radians(self.camera.getH())
        
    # defining wish velocity from input
        # at this stage wishSpeed doesn't matter
        
        if self.inputWatcher.isSet('forward'):
            wishVelocity += (Vec3(-sin(heading), cos(heading),0))
        if self.inputWatcher.isSet('back'):
            wishVelocity += (Vec3(sin(heading), -cos(heading),0))
        if self.inputWatcher.isSet('moveleft'):
            wishVelocity += (Vec3(-cos(heading), -sin(heading),0))
        if self.inputWatcher.isSet('moveright'):
            wishVelocity += (Vec3(cos(heading), sin(heading),0))
        
        # normalise to wishSpeed
        if wishVelocity.length() != 0:
            wishVelocity = wishVelocity.normalized().__mul__(self.acceleration * dt)
            
        # get max speed
        groundedWishSpeed = self.maxGroundSpeed * (self.fixedTimestep)
        
        return wishVelocity, groundedWishSpeed

    def groundAccelerate(self,dt: int, wishVelocity: PandaVec3, groundedWishSpeed: int):
        """
        Applies friction and movement in given direction, scaled between framerates

        Args:
            wishVelocity (Vec3): velocity in direction of input
            groundedWishSpeed (int): grounded speed cap
        """
        
        horizontalVelocity = self.velocity.getXy()
        
        frictionForce = self.getFriction(dt,horizontalVelocity)
        
        horizontalSpeed = horizontalVelocity.length()
        
        if horizontalSpeed != 0:
            self.velocity.addY(frictionForce.getY())
            self.velocity.addX(frictionForce.getX())  # must separate bc frictionForce is Vec2 and velocity is Vec3
        
        if horizontalSpeed < groundedWishSpeed:
            self.velocity.addY(wishVelocity.getY() * self.acceleration)
            self.velocity.addX(wishVelocity.getX() * self.acceleration)
            
            # snap to max
            if groundedWishSpeed * 1.05 > self.velocity.length() > groundedWishSpeed:
                self.velocity = self.velocity.normalized() * groundedWishSpeed
    
    def airAccelerate(self, wishVelocity: PandaVec3, groundedWishSpeed: int):
        """
        Applies quake/source style airstrafing
        
        Args:
            wishVelocity (Vec3): velocity in direction of input
            groundedWishSpeed (int): grounded speed cap
        """
        
        wishSpeed = groundedWishSpeed / 10  # since max walking speed never really changes...
        wishVelocity.normalize()  # turns out quake was badly coded but panda is not... really?
        currentSpeed = self.velocity.dot(wishVelocity)  # will be zero if they are perpendicular, i.e. ur moving a/d
        addSpeed = wishSpeed - currentSpeed  # if ur moving a/d, currentSpeed = 0, addSpeed = wishSpeed ;)
        
        if addSpeed > 0:
            accelSpeed = groundedWishSpeed * self.airAcceleration
            
            if accelSpeed > addSpeed:
                accelSpeed = addSpeed
            
            addVelocity = wishVelocity * accelSpeed
            self.velocity += addVelocity

    def jump(self):
        """
        What did you think this does?
        """
        
        self.player.play("jump",partName="legs")
        self.jumpState = True
        self.velocity += self.jumpImpulse

    def getFriction(self,dt: int,horizontalVelocity: PandaVec2):
        """
        Calculates friction vector from current horizontal velocity
        
        Args:
            horizontalVelocity (Vec2): XY component of velocity
        
        Returns:
            Vec2: Friction vector
        """
        
        frictionForce = horizontalVelocity.normalized() * self.friction * dt
        
        # velocity snap to zero
        if abs(frictionForce.getY()) > abs(horizontalVelocity.getY()):
            frictionForce.setY(-horizontalVelocity.getY())
        if abs(frictionForce.getX()) > abs(horizontalVelocity.getX()):
            frictionForce.setX(-horizontalVelocity.getX())
        
        return frictionForce

    def applyGravity(self, dt: int):
        """
        Applies gravity (negative z vector) to player velocity based on frametime
        """
        
        timestepScalar = dt / self.fixedTimestep
        if self.velocity.getZ() > self.terminalVelocity:
            self.velocity -= self.gravity * timestepScalar

    def groundSnap(self,dt):
        """
        Snaps player to the ground when travelling downwards
        """
        newPos = self.currentPos + self.velocity * dt
        slopeDownStep = tan(self.maxSlopeAngleRad) * (self.velocity.getXy().length() * self.fixedTimestep * 1.1) + 0.02
        bottomFrom = TransformState.makePos(newPos)
        self.bottomTo = TransformState.makePos(newPos - Vec3(0,0,slopeDownStep))
        bottomResult = self.world.sweepTestClosest(self.playerCollisionShape,bottomFrom,self.bottomTo,BitMask32(0x2),0)
        self.snappy = False
        if bottomResult.hasHit() and bottomResult.getNode().getName() != "floorButtonTrigger":
            hitPos = bottomResult.getHitPos()
            hitZ = hitPos.getZ()
            
            topFromPos = newPos + Vec3(0,0,hitZ - newPos.getZ())
            topFrom = TransformState.makePos(topFromPos)
            if hitZ > newPos.getZ():  # step up
                topToPos = topFromPos + Vec3(0,0,.0001)  # go up a negligible amount
            else:  # step down (usually a ramp)
                topToPos = newPos
            topTo = TransformState.makePos(topToPos)
            topResult = self.world.sweepTestClosest(self.playerCollisionShape,topFrom,topTo,BitMask32(0x2),0)
            if topResult.hasHit():
                return
            self.snappy = True
            self.currentPos.setZ(hitZ + 1)
    
    def applyLinearVelocity(self, dt: int):
        """
        Moves player position by velocity vector and calls penetration prevention function
        """
        
        self.currentPos += self.velocity * dt
        self.playerNP.setPos(self.currentPos)
    
    def resetPos(self):
        """
        Manually resets player position
        """
        
        self.velocity = Vec3()
        self.playerNP.setPos(self.startPos)
        self.camera.setHpr(self.startHpr)
        self.currentPos = self.playerNP.getPos()

# ====================
#
#   collision
#   - ground state
#   - bonk state
#   - collision prevention
#
# ====================

    def findGroundState(self,dt):  # see panda3d bullet sweep test
        """
        Sweep tests to check if player is on the ground
        """
        tsFrom = TransformState.makePos(self.currentPos)
        tsTo = TransformState.makePos(tsFrom.getPos() - Point3(0,0,.0001))
        mask = BitMask32.bit(3)
        mask.setBitTo(1,True)
        result = self.world.sweepTestClosest(self.playerCollisionShape,tsFrom,tsTo,mask,0)
        
        self.onFlatGround = False
        
        if not result.hasHit() and not self.onSlope:
            self.onGround = False
            return
        self.onGround = True

    def processCollisions(self,dt):
        """
        Prevents penetration into objects by moving current position by scaled manifold normal
        """
        
        result = self.world.contactTest(self.playerCollisionNode)
        contactList = []
        self.onSlope = False
        self.inPortal = False
        self.transitionNode = None
        
        for contact in result.getContacts():
            if contact.getNode1().getName() == "inportal trigger" and self.bluePortal.isPlaced and self.orangePortal.isPlaced:
                self.inPortal = True
            if contact.getNode1().getName() == "blue outportal trigger" and self.velocity.length() != 0:
                self.teleport(False)
                return
            elif contact.getNode1().getName() == "orange outportal trigger" and self.velocity.length() != 0:
                self.teleport(True)
                return
            if contact.getNode1().getName() == "levelEnd":
                self.transitionNode = contact.getNode1()
                return
        
        for contact in result.getContacts():
            contactNode = contact.getNode1()  # the collision node the player is in contact with
            if contactNode in contactList or type(contactNode) is bt.BulletGhostNode or contactNode is self.interactNode:
                continue
            contactList.append(contactNode)
            pairResult = self.world.contactTestPair(self.playerCollisionNode, contactNode)
                
            if contactNode.getMass() != 0:
                self.processDynamicCollision(pairResult, contactNode, dt)
            self.chastity(pairResult)
            self.playerNP.setPos(self.currentPos)
            
        if self.grabState or self.releaseState:
            self.interactEntity.processCollisions(dt)
        if self.releaseState and not self.inPortal:
            self.release(dt)
            
    def chastity(self,pairResult):  # prevent penetration for player
        normalList = []
        for pairContact in pairResult.getContacts():
            pulloutGame = Vec3()  # haha get it because its nothing but its meant to push you out of clip brushes
            pushoutGame = Vec3()
            mpoint = pairContact.getManifoldPoint()
            normal = mpoint.getNormalWorldOnB()
            node = pairContact.getNode1()
            
            if normal in normalList or node == self.interactNode or (self.inPortal and node.getName() == "portal collider"):
                continue
            normalList.append(normal)
            if mpoint.getDistance() < 0:
                pulloutGame -= normal * mpoint.getDistance()
                pushoutGame -= self.velocity.project(normal)
                if Vec3(0,0,1).angleDeg(normal) < self.maxSlopeAngleDeg:  # if angle of inclination is less than 45 deg
                    pulloutGame.setX(0)
                    pulloutGame.setY(0)
                    pushoutGame = Vec3()
                    self.onSlope = True
            
            self.currentPos += pulloutGame
            if pushoutGame.length() < 1000 and self.velocity.getXy().dot(normal.getXy()) < 0:
                self.velocity += pushoutGame
    
    def processDynamicCollision(self, pairResult, contactNode, dt):  # chat im cooked
        normalList = []
        for pairContact in pairResult.getContacts():
            mpoint = pairContact.getManifoldPoint()
            normal = mpoint.getNormalWorldOnB()
            node = pairContact.getNode1()
            
            if normal in normalList or node == self.interactNode:
                continue
            normalList.append(normal)
            
            if mpoint.getDistance() < 0:
                contactNode.applyImpulse(mpoint.getAppliedImpulse(), mpoint.getLocalPointB())

# ====================
#
#   interacts
#
# ====================

    def processUseState(self,camPos,rayTo,dt):  # prevent holding down the use key
        
        self.releaseState = False
        if not self.useState and self.inputWatcher.isSet("use"):
            self.useState = True
            self.use(camPos,rayTo,dt)
        elif not self.inputWatcher.isSet("use"):
            self.useState = False
        
    def use(self,camPos,rayTo,dt):
        
        if self.grabState:
            self.releaseState = True
            return
        
        result = self.world.rayTestAll(camPos, rayTo)
        for hit in result.getHits():
            node = hit.getNode()
            if node is None:
                return
            
            if node.getName() == "box" or node.getName() == "clone":
                self.portalGun.play("grab")
                self.startGrab = True
                self.interactNode = node
                return
         
    def release(self, dt):
        if self.world.contactTestPair(self.interactNode,self.playerCollisionNode).getNumContacts() == 0:
            self.portalGun.play("ungrab")
            self.grabState = False
            self.releaseState = False
            self.interactNP.setCollideMask(BitMask32.allOn())
            self.releaseKinematic = True
            self.interactNode.setLinearDamping(0)
            releaseVel = self.interactEntity.velocity
            newVel = releaseVel.normalized() * log(max(releaseVel.length() - self.velocity.length(), 0) + 1, 10) * 5
            newVel += self.velocity
            self.interactEntity.prephysicsReleaseVelocity = newVel
            self.interactEntity.velocity = newVel
            self.interactNode.setLinearVelocity(newVel)
    
    def updateReleasedEntity(self):
        self.releaseKinematic = False
        self.interactNode.setLinearVelocity(self.interactEntity.prephysicsReleaseVelocity)
        self.interactNode.setKinematic(False)
        self.interactEntity.cloneCollisionNode.setKinematic(False)
        self.interactEntity.isHeld = False
        self.interactNode = None
    
    def boxPull(self, rayDir, camPos, dt):  # pulls box to front of player when grabbed
        pos = self.interactNP.getPos()

        rayPos = camPos + rayDir.normalized() * self.grabLength
        pullDir = rayPos - pos
        if self.inPortal:
            self.interactEntity.forceInPortalState = True
            if (self.interactNP.getPos() - self.currentPos).length() > self.grabLength * 1.1:
                self.interactNP.setPos(self.currentPos)
                return
            self.interactEntity.velocity = Vec3()
            self.interactEntity.boxNP.setPos(rayPos)
            return
        self.interactEntity.forceInPortalState = False

        if (pos - self.currentPos).length() > 8:
            self.interactNP.setPos(rayPos)
            return
        elif (pos - self.currentPos).length() > self.maxHoldLength:
            self.releaseState = True
            return
        pullVec = pullDir.normalized() * self.maxPullSpeed
        if pullDir.length() > self.maxPullSpeed * dt:
            self.interactEntity.velocity = pullVec
        else:
            self.interactEntity.velocity = pullDir / dt
        
        self.interactNP.setHpr(self.camera.getH(),0,0)
        self.interactNP.setP(0)
    
    def setInteractEntity(self,entity):
        self.interactEntity = entity
        if self.interactNode.getName() == "clone":
            self.interactNode = self.interactEntity.boxCollisionNode
            self.interactEntity.teleport()
        self.interactNP = self.interactEntity.boxNP
        self.interactNode.setKinematic(True)
        self.interactEntity.cloneCollisionNode.setKinematic(True)
        self.interactEntity.isHeld = True
        self.interactNP.setCollideMask(BitMask32.bit(2))

# ====================
#
#   portals
#
# ====================

    def processShootState(self):
        if not self.inputWatcher.isSet("attack1") and not self.inputWatcher.isSet("attack2"):
            self.shootState = False
        elif self.inputWatcher.isSet("attack1") and not self.shootState and not self.grabState:
            self.shoot(False)
            self.shootState = True
        elif self.inputWatcher.isSet("attack2") and not self.shootState and not self.grabState:
            self.shoot(True)
            self.shootState = True
    
    def shoot(self,isOrange):
        self.portalGun.play("shoot")
        
        if isOrange:
            portal = self.orangePortal
            outPortal = self.bluePortal
        else:
            portal = self.bluePortal
            outPortal = self.orangePortal
        
        shootRot = Rot(self.camera.getH(),self.camera.getP(),self.camera.getR())
        shootVec = shootRot.getForward().normalized() * self.maxShootLength
        
        pFrom = self.currentPos + self.camera.getPos()
        pTo = pFrom + shootVec
        
        result = self.world.rayTestClosest(pFrom,pTo,BitMask32(0x0A))
        
        if result.hasHit():
            node = result.getNode()
            
            if node.getName() == "portal collider":
                hitPos = result.getHitPos()
                hitNormal = result.getHitNormal()
                hitNormal = Vec3(round(hitNormal.getX(),1),round(hitNormal.getY(),1),round(hitNormal.getZ(),1))
                
                if hitNormal.getZ() == 0:  # if its a wall; floors/ceilings are quaternion HELL
                    portalH = round(degrees(atan2(hitNormal.getY(),hitNormal.getX())),1)
                    if portalH % 15 > 1:  # idk like if its like a weird angle
                        return
                else:
                    # placement fail: surface not wall
                    self.portalGunSoundFail.play()
                    return
                
                # REPORTAL GO ZOOOOOOOOOOM
                if self.world.contactTestPair(self.playerCollisionNode,outPortal.inPortalTriggerNP.node()).getNumContacts():
                    self.velocity += outPortal.normal * 5
                
                testPoints = (
                    Vec3(sin(radians(portalH)),cos(radians(portalH)),0),
                    Vec3(-sin(radians(portalH)),-cos(radians(portalH)),0),
                    Vec3(0,0,1.5),
                    Vec3(0,0,-1.5))
                
                hitPos = self.fixOverhang(hitNormal,hitPos,testPoints)
                hitPos = self.fixLevelIntersection(hitNormal,hitPos,testPoints)
                hitPos = self.fixPortalIntersection(hitNormal,hitPos,outPortal,testPoints)
                
                if self.checkPlacement(outPortal,hitNormal,hitPos,testPoints):
                    # placement fail: cant find suitable position
                    self.portalGunSoundFail.play()
                    return
                # placement passes
                self.portalGunSound.play()
                self.placePortal(portal,outPortal,hitNormal,hitPos,portalH)
            else:
                # placement fail: invalid surface
                self.portalGunSoundFail.play()
        else:
            # placement fail: not hits
            self.portalGunSoundFail.play()
    
    def fixOverhang(self,hitNormal,pos,testPoints):
        portalPos = pos
        for i in range(4):
            point = testPoints[i] + pos
            
            # testing if point in wall
            inWall = False
            rayFrom = point + hitNormal * 0.01
            rayTo = point - hitNormal * 0.01
            result = self.world.rayTestAll(rayFrom,rayTo)
            for hit in result.getHits():
                if hit.getNode().getName() == "portal collider":
                    inWall = True
            
            # shifting portal
            if not inWall:
                rayFrom = rayTo
                rayTo = pos - hitNormal * 0.01
                shiftResult = self.world.rayTestAll(rayFrom,rayTo)
                for hit in shiftResult.getHits():
                    if hit.getNode().getName() == "portal collider":
                        offset = hit.getHitPos() - rayFrom
                        portalPos += offset
        return portalPos
    
    def fixLevelIntersection(self,hitNormal,pos,testPoints):
        portalPos = pos
        for i in range(4):
            point = testPoints[i]
            rayFrom = pos + hitNormal * 0.01
            rayTo = rayFrom + point
            shiftResult = self.world.rayTestAll(rayFrom,rayTo)
            for hit in shiftResult.getHits():
                if hit.getNode().getName() == "portal collider" or hit.getNode().getName() == "nonportal collider":
                    offset = hit.getHitPos() - rayFrom - point
                    portalPos += offset
        return portalPos
    
    def fixPortalIntersection(self,hitNormal,pos,outPortal,testPoints):
        portalPos = pos
        
        # towards left
        point = testPoints[0]
        if hitNormal.getX() != 0:
            sides = ("portalR","portalL")
        else:
            sides = ("portalL","portalR")
        rayFrom = pos - hitNormal * 0.01
        rayTo = rayFrom + point
        shiftResult = self.world.rayTestAll(rayFrom,rayTo)
        for hit in shiftResult.getHits():
            if hit.getNode() in outPortal.colliderNodes:
                offset = hit.getHitPos() - rayFrom - point
                if hit.getNode().getName() == sides[0]:  # inside
                    portalPos += point
                elif hit.getNode().getName() == sides[1]:  # outside
                    portalPos += offset
        # towards right
        point = testPoints[1]
        if hitNormal.getX() != 1:
            sides = ("portalR","portalL")
        else:
            sides = ("portalL","portalR")
        rayFrom = pos - hitNormal * 0.01
        rayTo = rayFrom + point
        shiftResult = self.world.rayTestAll(rayFrom,rayTo)
        for hit in shiftResult.getHits():
            if hit.getNode() in outPortal.colliderNodes:
                offset = hit.getHitPos() - rayFrom - point
                if hit.getNode().getName() == sides[0]:  # inside
                    portalPos += point * 2 + offset
                elif hit.getNode().getName() == sides[1]:  # outside
                    portalPos += offset
        return portalPos
    
    def checkPlacement(self,outPortal,hitNormal,pos,testPoints):  # True if invalid
        # checking overhang
        cornerCheck = 0
        for i in range(4):
            point = testPoints[i] + pos
            rayFrom = point + hitNormal * 0.01
            rayTo = point - hitNormal * 0.01
            result = self.world.rayTestAll(rayFrom,rayTo)
            for hit in result.getHits():
                if hit.getNode().getName() == "portal collider":
                    cornerCheck += 1
        if cornerCheck != 4:
            return True
        
        # checking level intersection
        for i in range(4):
            point = testPoints[i]
            rayFrom = pos + hitNormal * 0.01
            rayTo = rayFrom + point
            shiftResult = self.world.rayTestAll(rayFrom,rayTo)
            for hit in shiftResult.getHits():
                if hit.getNode().getName() == "portal collider" or hit.getNode().getName() == "nonportal collider":
                    return True
        
        # checking portal intersection
        if (pos - outPortal.portalFrame.getPos()).length() < 1.1:
            return True
        return False
    
    def placePortal(self,portal,outPortal,hitNormal,pos,h):
        portal.portalFrame.setH(h)
        portal.updatePos(pos)
        portal.normal = hitNormal
        portal.isPlaced = True
        
        outPortal.clipPlaneNP.removeNode()
        outPortal.renderState.clearClipPlane()
        
        portalClipPlane = Plane(hitNormal,pos + hitNormal * 0.01)
        clipPlaneNode = PlaneNode("clipPlane",portalClipPlane)
        outPortal.clipPlaneNP = self.worldNP.attachNewNode(clipPlaneNode)
        outPortal.renderState.setClipPlane(outPortal.clipPlaneNP)
        outPortal.camera.node().setInitialState(outPortal.renderState.getState())
    
    def teleport(self,isOrange):
        if isOrange:
            inPortal = self.orangePortal
            outPortal = self.bluePortal
        else:
            inPortal = self.bluePortal
            outPortal = self.orangePortal
        if self.velocity.getXy().dot(inPortal.normal.getXy()) < 0:  # if the play is moving towards into the portal
            self.currentPos = inPortal.camera.getPos() - Vec3(0,0,.5)
            self.playerNP.setPos(self.currentPos)
            self.velocity = Rot((outPortal.portalFrame.getH() - inPortal.portalFrame.getH()) + 180,0,0).xform(self.velocity)
            self.camera.setHpr(inPortal.camera.getHpr())
            self.camera.setR(0)
            if self.grabState:
                heading = radians(self.camera.getH())
                pitch = radians(self.camera.getP())
                camPos = Vec3(self.currentPos.getX(), self.currentPos.getY(), self.camera.getZ() + 1)
                rayDir = Vec3(-sin(heading), cos(heading), tan(pitch))
                rayPos = camPos + rayDir.normalized() * self.grabLength + Vec3(0,0,self.currentPos.getZ() - 1)
                self.interactEntity.velocity = Vec3()
                self.interactEntity.cloneNP.setPos(rayPos)
                self.interactEntity.teleport()
