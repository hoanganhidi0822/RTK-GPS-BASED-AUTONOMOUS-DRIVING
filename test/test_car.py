#!/usr/bin/env python3
"""
3D Road Environment with Moving Car
---------------------------------------

This example uses Panda3D to simulate a simple 3D environment where a car
moves forward along a flat road. The camera is attached to the car to provide
a driver's view. The road is created using a CardMaker, and the car is loaded
as a "box" model (with a fallback if unavailable).

DISCLAIMER:
This code is a starting point for experimentation. The scene is very basic
and can be extended with better models, lighting, curves, and physics as needed.
"""

from direct.showbase.ShowBase import ShowBase
from panda3d.core import DirectionalLight, AmbientLight, Vec4, CardMaker, NodePath, ClockObject
from direct.task import Task

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        # Disable default camera control to allow custom camera positioning
        self.disableMouse()
        
        # --------------------------------------------------------
        # Create the Road using CardMaker (global import of CardMaker is used)
        # --------------------------------------------------------
        cm = CardMaker("road")
        # Set the frame: left, right, bottom, top
        cm.setFrame(-5, 5, 0, 200)
        self.road = self.render.attachNewNode(cm.generate())
        self.road.setP(-90)    # Rotate the card to lay flat horizontally
        self.road.setPos(0, 50, -1)
        self.road.setColor(0.2, 0.2, 0.2, 1)

        # --------------------------------------------------------
        # Create the Car: Try loading a box model; if not available, use a fallback.
        # --------------------------------------------------------
        try:
            # Try loading a pre-made box model (if available)
            self.car = self.loader.loadModel("models/box")
        except Exception as e:
            # Fallback: use a CardMaker to create a simple car shape.
            cm_car = CardMaker("car")
            cm_car.setFrame(-1, 1, -0.5, 0.5)
            self.car = self.render.attachNewNode(cm_car.generate())
        self.car.setScale(1, 2, 0.5)
        self.car.setPos(0, 0, 0)
        self.car.setColor(1, 0, 0, 1)  # Red colored car
        self.car.reparentTo(self.render)

        # --------------------------------------------------------
        # Attach the camera to the car for a driver's view
        # --------------------------------------------------------
        self.camera.reparentTo(self.car)
        self.camera.setPos(0, -10, 3)  # Positioned behind and above the car
        self.camera.lookAt(self.car)

        # --------------------------------------------------------
        # Set up basic lighting for the scene
        # --------------------------------------------------------
        dlight = DirectionalLight("dlight")
        dlight.setColor(Vec4(0.8, 0.8, 0.8, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(0, -60, 0)
        self.render.setLight(dlnp)

        alight = AmbientLight("alight")
        alight.setColor(Vec4(0.2, 0.2, 0.2, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        # --------------------------------------------------------
        # Add a task to update and move the car forward continuously
        # --------------------------------------------------------
        self.taskMgr.add(self.moveCarTask, "moveCarTask")

    def moveCarTask(self, task):
        dt = ClockObject.getGlobalClock().getDt()
        # Move the car forward along its local y-axis
        self.car.setPos(self.car, 0, dt * 10, 0)  # Speed: 10 units per second
        return Task.cont

# --------------------------------------------------------
# Run the application
# --------------------------------------------------------
app = MyApp()
app.run()