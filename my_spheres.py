#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function
from yade import pack,ymport,export,geom,bodiesHandling, qt
import math
import os

class Material:
    def __init__(self, young, poisson, frictionAngle, density):
        self.young = young                            #[Pa]
        self.poisson = poisson
        self.frictionAngle = frictionAngle            #in radians
        self.density = density                        #[kg/m**3]


catalog = {}
#Other materials can be added here
catalog['Ni'] = Material(2e8, 0.3, 0.5, 8e3)

sample = catalog[sys.argv[1]]                          #studied material

boxHeight = 500
boxLength = 1000
boxWidth = 100
gap = 50                           #distance between bottom of the box and knife
velocity = 5                       #velocity of knife; can be variated

Diams = [10.0, 22.14, 31.53, 48.55, 58.23, 100.0]
massCum = [       0.10, 0.50 , 0.90 , 0.95 , 1.0]

O.materials.append(FrictMat(young = sample.young,
                            poisson = sample.poisson,
                            frictionAngle = sample.frictionAngle,
                            density = sample.density
                            ))

qt.Controller()
v = qt.View()

#creating a П-like box (with bottom and two opposed XoY walls)
#all positions considered according to starting point of view
#front wall
O.bodies.append(utils.facet([(boxLength, 0, 0), (0, 0, boxHeight), (0, 0, 0)], dynamic = False))
O.bodies.append(utils.facet([(boxLength, 0, 0), (0, 0, boxHeight), (boxLength, 0, boxHeight)], dynamic = False))

#back wall
O.bodies.append(utils.facet([(boxLength, boxWidth, 0), (0, boxWidth, boxHeight), (0, boxWidth, 0)], dynamic = False))
O.bodies.append(utils.facet([(boxLength, boxWidth, 0), (0, boxWidth, boxHeight), (boxLength, boxWidth, boxHeight)], dynamic = False))

#bottom
O.bodies.append(utils.facet([(0, boxWidth, 0), (boxLength, 0, 0), (0, 0, 0)], dynamic = False))
O.bodies.append(utils.facet([(0, boxWidth, 0), (boxLength, 0, 0), (boxLength, boxWidth, 0)], dynamic = False))

Knifes = []
Knifes.append(O.bodies.append(yade.geom.facetBox([-boxLength/10, boxWidth/2, (boxHeight+gap)/2],
                            [boxLength/100, boxWidth/2, (boxHeight-gap)/2],
                            color = [1, 0, 0]
                            )))

knifeMovement = TranslationEngine(translationAxis=[1,0,0],velocity=velocity,ids=Knifes[0])

v.eyePosition = Vector3(boxLength/2,-boxWidth*10, boxHeight*0.7)
v.viewDir = Vector3(0, 1, -0.35*boxHeight/boxLength)
v.sceneRadius = boxLength
v.screenSize = Vector2i(1280, 760)

factory = BoxFactory(maxParticles=3000, maxMass=-1,
        extents=(boxLength/2, boxWidth/2, 1000), center=(boxLength/2, boxWidth/2, 1000),
        vMin=20.0, vMax=25.0,
		PSDsizes=Diams, PSDcum=massCum, PSDcalculateMass=True, exactDiam=False,
		vAngle=0, massFlowRate=2e12, normal=(0.0,0.0,-1.0), label='factory',
        mask=0b11, silent=True, stopIfFailed=False
    )


O.engines=[
   ForceResetter(),
   InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()], verletDist = Diams[0]),
   InteractionLoop(
      [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
      [Ip2_FrictMat_FrictMat_FrictPhys()],
      [Law2_ScGeom_FrictPhys_CundallStrack()]
   ),
   NewtonIntegrator(gravity=(0,0,-9.81),damping=0.1),
   factory,
   knifeMovement
]

O.dt= 5e-2
O.saveTmp()
