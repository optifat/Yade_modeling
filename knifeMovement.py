#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function
from yade import pack, ymport, export, geom, bodiesHandling, qt
import math
import os

O.load("fast_data.yade")

class Material:
    def __init__(self, young, poisson, frictionAngle, density):
        self.young = young                            #[Pa]
        self.poisson = poisson
        self.frictionAngle = frictionAngle            #in radians
        self.density = density                        #[kg/m**3]

catalog = {}
#Other materials can be added here

catalog['Ni'] = Material(2e8, 0.3, 0.5, 8e3)

sample = catalog['Ni']           #studied material

boxHeight = 200
boxLength = 100
boxWidth = 1000
gap = 50                          #distance between bottom of the box and knife
knifeVelocity = 30
vector = [2, 3]                  #vector of knife movement


diamVal = [2.75, 3.889, 5.50, 7.778, 11.0, 15.56, 22.00, 31.11, 44.00, 62.23, 88.00, 124.5]
massPer = [0.17, 0.460, 0.63, 2.650, 6.36, 13.82, 22.17, 40.21, 70.65, 90.64, 99.65, 100.0]
massCum = [percent/100 for percent in massPer]

O.materials.append(FrictMat(young = sample.young,
                            poisson = sample.poisson,
                            frictionAngle = sample.frictionAngle,
                            density = sample.density
                            ))

qt.Controller()
v = qt.View()

v.eyePosition = Vector3(0, 0, 3*boxHeight)
v.viewDir = Vector3(0, 0, -1)
v.sceneRadius = boxWidth
v.screenSize = Vector2i(1280, 760)


x, y = vector
module = (x**2 + y**2)**0.5
k = boxWidth/boxLength

if x!=0:
    a = math.atan(-y/x)
elif y<0:
    a = -math.pi/2
else:
    a = math.pi/2

knife = []

#creating a knife according to its velocity vector
if x<=0:
    knife.append(O.bodies.append([utils.facet([
                (boxLength*(1+k*math.cos(a))+boxWidth/2*math.sin(a), boxWidth/2*(1+math.cos(a))-k*boxLength*math.sin(a), boxHeight+gap),
                (boxLength*(1+k*math.cos(a))-boxWidth/2*math.sin(a), boxWidth/2*(1-math.cos(a))-k*boxLength*math.sin(a), boxHeight+gap),
                (boxLength*(1+k*math.cos(a))+boxWidth/2*math.sin(a), boxWidth/2*(1+math.cos(a))-k*boxLength*math.sin(a), 2*boxHeight+gap)
                                          ]),
                              utils.facet([
                (boxLength*(1+k*math.cos(a))+boxWidth/2*math.sin(a), boxWidth/2*(1+math.cos(a))-k*boxLength*math.sin(a), 2*boxHeight+gap),
                (boxLength*(1+k*math.cos(a))-boxWidth/2*math.sin(a), boxWidth/2*(1-math.cos(a))-k*boxLength*math.sin(a), boxHeight+gap),
                (boxLength*(1+k*math.cos(a))-boxWidth/2*math.sin(a), boxWidth/2*(1-math.cos(a))-k*boxLength*math.sin(a), 2*boxHeight+gap)
                                                                      ])
                                          ]))
else:
    knife.append(O.bodies.append([utils.facet([
                (boxLength*(1-k*math.cos(a))+boxWidth/2*math.sin(a), boxWidth/2*(1+math.cos(a))+k*boxLength*math.sin(a), boxHeight+gap),
                (boxLength*(1-k*math.cos(a))-boxWidth/2*math.sin(a), boxWidth/2*(1-math.cos(a))+k*boxLength*math.sin(a), boxHeight+gap),
                (boxLength*(1-k*math.cos(a))+boxWidth/2*math.sin(a), boxWidth/2*(1+math.cos(a))+k*boxLength*math.sin(a), 2*boxHeight+gap)
                                          ]),
                              utils.facet([
                (boxLength*(1-k*math.cos(a))+boxWidth/2*math.sin(a), boxWidth/2*(1+math.cos(a))+k*boxLength*math.sin(a), 2*boxHeight+gap),
                (boxLength*(1-k*math.cos(a))-boxWidth/2*math.sin(a), boxWidth/2*(1-math.cos(a))+k*boxLength*math.sin(a), boxHeight+gap),
                (boxLength*(1-k*math.cos(a))-boxWidth/2*math.sin(a), boxWidth/2*(1-math.cos(a))+k*boxLength*math.sin(a), 2*boxHeight+gap)
                                                                      ])
                                          ]))

#adding z-component of vector velocity; we want knife to move in xy plain only
vector.append(0)

O.engines=[
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb(), Bo1_Wall_Aabb()], verletDist = diamVal[0]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom(), Ig2_Wall_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    TranslationEngine(translationAxis = vector,
                      velocity = knifeVelocity,
                      ids = knife[0]),
    NewtonIntegrator(gravity = (0, 0, -9.81), damping = 0.1),
    GlobalStiffnessTimeStepper(timeStepUpdateInterval = 100, timestepSafetyCoefficient = 0.8,
                               defaultDt = 5e-7, maxDt = 5e-3)
]
