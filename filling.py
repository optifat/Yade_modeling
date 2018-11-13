#!/usr/bin/python
# -*- coding: utf-8 -*-

from yade import pack, ymport, export, geom, bodiesHandling, qt
import math

class Material:
    def __init__(self, young, poisson, frictionAngle, density):
        self.young = young                            #[Pa]
        self.poisson = poisson
        self.frictionAngle = frictionAngle            #in radians
        self.density = density                        #[kg/m**3]

catalog = {}
#Other materials can be added here

catalog['Ni'] = Material(2e8, 0.3, 0.5, 8e3)

sample = catalog['Ni']     #studied material

boxHeight = 200
boxLength = 100
boxWidth = 1000
koeff = 3                         #the ration of length of plain to length of 'mountain'
gap = 50                          #distance between bottom of the box and knife
numberOfSpheres = 200             #number of generated spheres


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

v.eyePosition = Vector3(0,-boxWidth, 1.2*boxHeight)
v.viewDir = Vector3(0, 1, -0.5*boxHeight/boxWidth)
v.sceneRadius = 10*boxLength
v.screenSize = Vector2i(1280, 760)

#creating a box (like a stairs step)
#all positions considered according to starting point of view
box = []

#front wall
box.append(utils.facet([(boxLength, 0, 0), (-koeff*boxLength, 0, boxHeight),
                             (-koeff*boxLength, 0, 0)],
                             dynamic = False))
box.append(utils.facet([(boxLength, 0, 0), (-koeff*boxLength, 0, boxHeight),
                             (boxLength, 0, boxHeight)],
                             dynamic = False))

#back wall
box.append(utils.facet([(boxLength, boxWidth, 0), (-koeff*boxLength, boxWidth, boxHeight),
                             (-koeff*boxLength, boxWidth, 0)],
                             dynamic = False))
box.append(utils.facet([(boxLength, boxWidth, 0), (-koeff*boxLength, boxWidth, boxHeight),
                             (boxLength, boxWidth, boxHeight)],
                             dynamic = False))

#upper wall
box.append(utils.facet([(0, boxWidth, boxHeight), (boxLength, 0, boxHeight),
                             (0, 0, boxHeight)],
                             dynamic = False))
box.append(utils.facet([(0, boxWidth, boxHeight), (boxLength, 0, boxHeight),
                             (boxLength, boxWidth, boxHeight)],
                             dynamic = False))

#center wall
box.append(utils.facet([(0, 0, 0), (0, 0, boxHeight),
                             (0, boxWidth, 0)],
                             dynamic = False))
box.append(utils.facet([(0, boxWidth, boxHeight), (0, 0, boxHeight),
                             (0, boxWidth, 0)],
                             dynamic = False))

#bottom
box.append(utils.facet([(0, boxWidth, 0), (-koeff*boxLength, 0, 0),
                             (0, 0, 0)],
                             dynamic = False))
box.append(utils.facet([(0, boxWidth, 0), (-koeff*boxLength, 0, 0),
                             (-koeff*boxLength, boxWidth, 0)],
                             dynamic = False))

O.bodies.append(box)

factory = BoxFactory(maxParticles = numberOfSpheres, maxMass = -1,
        center = ((-koeff+1)*boxLength/2, boxWidth/2, boxHeight+3*gap),
        extents = ((koeff+1)*boxLength/2, boxWidth/2, 3*gap),
        vMin = 0, vMax = 0,
		PSDsizes = diamVal, PSDcum = massCum,
        PSDcalculateMass = False, exactDiam = True,
		vAngle = 0, massFlowRate = 2e12,
        normal = (0.0,0.0,-1.0), label = 'factory',
        mask = 0b11, silent = True, stopIfFailed = False
    )

#deletes sphere if it has fallen down
def delete():
    for body in O.bodies:
        if body.dict().get('state').pos[2] < -diamVal[-1]:
            O.bodies.erase(body.id)

#saves data
def save_data():
    if(O.running):
        return
    O.save('fast_data.yade')
    print('saved')

O.engines=[
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()], verletDist = diamVal[0]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    NewtonIntegrator(gravity = (0, 0, -9.81), damping = 0.1),
    factory,
    DomainLimiter(hi = (boxLength, boxWidth, 10*boxHeight), lo = (-5*boxLength, 0, 0),
                        nDo = numberOfSpheres),
    GlobalStiffnessTimeStepper(timeStepUpdateInterval = 100, timestepSafetyCoefficient = 0.8,
                               defaultDt = 5e-7, maxDt = 5e-4),
    PyRunner(iterPeriod = 1, command = 'save_data()'),
    PyRunner(iterPeriod = 10, command = 'delete()')
]

O.run()
