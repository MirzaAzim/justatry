# -*- coding: utf-8 -*-
from yade import pack
num_spheres=1000
compFricDegree = 30
targetPorosity = 0.43
finalFricDegree = 30
rate=-0.02
damp=0.2
stabilityThreshold=0.01
young=5e6
mn,mx=Vector3(0,0,0),Vector3(1,1,1)
O.materials.append(FrictMat(young=young,poisson=0.5,frictionAngle=radians(compFricDegree),density=2600,label='spheres'))
O.materials.append(FrictMat(young=young,poisson=0.5,frictionAngle=0,density=0,label='walls'))
walls=aabbWalls([mn,mx],thickness=0,material='walls')
wallIds=O.bodies.append(walls)
sp=pack.SpherePack()


clumps=False
if clumps:
 volume = (mx[0]-mn[0])*(mx[1]-mn[1])*(mx[2]-mn[2])
 mean_rad = pow(0.09*volume/num_spheres,0.3333)
 c1=pack.SpherePack([((-0.2*mean_rad,0,0),0.5*mean_rad),((0.2*mean_rad,0,0),0.5*mean_rad)])
 sp.makeClumpCloud(mn,mx,[c1],periodic=False)
 sp.toSimulation(material='spheres')
 O.bodies.updateClumpProperties()
else:
 sp.makeCloud(mn,mx,-1,0.3333,num_spheres,False, 0.95,seed=1)
 O.bodies.append([sphere(center,rad,material='spheres') for center,rad in sp])
triax=TriaxialStressController(
	maxMultiplier=1.0+2e4/young,
	finalMaxMultiplier=1.0+2e3/young,
	thickness = 0,
	stressMask = 7,
	internalCompaction=True,
)

newton=NewtonIntegrator(damping=damp)

O.engines=[
	ForceResetter(),
	InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Box_Aabb()]),
	InteractionLoop(
		[Ig2_Sphere_Sphere_ScGeom(),Ig2_Box_Sphere_ScGeom()],
		[Ip2_FrictMat_FrictMat_FrictPhys()],
		[Law2_ScGeom_FrictPhys_CundallStrack()]
	),
	GlobalStiffnessTimeStepper(active=1,timeStepUpdateInterval=100,timestepSafetyCoefficient=0.8),
	triax,
	newton
]
triax.goal1=triax.goal2=triax.goal3=-10000

while 1:
  O.run(1000, True)
  unb=unbalancedForce()
  print 'unbalanced force:',unb,' mean stress: ',triax.meanStress
  if unb<stabilityThreshold and abs(-10000-triax.meanStress)/10000<0.001:
    break

#triax.internalCompaction=False
bodiesToBeDeleted=[]
for b in O.bodies:
	if b.id in range(6):
		continue
	else:
		if b.state.pos[0]<.5:
			bodiesToBeDeleted.append(b)

for b in bodiesToBeDeleted:
	O.bodies.erase(b.id)
for b in O.bodies:
	if b.id in bodiesToBeDeleted:
		continue
triax.internalCompaction=False
triax.goal1=triax.goal2=triax.goal3=-12000
O.run(100,1)
