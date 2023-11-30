import pymunk
from pymunk.vec2d import Vec2d
from pymunk.transform import Transform
import pygame
import matplotlib.pyplot as plt

import sys
import math
pi = math.pi
cos = math.cos
sin = math.sin
from copy import deepcopy as dcp
import random

import fuzzy as fz


ws = (1000, 600) # screen dimensions in pixels
pxPerM = 100 # px/m
screen_center = Vec2d(ws[0]/2/pxPerM, -ws[1]/2/pxPerM) # screen center in physics coordinates
cam_t = Transform() # camera transform (zoom & translation)
def coord_phys2screen(coord, transfo): # transform physics coordinates to screen coordintaes
	tmp = transfo @ coord
	return (int(tmp[0]*pxPerM), -int(tmp[1]*pxPerM))

class Shape:
	def set_pos(self, pos):
		self.shape.body.position = pos
		self.shape.space.reindex_shapes_for_body(self.shape.body)
		return self
class CircleShape(Shape):
	def __init__(self, space,body, r, m):		
		self.shape = pymunk.Circle(body, r)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		space.add(self.shape)
	
	def draw(self, screen,transfo):
		pos = self.shape.offset.rotated(self.shape.body.angle) + self.shape.body.position
		pygame.draw.circle(screen, "blue", coord_phys2screen(pos,transfo), int(pxPerM*self.shape.radius))
class PolyShape(Shape):
	def __init__(self, space,body, pnts, m):
		self.shape = pymunk.Poly(body, pnts)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		space.add(self.shape)
	
	def draw(self, screen,transfo):
		pnts = []
		for v in self.shape.get_vertices():
			pos = v.rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append(coord_phys2screen(pos,transfo))
		pygame.draw.polygon(screen, "blue", pnts)
class RectShape(PolyShape):
	def __init__(self, space,body, w,h, m):
		pnts = [(-w/2,-h/2),(-w/2,h/2),(w/2,h/2),(w/2,-h/2)]
		PolyShape.__init__(self, space,body, pnts, m)

def shapeBody(ShapeClass, space, *args, **kwargs):
	body = pymunk.Body()
	space.add(body)
	shape = ShapeClass(space,body, *args, **kwargs)
	return shape

class Ground:
	def __init__(self, space, pnts, thick=0.05):		
		self.segments = [ pymunk.Segment(space.static_body, pnts[i], pnts[i-1], thick) for i in range(1,len(pnts)) ]
		for seg in self.segments:
			seg.elasticity = 0.7 
			seg.friction = 1
		
		# for the graphics
		self.pnts = [*pnts]
		self.thick = thick
		
		space.add(*self.segments)
	
	def draw(self, screen,transfo):
		pygame.draw.lines(screen, "red", False, [coord_phys2screen(pnt,transfo) for pnt in self.pnts], int(pxPerM*self.thick*2))
		#for pnt in self.pnts : pygame.draw.circle(screen, "red", coord_phys2screen(pnt,transfo), int(pxPerM*self.thick)) # line caps

class Rocket:
	def __init__(self, space, w,h):
		self.body = pymunk.Body()
		space.add(self.body)
		
		self.hull = RectShape(space,self.body, w,h, 10)
		self.props = [RectShape(space,self.body, h/2,w/2, 1), RectShape(space,self.body, h/2,w/2, 1)]
		
		self.forcepos = [(-3/4*w, -h/2),(3/4*w, -h/2)]
		self.forceangle = [pi/2,pi/2]
		
		for i in range(2) : self.props[i].shape.unsafe_set_vertices(
			self.props[i].shape.get_vertices(),
			pymunk.Transform.translation(*self.forcepos[i]).rotated(self.forceangle[i])
		)
	
	def set_pos(self, pos):
		self.body.position = pos
		self.body.space.reindex_shapes_for_body(self.body)
		return self
	def set_angle(self, angle):
		self.body.angle = angle
		self.body.space.reindex_shapes_for_body(self.body)
		return self
	def set_linvel(self, linvel):
		self.body.velocity = linvel
		return self
	def set_rotvel(self, rotvel):
		self.body.angular_velocity = rotvel
		return self
	
	def get_pos(self):
		return self.body.position
	def get_angle(self):
		return self.body.angle % (2*pi)
	def get_linvel(self):
		return self.body.velocity
	def get_rotvel(self):
		return self.body.angular_velocity
	
	def apply_damping(self, lin,rot):
		self.body.apply_force_at_world_point(-self.body.velocity*lin, self.body.position)
		# torque
		rot = -self.body.angular_velocity*rot
		self.body.apply_force_at_local_point(Vec2d(rot/2,0), Vec2d(0, -1))
		self.body.apply_force_at_local_point(Vec2d(-rot/2,0), Vec2d(0, 1))
	def apply_force(self, l, r):
		l = max(0,l) ; r = max(0,r) # only positive forces
		self.body.apply_force_at_local_point( (cos(self.forceangle[0])*l, sin(self.forceangle[0])*l), self.forcepos[0] )
		self.body.apply_force_at_local_point( (cos(self.forceangle[1])*r, sin(self.forceangle[1])*r), self.forcepos[1] )
		return self
	def stop(self):
		self.set_angle(0)
		self.set_linvel(Vec2d(0,0))
		self.set_rotvel(0)
		return self
	
	def draw_arrow(self, screen,transfo, p, color):
		pos0 = self.body.position
		pos1 = p + pos0
		pygame.draw.line(screen, color, coord_phys2screen(pos0,transfo), coord_phys2screen(pos1,transfo), 3)
	def draw(self, screen,transfo):
		self.hull.draw(screen,transfo)
		for prop in self.props : prop.draw(screen,transfo)
		# debug: force direction
		#self.draw_arrow(screen,transfo, Vec2d(cos(self.forceangle[1]),sin(self.forceangle[1]))*10, "red")

class Target:
	def __init__(self, pos):
		self.pos = pos
	def draw(self, screen,transfo):
		pygame.draw.circle(screen, "red", coord_phys2screen(self.pos,transfo), 10)

def main_manual():
	space = pymunk.Space()
	space.gravity = (0.0, -9.81)
	
	rocket = Rocket(space, 0.5, 1).set_pos(Vec2d(5,5))
	#j = pymunk.PivotJoint(space.static_body, rocket.body, rocket.get_pos()) ; space.add(j) # pin rocket
	target = Target(Vec2d(10,11))
	objects = [
		Ground(space, [(0,0),(20,0),(20,20),(0,20),(0,0)]),
		rocket,
		target
	]
	
	# Fuzzy systems ------------------------------------------------------------------------------------------
	autoOn = False
	nbsets = 5 # impaire
	mset = nbsets//2 # index of middle set
	# system (1) :: X -> force X
	fsets_errx = fz.SetGauss.autoN_cn(nbsets, True, tag='errx')
	fsets_velx = fz.SetGauss.autoN_cn(nbsets, True, tag='velx')
	fsets_fx = fz.SetGauss.autoN_cn(  nbsets, False, tag='fx') # output x-force
	rules1 = [fz.Rule(fz.AND(fsets_errx[i], fsets_velx[n]), fsets_fx[fz.clamp(0, -(i-mset+n-mset) + mset, nbsets-1)]) for i in range(nbsets) for n in range(nbsets)]
	sys1 = fz.System(rules1)
	#fz.Set.plotAll(fsets_errx) ; fz.Set.plotAll(fsets_velx) ; fz.Set.plotAll(fsets_fx) ; plt.show()
	# system (2) :: Y -> force Y
	fsets_erry = fz.SetGauss.autoN_cn(nbsets, True, tag='erry')
	fsets_vely = fz.SetGauss.autoN_cn(nbsets, True, tag='vely')
	fsets_fy = fz.SetGauss.autoN_cn(  nbsets, False, tag='fy') # output y-force
	rules2 = [fz.Rule(fz.AND(fsets_erry[i], fsets_vely[n]), fsets_fy[fz.clamp(0, -(i-mset+n-mset) + mset, nbsets-1)]) for i in range(nbsets) for n in range(nbsets)]
	sys2 = fz.System(rules2)
	#fz.Set.plotAll(fsets_erry) ; fz.Set.plotAll(fsets_vely) ; fz.Set.plotAll(fsets_fy) ; plt.show()
	# system (3) :: Angle -> increment thrust L & R
	fsets_erra = fz.SetGauss.autoN_cn(nbsets, True, tag='erra')
	fsets_vela = fz.SetGauss.autoN_cn(nbsets, True, tag='vela')
	fsets_fl = fz.SetGauss.autoN_cn(nbsets, False, tag='fl')
	fsets_fr = fz.SetGauss.autoN_cn(nbsets, False, tag='fr')
	rules3 = []
	for i in range(nbsets):
		for n in range(nbsets) :
			val = i-mset+n-mset
			indL = fz.clamp(0, val + mset, nbsets-1)
			indR = fz.clamp(0, -val + mset, nbsets-1)
			rules3 += [
				fz.Rule(fz.AND(fsets_erra[i], fsets_vela[n]), fsets_fl[indL]),
				fz.Rule(fz.AND(fsets_erra[i], fsets_vela[n]), fsets_fr[indR])
			]
	sys3 = fz.System(rules3)
	# ---------------------------------------------------------------------------------------------------------
	
	pygame.init()
	screen = pygame.display.set_mode(ws)
	pygame.display.set_caption("Rocket control")
	clock = pygame.time.Clock()
	keysdown = {}
	fps = 60
	while True:
		# user events
		for event in pygame.event.get():
			if event.type == pygame.QUIT : sys.exit(0)
			
			# get key events (no repeats)
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_r   : rocket.set_pos(Vec2d(5,5)).stop() # reset rocket
				elif event.key == pygame.K_a : autoOn = not autoOn # toggle auto control
		# get key downs (repeats)
		keys = pygame.key.get_pressed()
		if keys[pygame.K_ESCAPE] : sys.exit(0)
		if keys[pygame.K_LEFT]   : rocket.apply_force(100,0)
		if keys[pygame.K_RIGHT]  : rocket.apply_force(0,100)
		
		# physics
		rocket.apply_damping(2, 2)
		space.step(1/fps)
		
		rpos = rocket.get_pos()
		rangle = rocket.get_angle() + pi/2
		
		# controller
		if autoOn:
			err = rpos - target.pos
			vel = rocket.get_linvel()
			fx = sys1.compute({'errx': err.x/5, 'velx': vel.x/10}, 'fx')
			fy = sys2.compute({'erry': err.y/1, 'vely': vel.y/10}, 'fy')
			
			fa = math.atan2(abs(fy),fx)
			
			erra = rangle - fa
			if erra > pi : erra = -(2*pi - erra) # [-pi,pi] range
			r1 = sys3.compute({'erra': erra/pi*2, 'vela': rocket.get_rotvel()/pi/4}) # rotate towards force
			
			ftmp = Vec2d(fx, max(0,fy))
			f = Vec2d(1,1)*ftmp.length*120 + Vec2d(r1['fl'], r1['fr'])*400
			rocket.apply_force(f.x, f.y)
			
			#fa = math.atan2(fy,fx)
			#erra = rangle - fa
			#if erra > pi : erra = -(2*pi - erra) # [-pi,pi] range
			#r1 = sys3.compute({'erra': erra/pi*2, 'vela': rocket.get_rotvel()/pi/4}) # rotate towards force
			#
			#f = Vec2d(r1['fl']+1, r1['fr']+1) * 200
			#rocket.apply_force(f.x, f.y)
		
		# graphics
		screen.fill((255,255,255))
		
		off = rpos - screen_center
		cam_t = Transform.translation(-off.x, -off.y)
		
		for obj in objects : obj.draw(screen, cam_t)
		if autoOn:
			pass
			#rocket.draw_arrow(screen,cam_t, Vec2d(fx,abs(fy)), 'green')
		
		pygame.display.flip()
		clock.tick(fps)

# tune existing controller using GA
def main_training_tune():
	gains = [1/5, 1/10, 1/1, 1/10, 1/pi, 1/(2*pi), 120, 400]

# try to evolve a full controller using GA
def main_training_all():
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)
	
	rocket = Rocket(space, 0.5, 1).set_pos(Vec2d(2,0.1))
	target = Target(Vec2d(0,0))
	objects = [
		Ground(space, [(-1,8),(8,8)]),
		rocket,
		target
	]
	
	rocketposRangeX = (0, 8) ; rocketposRangeY = (0, 3)
	targetRangeX = (0, 8) ; targetRangeY = (0, 3)
	
	# fuzzy logic inputs
	insets = [
		fz.SetGauss(0,1, -5,5,       tag='errx'),
		fz.SetGauss(0,1, -5,5,       tag='velx'),
		fz.SetGauss(0,1, -10,5,      tag='erry'),
		fz.SetGauss(0,1, -5,5,       tag='vely'),
		fz.SetGauss(0,1, 0,2*pi,     tag='angle'),
		fz.SetGauss(0,1, -8*pi,8*pi, tag='velangle')
	]
	# fuzzy logic outputs
	outsets = [
		fz.SetGauss(0,1, 0,10000, tag='fl'),
		fz.SetGauss(0,1, 0,10000, tag='fr')
	]
	nbrules = 100
	rules = [ fz.Rule(fz.AND(*[dcp(fin) for fin in insets]), dcp(fout)) for i in range(nbrules) for fout in outsets]
	
	# GA population
	pop = fz.Population(rules)
	# GA fitness function
	def fitness():
		nbloops = 3
		res = 0
		for i in range(nbloops):
			target.pos = Vec2d(random.uniform(*targetRangeX), random.uniform(*targetRangeY))
			rocket.set_pos(Vec2d(random.uniform(*rocketposRangeX), random.uniform(*rocketposRangeY))).stop()
			
			t = 0 ; dt = 1/10
			while t <= 8:
				# fuzzy control
				err = target.pos - rocket.get_pos() ; vel = rocket.get_linvel()
				f = pop.system.compute({
					'errx': err.x, 'erry': err.y,
					'velx': vel.x, 'vely': vel.y,
					'angle': rocket.get_angle(), 'velangle': rocket.get_rotvel()
				})
				rocket.apply_force(f['fl'], f['fr'])
				
				space.step(dt)
				t += dt
			
			# fitness result for this loop
			res += 1 / ( 1e-9 + (err.get_length_sqrd() + vel.get_length_sqrd()) )
			
		return res / nbloops
	pop.init(100, 100, fitness)
	pop.evolve()

if __name__ == '__main__':
    sys.exit(main_manual())
