import pymunk
from pymunk.vec2d import Vec2d
import pygame

import sys
import math
pi = math.pi
from copy import deepcopy as dcp
import random

import fuzzy as fz


pxPerM = 100 # px/m

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
	
	def draw(self, screen,offset):
		pos = self.shape.offset.rotated(self.shape.body.angle) + self.shape.body.position
		pygame.draw.circle(screen, "blue", (int(pxPerM*pos.x)+offset[0],int(pxPerM*pos.y)+offset[1]), int(pxPerM*self.shape.radius))
class PolyShape(Shape):
	def __init__(self, space,body, pnts, m):
		self.shape = pymunk.Poly(body, pnts)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		space.add(self.shape)
	
	def draw(self, screen,offset):
		pnts = []
		for v in self.shape.get_vertices():
			pos = v.rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append((int(pxPerM*pos.x)+offset[0],int(pxPerM*pos.y)+offset[1]))
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
	
	def draw(self, screen,offset):
		pygame.draw.lines(screen, "red", False, [(int(pxPerM*pnt[0])+offset[0],int(pxPerM*pnt[1])+offset[1]) for pnt in self.pnts], int(pxPerM*self.thick*2))
		for pnt in self.pnts : pygame.draw.circle(screen, "red", (int(pxPerM*pnt[0])+offset[0],int(pxPerM*pnt[1])+offset[1]), int(pxPerM*self.thick))

class Rocket:
	def __init__(self, space, w,h):
		self.body = pymunk.Body()
		space.add(self.body)
		
		self.hull = RectShape(space,self.body, w,h, 10)
		self.props = [CircleShape(space,self.body, w/4, 1), CircleShape(space,self.body, w/4, 1)]
		
		self.forcepos = [(-w/2, h/2),(w/2, h/2)]
		self.forceangle = [pi/4, pi/4]
		
		self.props[0].shape.unsafe_set_offset(self.forcepos[0])
		self.props[1].shape.unsafe_set_offset(self.forcepos[1])
	
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
		return self.body.angle
	def get_linvel(self):
		return self.body.velocity
	def get_rotvel(self):
		return self.body.angular_velocity
	
	def apply_force(self, l, r):
		self.body.apply_force_at_local_point((0,-l), self.forcepos[0])
		self.body.apply_force_at_local_point((0,-r), self.forcepos[1])
		return self
	
	def draw(self, screen, offset):
		self.hull.draw(screen, offset)
		for prop in self.props : prop.draw(screen, offset)

class Target:
	def __init__(self, pos):
		self.pos = pos
	def draw(self, screen,offset):
		pygame.draw.circle(screen, "red", (int(pxPerM*self.pos.x)+offset[0],int(pxPerM*self.pos.y)+offset[1]), 10)

def main():
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)
	
	rocket = Rocket(space, 0.5, 1).set_pos(Vec2d(2,0.1))
	target = Target(Vec2d(0,0))
	objects = [
		Ground(space, [(-1,8),(8,8)]),
		rocket,
		target
	]
	
	mode = 'normal'
	if mode == 'training':
		
		rocketposRangeX = (0, 8) ; rocketposRangeY = (0, 3)
		targetRangeX = (0, 8) ; targetRangeY = (0, 3)
		
		def fitness(sys):
			target.pos = Vec2d(random.uniform(*targetRangeX), random.uniform(*targetRangeY))
			rocket.set_pos(Vec2d(random.uniform(*rocketposRangeX), random.uniform(*rocketposRangeY)))
			rocket.set_angle(0)
			rocket.set_linvel(Vec2d(0,0))
			rocket.set_rotvel(0)
			
			t = 0
			while t <= 8:
				# fuzzy control
				err = target.pos - rocket.get_pos() ; vel = rocket.get_linvel()
				f = sys.compute({
					'errx': err.x, 'erry': err.y,
					'velx': vel.x, 'vely': vel.y,
					'angle': rocket.get_angle(), 'velangle': rocket.get_rotvel()
				})
				rocket.apply_force(f['fl'], f['fr'])
				
				space.step(1/20)
				
				t += dt
			
			return 1/(1+(err**2 + vel**2)) *20
		
		insets = [
			fz.SetGauss(0,1, -5,5,       tag='errx'),
			fz.SetGauss(0,1, -5,5,       tag='velx'),
			fz.SetGauss(0,1, -10,5,      tag='erry'),
			fz.SetGauss(0,1, -5,5,       tag='vely'),
			fz.SetGauss(0,1, 0,2*pi,     tag='angle'),
			fz.SetGauss(0,1, -8*pi,8*pi, tag='velangle')
		]
		outsets = [
			fz.SetGauss(0,1, 0,10000, tag='fl'),
			fz.SetGauss(0,1, 0,10000, tag='fr')
		]
		nbrules = 100
		rules = [ fz.Rule(fz.Cond(fz.AND(*[dcp(fin) for fin in insets])), dcp(fout)) for i in range(nbrules) for fout in outsets]
		
		pop = fz.Population(rules)
		pop.init(1000)
		pop.evolve(fitness, 100)
		
	else:
		
		pygame.init()
		ws = (1000, 600)
		screen = pygame.display.set_mode(ws)
		pygame.display.set_caption("Rocket control")
		clock = pygame.time.Clock()
		keysdown = {}
		fps = 60
		
		while True:
			for event in pygame.event.get():
				if event.type == pygame.QUIT : sys.exit(0)
			
			keys = pygame.key.get_pressed()
			if keys[pygame.K_ESCAPE] : sys.exit(0)
			if keys[pygame.K_LEFT]   : rocket.apply_force(100,0)
			if keys[pygame.K_RIGHT]  : rocket.apply_force(0,100)
			if keys[pygame.K_r]: # reset rocket
				rocket.set_pos(Vec2d(4,0))
				rocket.set_angle(0)
				rocket.set_linvel(Vec2d(0,0))
				rocket.set_rotvel(0) 
			
			screen.fill((255,255,255))
		
			space.step(1/fps)
			
			rocketpos = rocket.get_pos()
			offset = (ws[0]/2 - int(rocketpos.x*pxPerM), ws[1]/2 - int(rocketpos.y*pxPerM))
			for obj in objects : obj.draw(screen, offset)
		
			pygame.display.flip()
			clock.tick(fps)

if __name__ == '__main__':
    sys.exit(main())
