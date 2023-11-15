import pymunk
from pymunk.vec2d import Vec2d
import numpy as np
pi = np.pi
import pygame

import sys

pxPerM = 100; # px/m

class Shape:
	def set_pos(self, x, y):
		self.shape.body.position = Vec2d(x,y)
		self.shape.space.reindex_shapes_for_body(self.shape.body)
		return self
class CircleShape(Shape):
	def __init__(self, space,body, r, m):		
		self.shape = pymunk.Circle(body, r)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		space.add(self.shape)
	
	def draw(self, screen):
		pos = self.shape.offset.rotated(self.shape.body.angle) + self.shape.body.position
		pygame.draw.circle(screen, "blue", (int(pxPerM*pos.x), int(pxPerM*pos.y)), int(pxPerM*self.shape.radius))
class PolyShape(Shape):
	def __init__(self, space,body, pnts, m):
		self.shape = pymunk.Poly(body, pnts)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		space.add(self.shape)
	
	def draw(self, screen):
		pnts = []
		for v in self.shape.get_vertices():
			pos = v.rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append((int(pxPerM*pos.x),int(pxPerM*pos.y)))
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
		self.pnts = [(int(pxPerM*pnt[0]),int(pxPerM*pnt[1])) for pnt in pnts]
		self.thick = thick
		
		space.add(*self.segments)
	
	def draw(self, screen):
		pygame.draw.lines(screen, "red", False, self.pnts, int(pxPerM*self.thick*2))
		for pnt in self.pnts : pygame.draw.circle(screen, "red", pnt, int(pxPerM*self.thick))

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
	
	def set_pos(self, x, y):
		self.body.position = Vec2d(x,y)
		self.body.space.reindex_shapes_for_body(self.body)
		return self
	
	def apply_force(self, l, r):
		self.body.apply_force_at_local_point((0,-l), self.forcepos[0])
		self.body.apply_force_at_local_point((0,-r), self.forcepos[1])
		return self
	
	def draw(self, screen):
		self.hull.draw(screen)
		for prop in self.props : prop.draw(screen)

keysdown = {}

fps = 60
def main():
	pygame.init()
	screen = pygame.display.set_mode((600, 600))
	pygame.display.set_caption("Rocket control")
	clock = pygame.time.Clock()
	
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)
	
	rocket = Rocket(space, 0.5, 1).set_pos(2,0.1)
	objects = [
		shapeBody(CircleShape,space, 0.1, 0.2).set_pos(1, 0.1),
		shapeBody(RectShape,space, 0.4, 0.2, 0.3).set_pos(4, 0.1),
		Ground(space, [(0,4),(2,4.3), (4,4), (5,4)]),
		rocket
	]
	
	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT : sys.exit(0)
		
		keys = pygame.key.get_pressed()
		if keys[pygame.K_ESCAPE] : sys.exit(0)
		if keys[pygame.K_LEFT]   : rocket.apply_force(100,0)
		if keys[pygame.K_RIGHT]  : rocket.apply_force(0,100)
		
		screen.fill((255,255,255))
	
		space.step(1/fps)
		
		for obj in objects : obj.draw(screen)
	
		pygame.display.flip()
		clock.tick(fps)

if __name__ == '__main__':
    sys.exit(main())
