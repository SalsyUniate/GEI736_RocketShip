import pymunk
import numpy as np
import pygame

import sys

pxPerM = 100; # px/m

class Object:
	def set_pos(self, x, y):
		self.body.position = x, y;
		return self
	def get_pos(self):
		return self.body.position;
class Circle(Object):
	def __init__(self, space, r, m):
		self.space = space
		
		self.body = pymunk.Body()
		self.body.position = 0, 0;
		self.shape = pymunk.Circle(self.body, r)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		self.space.add(self.body, self.shape)
		
	def get_r(self):
		return self.shape.radius
	
	def draw(self, screen):
		pos = self.get_pos()
		pygame.draw.circle(screen, "blue", (int(pxPerM*pos.x), int(pxPerM*pos.y)), int(pxPerM*self.get_r()))
class Poly(Object):
	def __init__(self, space, pnts, m):
		self.space = space
		self.body = pymunk.Body()
		
		self.shape = pymunk.Poly(self.body, pnts)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		self.space.add(self.body, self.shape)
	
	def draw(self, screen):
		pnts = []
		for v in self.shape.get_vertices():
			pos = v.rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append((int(pxPerM*pos.x),int(pxPerM*pos.y)))
		pygame.draw.polygon(screen, "blue", pnts)
class Rect(Poly):
	def __init__(self, space, w,h, m):
		pnts = [(-w/2,-h/2),(-w/2,h/2),(w/2,h/2),(w/2,-h/2)]
		Poly.__init__(self, space, pnts, m)
class Ground:
	def __init__(self, space, pnts, thick=0.05):
		self.space = space
		
		self.segments = [ pymunk.Segment(space.static_body, pnts[i], pnts[i-1], thick) for i in range(1,len(pnts)) ]
		for seg in self.segments:
			seg.elasticity = 0.7 
			seg.friction = 1
		
		# for the graphics
		self.pnts = [(int(pxPerM*pnt[0]),int(pxPerM*pnt[1])) for pnt in pnts]
		self.thick = thick
		
		self.space.add(*self.segments)
	
	def draw(self, screen):
		pygame.draw.lines(screen, "red", False, self.pnts, int(pxPerM*self.thick*2))
		for pnt in self.pnts : pygame.draw.circle(screen, "red", pnt, int(pxPerM*self.thick))
class Rocket(Object):
	def __init__(self, space, w,h, px, py, angle):
		self.hull = Rect(space, w,h, 10)
		self.props = [Rect(space, w/3,h/4, 1), Rect(space, w/3,h/4, 1)]
		

fps = 60
def main():
	pygame.init()
	screen = pygame.display.set_mode((600, 600))
	pygame.display.set_caption("Rocket control")
	clock = pygame.time.Clock()
	
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)
	
	objects = [
		Circle(space, 0.1, 0.2).set_pos(1, 0.1),
		Rect(space, 0.4, 0.2, 0.3).set_pos(4, 0.1),
		Ground(space, [(0,3),(2,3), (4,4), (5,4)])
	]
	
	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				sys.exit(0)
			elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
				sys.exit(0)
	
		screen.fill((255,255,255))
	
		space.step(1/fps)
		
		for obj in objects : obj.draw(screen)
	
		pygame.display.flip()
		clock.tick(fps)

if __name__ == '__main__':
    sys.exit(main())
