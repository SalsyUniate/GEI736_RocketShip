import pymunk
from pymunk.vec2d import Vec2d
import numpy as np
pi = np.pi
import pygame
import pygame.gfxdraw

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
	
	def draw(self, screen,offset,color="blue"):
		pos = self.shape.offset.rotated(self.shape.body.angle) + self.shape.body.position
		pygame.draw.circle(screen, color, (int(pxPerM*pos.x)+offset[0], int(pxPerM*pos.y)+offset[1]), int(pxPerM*self.shape.radius))
class PolyShape(Shape):
	def __init__(self, space,body, pnts, m):
		self.shape = pymunk.Poly(body, pnts)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		space.add(self.shape)
	
	def draw(self, screen, offset,sprite):
		pnts = []
		for v in self.shape.get_vertices():
			pos = v.rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append((int(pxPerM*pos.x)+offset[0],int(pxPerM*pos.y)+offset[1]))
		pygame.draw.polygon(screen, sprite, pnts)
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
	
	def draw(self, screen, offset):
		pygame.draw.lines(screen, "red", False, [(int(pxPerM*pnt[0] + offset[0]),int(pxPerM*pnt[1] + offset[1])) for pnt in self.pnts], int(pxPerM*self.thick*2))
		for pnt in self.pnts : pygame.draw.circle(screen, "red", (int(pxPerM*pnt[0] + offset[0]),int(pxPerM*pnt[1] + offset[1])), int(pxPerM*self.thick))

class Rocket:
	def __init__(self, space, w,h):
		self.body = pymunk.Body()
		space.add(self.body)
		
		self.hull = PolyShape(space,self.body, [(-0.25,0.),(-0.1875,-0.25),(0.,-0.5),(0.1875,-0.25),(0.25,0.),(0.25,0.5),(-0.25,0.5)], 10)
		self.props = [PolyShape(space,self.body, [(-0.125,-0.0625),(-0.0625,-0.125),(0.0625,-0.125),(0.125,-0.0625),(0.0625,0.125),(-0.0625,0.125)], 1), PolyShape(space,self.body, [(-0.125,-0.0625),(-0.0625,-0.125),(0.0625,-0.125),(0.125,-0.0625),(0.0625,0.125),(-0.0625,0.125)], 1)]
		
		self.forcepos = [(-0.25, 0.5),(0.25, 0.5)]
		self.forceangle = [11*pi/8, 13*pi/8]
		self.sprites = (pygame.PixelArray(pygame.image.load("img/rocket_core.png")),pygame.PixelArray(pygame.image.load("img/booster_off.png")),pygame.PixelArray(pygame.image.load("img/booster_on.png")))
		
		self.props[0].shape.unsafe_set_vertices(self.props[0].shape.get_vertices(),pymunk.Transform.rotation(pi/2 + self.forceangle[1]))
		self.props[0].shape.unsafe_set_vertices(self.props[0].shape.get_vertices(),pymunk.Transform.translation(*self.forcepos[0]))
		self.props[1].shape.unsafe_set_vertices(self.props[1].shape.get_vertices(),pymunk.Transform.rotation(pi/2 + self.forceangle[0]))
		self.props[1].shape.unsafe_set_vertices(self.props[1].shape.get_vertices(),pymunk.Transform.translation(*self.forcepos[1]))
  
		self.propsvisibility = [False, False]
	
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
		self.body.apply_force_at_local_point((np.cos(self.forceangle[0])*l,np.sin(self.forceangle[0])*l), self.forcepos[0])
		self.body.apply_force_at_local_point((np.cos(self.forceangle[1])*r,np.sin(self.forceangle[1])*r), self.forcepos[1])
		return self
	
	def draw(self, screen, offset):
		self.hull.draw(screen,offset,self.sprites[0])
		for i in range(len(self.props)) : 
			if self.propsvisibility[i] :
				self.props[i].draw(screen,offset,self.sprites[2])
			else:
				self.props[i].draw(screen,offset,self.sprites[1])
class Target:
	def __init__(self, pos):
		self.pos = pos
	def draw(self, screen,offset):
		pygame.draw.circle(screen, "green", (int(pxPerM*self.pos.x)+offset[0],int(pxPerM*self.pos.y)+offset[1]), 10)    
    

keysdown = {}

fps = 60
def main():
	pygame.init()
	windowSize = (1300,700)
	simSize = (20,20)
	screen = pygame.display.set_mode(windowSize)
	pygame.display.set_caption("Rocket control")
	clock = pygame.time.Clock()
	
	
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)
	
	rocket = Rocket(space, 0.5, 1).set_pos((2,0.1))
	target = Target(Vec2d(15,8))
	objects = [
		Ground(space, [(0,0),(0,simSize[1]), (simSize[0],simSize[1]), (simSize[0],0),(0,0)]),
		rocket,
		target
	]	

	leftSelected = True
	boosterStrength = [100,100]

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT : sys.exit(0)
		
		keys = pygame.key.get_pressed()
		if keys[pygame.K_ESCAPE] : sys.exit(0)
		if keys[pygame.K_d]   : # booster gauche
			rocket.apply_force(boosterStrength[0],0)
			rocket.propsvisibility[0] = True
		else:
			rocket.propsvisibility[0] = False
		if keys[pygame.K_q]  : # booster gauche
			rocket.apply_force(0,boosterStrength[1])
			rocket.propsvisibility[1] = True
		else:
			rocket.propsvisibility[1] = False
		if keys[pygame.K_r]: # reset rocket
			rocket.set_pos(Vec2d(5,5))
			rocket.set_angle(0)
			rocket.set_linvel(Vec2d(0,0))
			rocket.set_rotvel(0) 
		if keys[pygame.K_LEFT]:
			leftSelected = True
		if keys[pygame.K_RIGHT]:
			leftSelected = False
		if keys[pygame.K_UP]:
			if leftSelected :
				boosterStrength[0] += 5
				if boosterStrength[0] > 100 :
					boosterStrength[0] = 100
			else :
				boosterStrength[1] += 5
				if boosterStrength[1] > 100 :
					boosterStrength[1] = 100
		if keys[pygame.K_DOWN]:
			if leftSelected :
				boosterStrength[0] -= 5
				if boosterStrength[0] < 0 :
					boosterStrength[0] = 0
			else :
				boosterStrength[1] -= 5
				if boosterStrength[1] < 0 :
					boosterStrength[1] = 0
		
		screen.fill((255,255,255))
	
		space.step(1/fps)
		
		offset = (-int(rocket.body.position.x*pxPerM)+windowSize[0]/2,-int(rocket.body.position.y*pxPerM)+windowSize[1]/2)

		for obj in objects : obj.draw(screen,offset)
  
		if leftSelected:
			pygame.draw.polygon(screen, "red", [(20,windowSize[1]/2-105),(20,windowSize[1]/2+105),(55,windowSize[1]/2+105),(55,windowSize[1]/2-105)])
			pygame.draw.polygon(screen, "black", [(windowSize[0]-20,windowSize[1]/2-105),(windowSize[0]-20,windowSize[1]/2+105),(windowSize[0]-55,windowSize[1]/2+105),(windowSize[0]-55,windowSize[1]/2-105)])
		else:
			pygame.draw.polygon(screen, "black", [(20,windowSize[1]/2-105),(20,windowSize[1]/2+105),(55,windowSize[1]/2+105),(55,windowSize[1]/2-105)])
			pygame.draw.polygon(screen, "red", [(windowSize[0]-20,windowSize[1]/2-105),(windowSize[0]-20,windowSize[1]/2+105),(windowSize[0]-55,windowSize[1]/2+105),(windowSize[0]-55,windowSize[1]/2-105)])

		pygame.draw.polygon(screen, "cyan", [(25,windowSize[1]/2+100),(50,windowSize[1]/2+100),(50,windowSize[1]/2+(100-boosterStrength[0]*2)),(25,windowSize[1]/2+(100-boosterStrength[0]*2))])
		pygame.draw.polygon(screen, "cyan", [(windowSize[0]-25,windowSize[1]/2+100),(windowSize[0]-50,windowSize[1]/2+100),(windowSize[0]-50,windowSize[1]/2+(100-boosterStrength[1]*2)),(windowSize[0]-25,windowSize[1]/2+(100-boosterStrength[1]*2))])

		minimapSize = (200,200)
		minimap = pygame.Surface(minimapSize)  # the size of your rect
		minimap.set_alpha(128)                # alpha level
		minimap.fill((128,128,128))           # this fills the entire surface
		pygame.draw.circle(minimap, "red", (int((rocket.body.position.x/simSize[0])*minimapSize[0]), int((rocket.body.position.y/simSize[1])*minimapSize[1])), 5)
		pygame.draw.circle(minimap, "green", (int((target.pos.x/simSize[0])*minimapSize[0]), int((target.pos.y/simSize[1])*minimapSize[1])), 5)
		screen.blit(minimap, (10,windowSize[1]-(minimapSize[1]+10))) 

		pygame.display.flip()
		clock.tick(fps)

if __name__ == '__main__':
    sys.exit(main())