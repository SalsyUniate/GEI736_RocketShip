import pymunk
import numpy as np
import pygame

import sys

pxPerM = 100; # px/m

class Wall:
    def __init__(self, space,X,Y,thickness):
        self.space = space
        self.X = list(X)
        self.Y = list(Y)
        self.thickness = thickness
        self.body = pymunk.Segment(self.space.static_body,X,Y,thickness)
        self.body.elasticity = 0.25
        self.body.friction = 1
        self.space.add(self.body)
    def draw(self, screen):
        new_X,new_Y = [],[]
        for i in self.X : new_X.append(i*pxPerM)
        for i in self.Y : new_Y.append(i*pxPerM)
        pygame.draw.line(screen,'black',new_X,new_Y,int(self.thickness*2*pxPerM))

class Circle:
	def __init__(self, space, r, m):
		self.space = space
		
		self.body = pymunk.Body()
		self.body.position = 0, 0;
		self.shape = pymunk.Circle(self.body, r)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 1
		self.space.add(self.body, self.shape)
		
	def set_pos(self, x, y):
		self.body.position = x, y;
		return self
	def get_pos(self):
		return self.body.position;
	def get_r(self):
		return self.shape.radius
	
	def draw(self, screen):
		pos = self.get_pos()
		pygame.draw.circle(screen, (0,0,255), (int(pxPerM*pos.x), int(pxPerM*pos.y)), int(pxPerM*self.get_r()))

class Rocket:
	def __init__(self, space):
		self.space = space
		
		self.body = pymunk.Body()
		self.body.position = 0, 0;
		self.shape = pymunk.Poly(self.body, [(-0.25,-0.5),(-0.25,0.5),(0.25,0.5),(0.25,-0.5)], None, 0.1)
		self.shape.mass = 1
		self.shape.friction = 1
		self.shape.elasticity = 1
		self.space.add(self.body, self.shape)
		
	def set_pos(self, x, y):
		self.body.position = x, y;
		return self
	def get_pos(self):
		return self.body.position;
	def get_r(self):
		return self.shape.radius
	
	def draw(self, screen):
		pos = self.get_pos()
		box = [(int((pos.x-0.25)*pxPerM),int((pos.y-0.5)*pxPerM)),(int((pos.x-0.25)*pxPerM),int((pos.y+0.5)*pxPerM)),(int((pos.x+0.25)*pxPerM),int((pos.y+0.5)*pxPerM)),(int((pos.x+0.25)*pxPerM),int((pos.y-0.5)*pxPerM))]
		pygame.draw.polygon(screen, (0,0,255), box, int(pxPerM*self.get_r()))

fps = 60
def main():
	pygame.init()
	screen = pygame.display.set_mode((600, 600))
	pygame.display.set_caption("Rocket control")
	clock = pygame.time.Clock()
	
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)
	
	objects = [ Rocket(space).set_pos(1, 0.1), Circle(space, 0.1, 0.2).set_pos(1, 2), Wall(space,(0,4),(2,5),0.1), Wall(space,(2,5),(8,5),0.1) ]
	controller = objects[0]

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