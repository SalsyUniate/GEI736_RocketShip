import pymunk
from pymunk.vec2d import Vec2d
import pygame
import math


pi = math.pi
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