import pymunk
from pymunk.vec2d import Vec2d
import pygame
import math
import numpy as np


pi = math.pi
pxPerM = 100 # px/m


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
	
	def draw(self, screen, offset,color = "blue"):
		pnts = []
		for v in self.shape.get_vertices():
			pos = v.rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append((int(pxPerM*pos.x)+offset[0],int(pxPerM*pos.y)+offset[1]))
		pygame.draw.polygon(screen, color, pnts)
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
	def __init__(self, space, pnts, thick=0.20):		
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
		
		self.props[0].shape.unsafe_set_vertices(self.props[0].shape.get_vertices(),pymunk.Transform.rotation(self.forceangle[0]).translation(*self.forcepos[0]))
		self.props[1].shape.unsafe_set_vertices(self.props[1].shape.get_vertices(),pymunk.Transform.rotation(self.forceangle[1]).translation(*self.forcepos[1]))
  
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
	
	def apply_damping(self, lin,rot):
		self.body.apply_force_at_world_point(-self.body.velocity*lin, self.body.position)
		# torque
		rot = -self.body.angular_velocity*rot
		self.body.apply_force_at_local_point(Vec2d(rot/2,0), Vec2d(0, -1))
		self.body.apply_force_at_local_point(Vec2d(-rot/2,0), Vec2d(0, 1))
		
	def apply_force(self, l, r):
		self.body.apply_force_at_local_point((np.cos(self.forceangle[0])*l,np.sin(self.forceangle[0])*l), self.forcepos[0])
		self.body.apply_force_at_local_point((np.cos(self.forceangle[1])*r,np.sin(self.forceangle[1])*r), self.forcepos[1])
		return self
	
	def draw(self, screen, offset):
		self.hull.draw(screen,offset)
		for i in range(len(self.props)) : 
			if self.propsvisibility[i] :
				self.props[i].draw(screen,offset,"red")
			else:
				self.props[i].draw(screen,offset)
class Target:
	def __init__(self, pos):
		self.pos = pos
	def draw(self, screen,offset):
		pygame.draw.circle(screen, "green", (int(pxPerM*self.pos.x)+offset[0],int(pxPerM*self.pos.y)+offset[1]), 10)