import pymunk
from pymunk.vec2d import Vec2d
from pymunk.transform import Transform
import pygame
import matplotlib.pyplot as plt

import sys
import math
pi = math.pi ; cos = math.cos ; sin = math.sin
from copy import deepcopy as dcp
import random
from time import time

import fuzzy as fz

# object literal
Obj = lambda **kwargs: type("Object", (), kwargs)()

class Camera:
	def __init__(self, pos, screen, ratio):
		self.screen = screen
		self.r = ratio # pxPerM
		self.sz = Vec2d(*screen.get_size()) # px size
		self.pos = pos
	def conv_coord(self, coord): # convert from physics coord to this camera's screen coord
		tmp = (-self.pos*self.r + self.sz/2) + coord*self.r
		return (int(tmp[0]), int(-tmp[1] + self.sz.y))

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
	def draw(self, cam):
		pos = self.shape.offset.rotated(self.shape.body.angle) + self.shape.body.position
		pygame.draw.circle(cam.screen, "blue", cam.conv_coord(pos), int(cam.r*self.shape.radius))
class PolyShape(Shape):
	def __init__(self, space,body, pnts, m):
		self.shape = pymunk.Poly(body, pnts)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		
		space.add(self.shape)
	
	def draw(self, cam):
		pnts = []
		for v in self.shape.get_vertices():
			pos = v.rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append(cam.conv_coord(pos))
		pygame.draw.polygon(cam.screen, "blue", pnts)
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
		self.pnts = [Vec2d(*pnt) for pnt in pnts]
		self.thick = thick
		
		space.add(*self.segments)
	
	def draw(self, cam):
		pygame.draw.lines(cam.screen, "black", False, [cam.conv_coord(pnt) for pnt in self.pnts], int(pxPerM*self.thick*2))
	def drawmini(self, cam):
		pygame.draw.lines(cam.screen, "black", False, [cam.conv_coord(pnt) for pnt in self.pnts], 3)

class Target:
	def __init__(self, pos, r, dt):
		self.pos = pos
		self.r = r
		self.dt = dt
	def draw(self, cam):
		pygame.draw.circle(cam.screen, "red", cam.conv_coord(self.pos), int(self.r*cam.r), max(1,int(0.05*cam.r)))
	def drawmini(self, cam):
		pygame.draw.circle(cam.screen, "red", cam.conv_coord(self.pos), 4)
class Rocket:
	def __init__(self, space, w,h):
		self.set_targets([])
		
		self.controller = None
		self.auto = True
		
		self.body = pymunk.Body()
		space.add(self.body)
		
		self.hull = RectShape(space,self.body, w,h, 10)
		self.props = [RectShape(space,self.body, h/2,w/2, 1), RectShape(space,self.body, h/2,w/2, 1)]
		
		self.forcepos = [(-3/4*w, -h/2),(3/4*w, -h/2)]
		self.forceangle = [pi/2,pi/2]
		self.refresh_applied_f()
		
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
		fl = Vec2d( cos(self.forceangle[0])*l, sin(self.forceangle[0])*l )
		fr = Vec2d( cos(self.forceangle[1])*r, sin(self.forceangle[1])*r )
		
		self.body.apply_force_at_local_point(fl, self.forcepos[0] )
		self.body.apply_force_at_local_point(fr, self.forcepos[1] )
		
		self.applied_f[0] += fl
		self.applied_f[1] += fr
		
		return self
	def refresh_applied_f(self):
		self.applied_f = [Vec2d(0,0), Vec2d(0,0)] # left, right
	def stop(self):
		self.set_angle(0)
		self.set_linvel(Vec2d(0,0))
		self.set_rotvel(0)
		return self
	
	def set_targets(self, targets):
		self.targets = targets
		self.tind = 0 # current target index
		self.target = None if len(self.targets) == 0 else self.targets[0] # current target
		self.lastT = None # target start time
	def in_target(self):
		if self.target is None : return False
		return (self.body.position - self.target.pos).length <= self.target.r
	def update_target(self, dt):
		if not self.in_target() : self.lastT = None
		else:
			if self.lastT is None : self.lastT = dt
			elif dt - self.lastT >= self.target.dt:
				self.lastT = 0
				self.tind = min(self.tind+1, len(self.targets)-1)
				self.target = self.targets[self.tind]
	
	def set_controller(self, controller):
		self.controller = controller
		return self
	def apply_control(self):
		if not self.auto : return
		
		target = self.targets[self.tind]
		
		fl, fr = self.controller.action(
			self.get_pos() - target.pos,
			self.get_linvel(),
			self.get_angle() + pi/2, # [0, 2pi]
			self.get_rotvel(),
		)
		self.apply_force(fl, fr)
	
	def draw_arrow(self, cam, p, color):
		pos0 = self.body.position
		pos1 = p + pos0
		pygame.draw.line(cam.screen, color, cam.conv_coord(pos0), cam.conv_coord(pos1), 3)
	def draw(self, cam):
		self.hull.draw(cam)
		for prop in self.props : prop.draw(cam)
	def drawmini(self, cam): # for minimap : display as triangle
		pos = Vec2d(*cam.conv_coord(self.body.position))
		h = 12 ; w = 6
		t = Transform.rotation(-self.get_angle())
		pnts = [pos + t @ Vec2d(*pnt) for pnt in [(0,-h/2),(-w/2,h/2),(w/2,h/2)]]
		pygame.draw.polygon(cam.screen, "green", pnts)

class GUI : pass
class LevelBar(GUI):
	def __init__(self, w,h, x,y, maxl):
		self.w = w; self.h = h;
		self.x = x; self.y = y;
		
		self.maxl = maxl
		self.l = 0
	
	def draw(self, cam):
		x,w = [self.x*cam.sz[0], self.w*cam.sz[0]]
		y,h = [self.y*cam.sz[1], self.h*cam.sz[1]]
		bgsz = 10 # px
		wbg,hbg = [w+bgsz, h+bgsz]
		pygame.draw.rect(cam.screen, "black", [int(v) for v in [x-wbg/2, y-hbg/2, wbg, hbg]]) # bg
		# modify full height depending on level
		htmp  = self.l * h / self.maxl
		pygame.draw.rect(cam.screen, "cyan", [int(v) for v in [x-w/2, y+h/2-htmp, w, htmp]]) # color bar

# Fuzzy controllers
class Controller : pass
class Controller_alix1(Controller):
	def __init__(self):
		
		nbsets = 5 # impaire
		mset = nbsets//2 # index of middle set
		# system (1) :: X -> force X
		fsets_errx = fz.SetGauss.autoN_cn(nbsets, True, tag='errx')
		fsets_velx = fz.SetGauss.autoN_cn(nbsets, True, tag='velx')
		fsets_fx = fz.SetGauss.autoN_cn(  nbsets, False, tag='fx') # output x-force
		rules1 = [fz.Rule(fz.AND(fsets_errx[i], fsets_velx[n]), fsets_fx[fz.clamp(0, -(i-mset+n-mset) + mset, nbsets-1)]) for i in range(nbsets) for n in range(nbsets)]
		self.sys1 = fz.System(rules1)
		#fz.Set.plotAll(fsets_errx) ; fz.Set.plotAll(fsets_velx) ; fz.Set.plotAll(fsets_fx) ; plt.show()
		# system (2) :: Y -> force Y
		fsets_erry = fz.SetGauss.autoN_cn(nbsets, True, tag='erry')
		fsets_vely = fz.SetGauss.autoN_cn(nbsets, True, tag='vely')
		fsets_fy = fz.SetGauss.autoN_cn(  nbsets, False, tag='fy') # output y-force
		rules2 = [fz.Rule(fz.AND(fsets_erry[i], fsets_vely[n]), fsets_fy[fz.clamp(0, -(i-mset+n-mset) + mset, nbsets-1)]) for i in range(nbsets) for n in range(nbsets)]
		self.sys2 = fz.System(rules2)
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
		self.sys3 = fz.System(rules3)
		
	# must return (thrust left ; thrust right)
	def action(self, err,vel,rangle,rvel):
		fx = self.sys1.compute({'errx': err.x/5, 'velx': vel.x/5}, 'fx')
		fy = self.sys2.compute({'erry': err.y/1, 'vely': vel.y/5}, 'fy')
		
		f1 = Vec2d(fx, fy) # force to target
		f2 = Vec2d(cos(rangle), sin(rangle)) # rocket dir
		f3 = Vec2d(0, 1) # vertical
		
		# which angle should rocket turn towards ?
		ftot = f1.normalized() + f3.normalized()*4 # which is more important : staying vertical or going towards target ?
		fa = math.atan2(abs(ftot.y),ftot.x) # don't turn towards -y
		
		erra = rangle - fa
		if erra > pi : erra = -(2*pi - erra) # [-pi, pi] range
		erra = fz.clamp(-pi/5, erra, pi/5) # [-pi/5, pi/5] range
		r1 = self.sys3.compute({'erra': erra/(2/5*pi), 'vela': rvel/10}) # rotate towards angle
		
		thrust = max(0, f1.dot(f2)) * 200 # more thrust if rocket is facing target
		
		f = Vec2d(1,1)*thrust + Vec2d(r1['fl'], r1['fr'])*200
		return (f.x,f.y)

# graphics + physics loop (interactive)
def main_manual():
	global pxPerM
	
	# Physics init ----------------------------------------------------------------------------------------------
	space = pymunk.Space()
	space.gravity = (0.0, -9.81)
	
	rockets = [
		Rocket(space, 0.5, 1).set_pos(Vec2d(i,5)).set_controller(Controller_alix1()) \
		for i in range(1)
	]
	rind = 0 # focused rocket index
	#j = pymunk.PivotJoint(space.static_body, rocket.body, rocket.get_pos()) ; space.add(j) # pin rocket
	
	waypoints = [(2,0),(8,8),(-5, -10)]
	targets = [ Target(Vec2d(*p), 1, 4) for p in waypoints]
	for rocket in rockets : rocket.set_targets(targets)
	
	cage_w = 40 ; cage_h = 30
	ground = Ground(space, [(-cage_w/2,cage_h/2),(-cage_w/2,-cage_h/2),(cage_w/2,-cage_h/2),(cage_w/2,cage_h/2),(-cage_w/2,cage_h/2)])
	
	physObjects = [ground, *rockets, *targets]
	# -----------------------------------------------------------------------------------------------------------
	
	# Fuzzy control ------------------------------------------------------------------------------------------
	# ---------------------------------------------------------------------------------------------------------
	
	# Graphics init -------------------------------------------------------------------------------------------
	ws = (1000, 600) # screen dimensions in pixels
	pxPerM = 100 # px/m
	pygame.init()
	screen = pygame.display.set_mode(ws)
	pygame.display.set_caption("Rocket control")
	clock = pygame.time.Clock()
	keysdown = {}
	fps = 60
	maincam = Camera(Vec2d(0,0), screen, pxPerM)
	
	mms = (ws[0]/5, ws[1]/5)
	minimap = pygame.Surface(mms)
	mmcam = Camera(Vec2d(0,0), minimap, pxPerM/40) # minimap camera
	mmdots = 3 # minimap dot size
	#minimap.set_alpha(128)
	
	thrustbars = [
		LevelBar(5/100,50/100, 5/100,50/100, 300), # left
		LevelBar(5/100,50/100, 95/100,50/100, 300) # right
	]
	guiObjects = [*thrustbars]
	# ----------------------------------------------------------------------------------------------------------
	
	while True:
		# physics
		rocket.apply_damping(2, 2)
		space.step(1/fps)
		for rocket in rockets : rocket.refresh_applied_f()
		
		rpos = rockets[rind].get_pos()
		
		# user events
		for event in pygame.event.get():
			if event.type == pygame.QUIT : sys.exit(0)
			
			# get key events (no repeats)
			elif event.type == pygame.KEYDOWN:
				if   event.key == pygame.K_r : rockets[rind].set_pos(Vec2d(5,5)).stop() # reset rocket
				elif event.key == pygame.K_a : rockets[rind].auto = not rockets[rind].auto # toggle auto control
				elif event.key == pygame.K_i : maincam.r *= 120/100 # zoom in
				elif event.key == pygame.K_o : maincam.r *= 80/100 # zoom out
		# get key downs (repeats)
		keys = pygame.key.get_pressed()
		if keys[pygame.K_ESCAPE] : sys.exit(0)
		if keys[pygame.K_LEFT]   : rockets[rind].apply_force(100,0)
		if keys[pygame.K_RIGHT]  : rockets[rind].apply_force(0,100)
		
		# controller
		for rocket in rockets:
			rocket.update_target(time())
			rocket.apply_control()
		
		# graphics
		screen.fill((255,255,255))
		
		applied_f = rockets[rind].applied_f
		for i in [0,1] : thrustbars[i].l = applied_f[i].length
		
		maincam.pos = rpos
		for obj in physObjects+guiObjects : obj.draw(maincam)
		
		# minimap
		minimap.fill((128,128,128))
		for obj in physObjects : obj.drawmini(mmcam)
		screen.blit(minimap, (10,ws[1]-(mms[1]+10))) 
		
		pygame.display.flip()
		clock.tick(fps)
# physics loop (non-interactive, for training)
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
