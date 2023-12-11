import pymunk
from pymunk.vec2d import Vec2d
from pymunk.transform import Transform
I = pymunk.Transform.identity

import pygame as pg
import pygame_gui as pgui

import matplotlib.pyplot as plt

import sys
import math
pi = math.pi ; cos = math.cos ; sin = math.sin
from copy import deepcopy as dcp
import random
from time import time
import os

import fuzzy as fz

# object literal
Obj = lambda **kwargs: type("Object", (), kwargs)()

class Camera:
	def __init__(self, pos, screen, ratio, offscreen=None):
		self.screen = screen
		self.offscreen = offscreen
		self.r = ratio # pxPerM
		self.sz = Vec2d(*screen.get_size()) # px size
		self.pos = pos
	def conv_coord(self, coord): # convert from physics coord to this camera's screen coord
		tmp = (-self.pos*self.r + self.sz/2) + coord*self.r
		return (int(tmp[0]), int(-tmp[1] + self.sz.y))
	def flip_screen(self):
		tmp = self.screen
		self.screen = self.offscreen
		self.offscreen = tmp
	
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
		self.color = 'blue'
		
		space.add(self.shape)
	def draw(self, cam):
		pos = self.shape.offset.rotated(self.shape.body.angle) + self.shape.body.position
		pg.draw.circle(cam.screen, self.color, cam.conv_coord(pos), int(cam.r*self.shape.radius))
class PolyShape(Shape):
	def __init__(self, space,body, pnts, m):
		self.shape = pymunk.Poly(body, pnts)
		self.shape.mass = m
		self.shape.friction = 1
		self.shape.elasticity = 0.7
		self.color = 'blue'
		
		space.add(self.shape)
	def draw(self, cam, transfo=I()):
		pnts = []
		for v in self.shape.get_vertices():
			pos = (transfo @ v).rotated(self.shape.body.angle) + self.shape.body.position
			pnts.append(cam.conv_coord(pos))
		pg.draw.polygon(cam.screen, self.color, pnts)
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
		self.color = "black"
		
		space.add(*self.segments)
	
	def draw(self, cam):
		pg.draw.lines(cam.screen, self.color, False, [cam.conv_coord(pnt) for pnt in self.pnts], int(pxPerM*self.thick*2))
	def drawmini(self, cam):
		pg.draw.lines(cam.screen, self.color, False, [cam.conv_coord(pnt) for pnt in self.pnts], 3)

class Goal : pass
class Target(Goal):
	def __init__(self, pos, r, dt):
		self.pos = pos
		self.r = r
		self.dt = dt
	def draw(self, cam):
		pg.draw.circle(cam.screen, "red", cam.conv_coord(self.pos), int(0.1*cam.r))
		pg.draw.circle(cam.screen, "red", cam.conv_coord(self.pos), int(self.r*cam.r), max(1,int(0.05*cam.r)))
	def drawmini(self, cam):
		pg.draw.circle(cam.screen, "red", cam.conv_coord(self.pos), 4)
class Platform(Goal, Ground):
	def __init__(self, space, pos, w, dt):
		Ground.__init__(self, space, [pos-Vec2d(w/2,0), pos+Vec2d(w/2,0)])
		self.pos = pos
		self.w = w
		self.dt = dt
		
		self.color = "red"	

class Rocket:
	def __init__(self, space, angle=0):
		self.set_goals([])
		
		self.controller = None
		self.auto = True
		
		self.body = pymunk.Body()
		self.space = space
		self.space.add(self.body)
		
		self.refresh_applied_f()
		
		self.forceangle = [pi/2-angle, pi/2+angle]
		self.forcepos = [0,0]	
	
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
	def get_pos_b(self): # bottom of rocket
		return self.body.local_to_world(self.bottom)
	def get_angle(self):
		return self.body.angle % (2*pi)
	def get_linvel(self):
		return self.body.velocity
	def get_rotvel(self):
		return self.body.angular_velocity
	
	def set_color(self, color):
		self.hull.color = color
		for prop in self.props : prop.color = color
	
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
	
	def reset(self):
		self.stop()
		self.refresh_applied_f()
		self.set_goals(self.goals)
	
	def set_goals(self, goals):
		self.goals = goals
		self.tind = 0 # current target index
		self.goal = None if len(self.goals) == 0 else self.goals[0] # current target
		self.lastT = None # target start time
	def in_goal(self):
		if self.goal is None : return False
		
		if isinstance(self.goal, Target)     : return self.in_target()
		elif isinstance(self.goal, Platform) : return self.on_platform()
	def in_target(self):
		return (self.body.position - self.goal.pos).length <= self.goal.r
	def on_platform(self):
		contacts = self.props[0].shape.shapes_collide(self.goal.segments[0]).points + self.props[1].shape.shapes_collide(self.goal.segments[0]).points
		return len(contacts) != 0 and self.get_linvel().length < 1e-2
	def update_goal(self, dt):
		if not self.in_goal() : self.lastT = None
		else:
			if self.lastT is None : self.lastT = dt
			elif dt - self.lastT >= self.goal.dt:
				self.lastT = 0
				self.tind = min(self.tind+1, len(self.goals)-1)
				self.goal = self.goals[self.tind]
	
	def set_controllers(self, controllers):
		self.controllers = controllers
		return self
	def apply_control(self):
		if not self.auto : return
		
		goal = self.goals[self.tind]
		
		if isinstance(goal, Target):
			fl, fr = self.controllers[Target].action(
				self.get_pos_b() - goal.pos,
				self.get_linvel(),
				self.get_angle() + pi/2, # [0, 2pi]
				self.get_rotvel(),
			)
		elif isinstance(goal, Platform):
			fl, fr = self.controllers[Platform].action(
				self.get_pos_b(),
				self.get_linvel(),
				self.get_angle() + pi/2, # [0, 2pi]
				self.get_rotvel(),
				
				goal.pos, goal.w
			)
		
		self.apply_force(fl, fr)
	
	def draw_arrow(self, cam, p, color):
		pos0 = self.body.position
		pos1 = p + pos0
		pg.draw.line(cam.screen, color, cam.conv_coord(pos0), cam.conv_coord(pos1), 3)
	def draw(self, cam, transfo1=I(),transfo2=I()):
		self.hull.draw(cam, transfo1)
		for prop in self.props : prop.draw(cam, transfo2)
	def drawmini(self, cam): # for minimap : display as triangle
		pos = Vec2d(*cam.conv_coord(self.body.position))
		h = 12 ; w = 6
		t = Transform.rotation(-self.get_angle())
		pnts = [pos + t @ Vec2d(*pnt) for pnt in [(0,-h/2),(-w/2,h/2),(w/2,h/2)]]
		pg.draw.polygon(cam.screen, "green", pnts)
class Rocket_fancy(Rocket):
	def __init__(self, *args, **kwargs):
		Rocket.__init__(self, *args, **kwargs)
		
		hullpnts = [(-0.25,0.),(-0.1875,0.25),(0.,0.5),(0.1875,0.25),(0.25,0.),(0.25,-0.5),(-0.25,-0.5)]
		proppnts = [(-0.125,0.0625),(-0.0625,0.125),(0.0625,0.125),(0.125,0.0625),(0.0625,-0.125),(-0.0625,-0.125)]
		self.hull = PolyShape(self.space,self.body, hullpnts, 10)
		self.props = [
			PolyShape(self.space,self.body, proppnts, 1), 
			PolyShape(self.space,self.body, proppnts, 1)
		]
		
		self.forcepos = [(-0.25, -0.5),(0.25, -0.5)]
		self.bottom = Vec2d(0, -0.5)
		
		for i in range(2) : self.props[i].shape.unsafe_set_vertices(
			self.props[i].shape.get_vertices(),
			pymunk.Transform.translation(*self.forcepos[i]).rotated(self.forceangle[i] -pi/2)
		)
		
		self.tex = pg.image.load(os.path.join('res', 'rocket2.png')).convert_alpha()
	def draw(self, cam):
		
		# texture support
		if cam.offscreen is not None:
			
			# draw outline
			self.set_color('red')
			Rocket.draw(self, cam)
			
			# draw mask
			self.hull.color = 'white'
			cam.offscreen.fill((0,0,0, 0))
			cam.flip_screen()
			self.hull.draw(cam)
			cam.flip_screen()
			
			# blit texture
			sz = cam.r*1.5
			surf = pg.transform.scale(self.tex, (sz,sz))
			surf = pg.transform.rotate(surf, self.get_angle()*180/pi)
			r = surf.get_rect()
			
			cam.offscreen.blit(surf, cam.conv_coord(self.get_pos()+Vec2d(-r.w/2,r.h/2)/cam.r), None, pg.BLEND_RGBA_MULT)
			cam.screen.blit(cam.offscreen, (0,0))
			
			# draw red thrusters
			for prop in self.props:
				prop.color = 'red'
				prop.draw(cam)
		
		# no texture
		else:
			self.set_color('blue')
			Rocket.draw(self, cam)
class Rocket_basic(Rocket):
	def __init__(self, w,h, *args, **kwargs):
		Rocket.__init__(self, *args, **kwargs)
		
		self.hull = RectShape(self.space,self.body, w,h, 10)
		self.props = [RectShape(self.space,self.body, h/2,w/2, 1), RectShape(self.space,self.body, h/2,w/2, 1)]
		
		self.forcepos = [(-3/4*w, -h/2),(3/4*w, -h/2)]
		self.bottom = Vec2d(0, -h/2)
		
		for i in range(2) : self.props[i].shape.unsafe_set_vertices(
			self.props[i].shape.get_vertices(),
			pymunk.Transform.translation(*self.forcepos[i]).rotated(self.forceangle[i])
		)

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
		pg.draw.rect(cam.screen, "black", [int(v) for v in [x-wbg/2, y-hbg/2, wbg, hbg]]) # bg
		# modify full height depending on level
		htmp  = self.l * h / self.maxl
		pg.draw.rect(cam.screen, "cyan", [int(v) for v in [x-w/2, y+h/2-htmp, w, htmp]]) # color bar

def show_menu(menu, elements):
	pass
def hide_menu(menu, elements):
	pass
def events_menu(elements, event):
	res = Obj(rind=None,angle=None)
	
	if event.type == pgui.UI_TEXT_ENTRY_FINISHED:
		if   event.ui_element == elements.in_rocket : res.rind = int(event.text)
		elif event.ui_element == elements.in_angle  : res.angle = float(event.text)
	
	return res

# Fuzzy controllers
class Controller_target : pass
class Controller_platform : pass
class Controller_target_alix1(Controller_target):
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
		fy = self.sys2.compute({'erry': err.y*3, 'vely': vel.y/3}, 'fy')
		
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
		
		thrust = max(0, f1.dot(f2)) * 250 # more thrust if rocket is facing target
		
		f = Vec2d(1,1)*thrust + Vec2d(r1['fl'], r1['fr'])*200
		return (f.x,f.y)
class Controller_platform_alix1(Controller_platform):
	def __init__(self):
		self.c = Controller_target_alix1()
	def action(self, rpos,vel,rangle,rvel, pos,w):
		margin = 0.4
		err = rpos - pos
		
		if abs(err.x) < 1e-1 and err.y > 0 : return (0,0) # do nothing
		elif err.y > margin/2 : target = pos + Vec2d(0, margin)
		else :
			if err.x > margin/2   : target = pos + Vec2d(w/2 + margin, margin)
			elif err.x < margin/2 : target = pos + Vec2d(-w/2 - margin, margin)
			else:
				if err.x > 0   : target = pos + Vec2d(w/2 + margin, -margin)
				elif err.x < 0 : target = pos + Vec2d(-w/2 - margin, -margin)
		
		return self.c.action(rpos-target,vel,rangle,rvel)

# graphics + physics loop (interactive)
def main_manual():
	global pxPerM
	
	# Graphics init -------------------------------------------------------------------------------------------
	pg.init()
	# main window
	ws = (1000, 600) # screen dimensions in pixels
	pxPerM = 100 # px/m
	screen = pg.display.set_mode(ws)
	offscreen = pg.Surface(ws, pg.SRCALPHA) # for texture rendering
	pg.display.set_caption("Rocket control")
	clock = pg.time.Clock()
	keysdown = {}
	fps = 60
	maincam = Camera(Vec2d(0,0), screen, pxPerM, offscreen)
	# minimap
	mms = (ws[0]/5, ws[1]/5)
	minimap = pg.Surface(mms)
	mmcam = Camera(Vec2d(0,0), minimap, pxPerM/40) # minimap camera
	# -----------------------------------------------------------------------------------------------------------
	
	# Physics init ----------------------------------------------------------------------------------------------
	space = pymunk.Space()
	space.gravity = (0.0, -9.81)
	
	controllers_alix = {
		Target: Controller_target_alix1(),
		Platform: Controller_platform_alix1()
	}
	rockets = [
		Rocket_fancy(space, pi/8).set_pos(Vec2d(2,5)).set_controllers(controllers_alix), 
		Rocket_basic(0.5,1, space, 0).set_pos(Vec2d(0,5)).set_controllers(controllers_alix)
	]
	rind = 0 # focused rocket index
	# disable inter-rockets collisions
	for rocket in rockets:
		rocket.hull.shape.filter = pymunk.ShapeFilter(group=1)
		for prop in rocket.props : prop.shape.filter = pymunk.ShapeFilter(group=1)
	
	waypoints = [(2,0),(8,8),(-5, -10)]
	goals = [
		Platform(space, Vec2d(-8, -5), 3, 4),
		*[Target(Vec2d(*p), 1, 4) for p in waypoints],
	]
	for rocket in rockets : rocket.set_goals(goals)
	
	cage_w = 40 ; cage_h = 30
	cage_pnts = [(-cage_w/2,cage_h/2),(-cage_w/2,-cage_h/2),(cage_w/2,-cage_h/2),(cage_w/2,cage_h/2),(-cage_w/2,cage_h/2)]
	grounds = [
		Ground(space, [Vec2d(*p) for p in cage_pnts]), # cage
	]
	
	physObjects = [*grounds, *goals, *rockets]
	# -----------------------------------------------------------------------------------------------------------
	
	# GUI init --------------------------------------------------------------------------------------------------
	# lateral bars
	thrustbars = [
		LevelBar(5/100,50/100, 5/100,50/100, 300), # left
		LevelBar(5/100,50/100, 95/100,50/100, 300) # right
	]
	guiObjects = [*thrustbars]
	
	# menu
	mW = 0.5 * ws[0] ; mH = 0.9 * ws[1]
	menu = pgui.UIManager(ws)
	rocket_label = pgui.elements.UILabel( relative_rect=pg.Rect((ws[0]/2, 20), (100, 50)), text="Fusée sélectionnée" )
	in_rocket = pgui.elements.UITextEntryLine( relative_rect=pg.Rect((20, 20), (100, 50)), initial_text=str(rind), anchors={'left_target':rocket_label} )
	angle_label = pgui.elements.UILabel( relative_rect=pg.Rect((ws[0]/2, 20), (150, 50)), text="Angle des propulseurs", anchors={'top_target':rocket_label} )
	in_angle = pgui.elements.UITextEntryLine( relative_rect=pg.Rect((20, 20), (100, 50)), initial_text=str(rind), anchors={'left_target':angle_label,'top_target':in_rocket} )
	m_elements = Obj(
		in_rocket = in_rocket,
		in_angle = in_angle,
	)
	# ----------------------------------------------------------------------------------------------------------
	
	while True:
		dt = clock.tick(fps)
		
		# physics
		rocket.apply_damping(2, 2)
		space.step(1/fps)
		for rocket in rockets : rocket.refresh_applied_f()
		
		rpos = rockets[rind].get_pos()
		
		# user events
		for event in pg.event.get():
			if event.type == pg.QUIT : sys.exit(0)
			
			# get key events (no repeats)
			elif event.type == pg.KEYDOWN:
				if   event.key == pg.K_r: # reset all rockets
					for rocket in rockets : rocket.reset()
				elif event.key == pg.K_a : rockets[rind].auto = not rockets[rind].auto # toggle auto control
				elif event.key == pg.K_i : maincam.r *= 120/100 # zoom in
				elif event.key == pg.K_o : maincam.r *= 80/100 # zoom out
			
			events_menu(m_elements, event)
			menu.process_events(event)
		# get key downs (repeats)
		keys = pg.key.get_pressed()
		if keys[pg.K_ESCAPE] : sys.exit(0)
		if keys[pg.K_LEFT]   : rockets[rind].apply_force(100,0)
		if keys[pg.K_RIGHT]  : rockets[rind].apply_force(0,100)
		
		# controller
		for rocket in rockets:
			rocket.update_goal(time())
			rocket.apply_control()
		
		# main screen
		screen.fill((255,255,255))
		maincam.pos = rpos
		for obj in physObjects+guiObjects : obj.draw(maincam)
		
		# level bars
		applied_f = rockets[rind].applied_f
		for i in [0,1] : thrustbars[i].l = applied_f[i].length
		
		# minimap
		minimap.fill((128,128,128))
		for obj in physObjects : obj.drawmini(mmcam)
		screen.blit(minimap, (10,ws[1]-(mms[1]+10))) 
		
		# menu
		menu.update(dt)
		menu.draw_ui(screen)
		
		pg.display.flip()
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


# TODO : 
# - integrer les codes (controlleur de audric)
# - finir menu
