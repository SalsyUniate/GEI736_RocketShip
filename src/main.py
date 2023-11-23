import pymunk
from pymunk.vec2d import Vec2d
import pygame
import numpy as np

import sys
import math
pi = math.pi
from copy import deepcopy as dcp
import random

import fuzzy as fz
import simulation as sim

pxPerM = 100 # px/m

def init_fuzzy():
    x_rules_gauche = [
        [3, 3, 2, 2, 1],
        [2, 2, 1, 1, 1],
        [2, 1, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ]

    x_rules_droit = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 1, 2],
        [1, 1, 1, 2, 2],
        [1, 2, 2, 3, 3]
    ]

    theta_rules_gauche = [
		[0, 0, 0, 0, 0],
		[0, 0, 0, 0, 1],
		[0, 0, 0, 1, 2],
		[0, 0, 1, 2, 2],
		[0, 1, 2, 2, 3]
	]

    theta_rules_droit = [
		[3, 2, 2, 1, 0],
		[2, 2, 1, 0, 0],
		[2, 1, 0, 0, 0],
		[1, 0, 0, 0, 0],
		[0, 0, 0, 0, 0]
	]

    y_rules_gauche = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1],
        [0, 0, 0, 1, 2],
        [0, 1, 2, 2, 3],
        [1, 2, 3, 3, 3]
    ]

    y_rules_droit = y_rules_gauche

    rule_sets = {
        
        'err_x': fz.SetTrig.autoN(5, -500, 500, True, 'err_x'),
		'derr_x': fz.SetTrig.autoN(5, -5, 5, True, 'derr_x'),

        'err_theta': fz.SetTrig.autoN(5, -np.pi/2, np.pi/2, True, 'err_theta'),
		'derr_theta': fz.SetTrig.autoN(5, -np.pi, np.pi, True, 'derr_theta'),

        'err_y': fz.SetTrig.autoN(5, -500, 500, True, 'err_y'),
		'derr_y': fz.SetTrig.autoN(5, -5, 5, True, 'err_y'),

        'force_gauche': fz.SetTrig.autoN(4, 0, 300, False, 'force_gauche'),
		'force_droite': fz.SetTrig.autoN(4, 0, 300, False, 'force_droite')
    }

    #X
    f_rules_x_gauche = [ fz.Rule(fz.AND(rule_sets['err_x'][err_i],rule_sets['derr_x'][derr_i]), rule_sets['force_gauche'][x_rules_gauche[err_i][derr_i]]) for err_i in range(5) for derr_i in range(5) ]
    fz_sys_x_gauche = fz.System(f_rules_x_gauche)

    f_rules_x_droit = [ fz.Rule(fz.AND(rule_sets['err_x'][err_i],rule_sets['derr_x'][derr_i]), rule_sets['force_droite'][x_rules_droit[err_i][derr_i]]) for err_i in range(5) for derr_i in range(5) ]
    fz_sys_x_droit = fz.System(f_rules_x_droit)

    #Theta
    f_rules_theta_gauche = [ fz.Rule(fz.AND(rule_sets['err_theta'][err_i],rule_sets['derr_theta'][derr_i]), rule_sets['force_gauche'][theta_rules_gauche[err_i][derr_i]]) for err_i in range(5) for derr_i in range(5) ]
    fz_sys_theta_gauche = fz.System(f_rules_theta_gauche)

    f_rules_theta_droit = [ fz.Rule(fz.AND(rule_sets['err_theta'][err_i],rule_sets['derr_theta'][derr_i]), rule_sets['force_droite'][theta_rules_droit[err_i][derr_i]]) for err_i in range(5) for derr_i in range(5) ]
    fz_sys_theta_droit = fz.System(f_rules_theta_droit)

    #Y
    f_rules_y_gauche = [ fz.Rule(fz.AND(rule_sets['err_y'][err_i],rule_sets['derr_y'][derr_i]), rule_sets['force_gauche'][y_rules_gauche[err_i][derr_i]]) for err_i in range(5) for derr_i in range(5) ]
    fz_sys_y_gauche = fz.System(f_rules_y_gauche)

    f_rules_y_droit = [ fz.Rule(fz.AND(rule_sets['err_y'][err_i],rule_sets['derr_y'][derr_i]), rule_sets['force_droite'][y_rules_droit[err_i][derr_i]]) for err_i in range(5) for derr_i in range(5) ]
    fz_sys_y_droit = fz.System(f_rules_y_droit)



    x_theta_rules_gauche = [
        [0, 0, 1, 1],
        [0, 1, 1, 2],
        [1, 1, 2, 3],
        [1, 2, 3, 3]
    ]

    x_theta_rules_droit = [
        [3, 3, 2, 2],
        [3, 2, 2, 1],
        [2, 2, 1, 0],
        [2, 1, 0, 0]
    ]

    rule_sets_x_theta = {
        'x_gauche': fz.SetTrig.autoN(4, 0, 3, True, 'x_gauche'),
		'theta_gauche': fz.SetTrig.autoN(4, 0, 3, True, 'theta_gauche'),
        
        'x_droit': fz.SetTrig.autoN(4, 0, 3, True, 'x_droit'),
		'theta_droit': fz.SetTrig.autoN(4, 0, 3, True, 'theta_droit'),

        
        'force_gauche': fz.SetTrig.autoN(4, 0, 300, False, 'force_gauche'),
		'force_droite': fz.SetTrig.autoN(4, 0, 300, False, 'force_droite')
    }

    
    f_rules_xtheta_gauche = [ fz.Rule(fz.AND(rule_sets_x_theta['x_gauche'][x_i],rule_sets_x_theta['theta_gauche'][theta_i]), rule_sets_x_theta['force_gauche'][x_theta_rules_gauche[x_i][theta_i]]) for x_i in range(4) for theta_i in range(4) ]
    fz_sys_xtheta_gauche = fz.System(f_rules_xtheta_gauche)

    f_rules_xtheta_droit = [ fz.Rule(fz.AND(rule_sets_x_theta['x_gauche'][x_i],rule_sets_x_theta['theta_gauche'][theta_i]), rule_sets_x_theta['force_droite'][x_theta_rules_droit[x_i][theta_i]]) for x_i in range(4) for theta_i in range(4) ]
    fz_sys_xtheta_droit = fz.System(f_rules_xtheta_droit)


    xtheta_y_rules_gauche = [
        [0, 1, 2, 3],
        [1, 2, 3, 4],
        [2, 3, 4, 5],
        [3, 4, 5, 6]
    ]

    xtheta_y_rules_droit = x_theta_rules_gauche

    rule_sets_xtheta_y = {
        'xtheta_gauche': fz.SetTrig.autoN(4, 0, 3, True, 'xtheta_gauche'),
		'y_gauche': fz.SetTrig.autoN(4, 0, 3, True, 'y_gauche'),
        
        'xtheta_droit': fz.SetTrig.autoN(4, 0, 3, True, 'xtheta_droit'),
		'y_droit': fz.SetTrig.autoN(4, 0, 3, True, 'y_droit'),

        
        'force_gauche': fz.SetTrig.autoN(7, 0, 300, False, 'force_gauche'),
		'force_droite': fz.SetTrig.autoN(7, 0, 300, False, 'force_droite')
    }

    f_rules_gauche = [ fz.Rule(fz.AND(rule_sets_xtheta_y['xtheta_gauche'][xtheta_i],rule_sets_xtheta_y['y_gauche'][y_i]), rule_sets_xtheta_y['force_gauche'][xtheta_y_rules_gauche[xtheta_i][y_i]]) for xtheta_i in range(4) for y_i in range(4) ]
    fz_sys_gauche = fz.System(f_rules_gauche)

    f_rules_droit = [ fz.Rule(fz.AND(rule_sets_xtheta_y['xtheta_droit'][xtheta_i],rule_sets_xtheta_y['y_droit'][y_i]), rule_sets_xtheta_y['force_droite'][xtheta_y_rules_droit[xtheta_i][y_i]]) for xtheta_i in range(4) for y_i in range(4) ]
    fz_sys_droit = fz.System(f_rules_droit)

    return {
        'fz_sys_x_gauche' : fz_sys_x_gauche,
        'fz_sys_x_droit' : fz_sys_x_droit,
        'fz_sys_theta_gauche' : fz_sys_theta_gauche,        
        'fz_sys_theta_droit' : fz_sys_theta_droit,
        'fz_sys_y_gauche' : fz_sys_y_gauche,        
        'fz_sys_y_droit' : fz_sys_y_droit,
        'fz_sys_xtheta_gauche' : fz_sys_xtheta_gauche,
        'fz_sys_xtheta_droit' : fz_sys_xtheta_droit,
        'fz_sys_gauche' : fz_sys_gauche,
        'fz_sys_droit' : fz_sys_droit
    }

def main():
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)
	
	rocket = sim.Rocket(space, 0.5, 1).set_pos(Vec2d(2,0.1))
	target = sim.Target(Vec2d(0,0))
	objects = [
		sim.Ground(space, [(-1,8),(8,8)]),
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