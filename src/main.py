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
    THROTLE_MAX = 150

    x_rules_gauche = [
        [3, 3, 2, 1, 1],
        [2, 1, 1, 1, 1],
        [1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ]

    x_rules_droit = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1],
        [1, 1, 1, 2, 2],
        [1, 1, 2, 3, 3]
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
        [2, 2, 2, 2, 3],
        [2, 2, 2, 2, 3],
        [2, 2, 2, 2, 3],
        [2, 2, 2, 2, 3],
        [2, 3, 3, 3, 3]
    ]

    y_rules_droit = y_rules_gauche

    rule_sets = {
        
        'err_x': fz.SetTrig.autoN(5, -7.5, 7.5, True, 'err_x'),
		'derr_x': fz.SetTrig.autoN(5, -15, 15, True, 'derr_x'),

        'err_theta': fz.SetTrig.autoN(5, -np.pi/2, np.pi/2, True, 'err_theta'),
		'derr_theta': fz.SetTrig.autoN(5, -np.pi, np.pi, True, 'derr_theta'),

        'err_y': fz.SetTrig.autoN(5, -1, 1, True, 'err_y'),
		'derr_y': fz.SetTrig.autoN(5, -10, 10, True, 'derr_y'),

        'force_gauche': fz.SetTrig.autoN(4, 0, THROTLE_MAX, False, 'force_gauche'),
		'force_droite': fz.SetTrig.autoN(4, 0, THROTLE_MAX, False, 'force_droite')
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
        [0, 1, 2, 3],
        [0, 1, 2, 3],
        [1, 2, 2, 3],
        [2, 3, 3, 3]
    ]

    """
    x_theta_rules_droit = [
        [3, 3, 2, 2],
        [3, 2, 2, 1],
        [2, 2, 1, 0],
        [2, 1, 0, 0]
    ]
    """
    x_theta_rules_droit = x_theta_rules_gauche

    rule_sets_x_theta = {
        'x_gauche': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'x_gauche'),
		'theta_gauche': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'theta_gauche'),
        
        'x_droit': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'x_droit'),
		'theta_droit': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'theta_droit'),

        
        'force_gauche': fz.SetTrig.autoN(4, 0, THROTLE_MAX, False, 'force_gauche'),
		'force_droite': fz.SetTrig.autoN(4, 0, THROTLE_MAX, False, 'force_droite')
    }

    
    f_rules_xtheta_gauche = [ fz.Rule(fz.AND(rule_sets_x_theta['x_gauche'][x_i],rule_sets_x_theta['theta_gauche'][theta_i]), rule_sets_x_theta['force_gauche'][x_theta_rules_gauche[x_i][theta_i]]) for x_i in range(4) for theta_i in range(4) ]
    fz_sys_xtheta_gauche = fz.System(f_rules_xtheta_gauche)

    f_rules_xtheta_droit = [ fz.Rule(fz.AND(rule_sets_x_theta['x_droit'][x_i],rule_sets_x_theta['theta_droit'][theta_i]), rule_sets_x_theta['force_droite'][x_theta_rules_droit[x_i][theta_i]]) for x_i in range(4) for theta_i in range(4) ]
    fz_sys_xtheta_droit = fz.System(f_rules_xtheta_droit)


    xtheta_y_rules_gauche = [
        [1, 2, 2, 3],
        [2, 2, 3, 4],
        [3, 4, 4, 5],
        [4, 4, 5, 6]
    ]

    xtheta_y_rules_droit = xtheta_y_rules_gauche

    rule_sets_xtheta_y = {
        'xtheta_gauche': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'xtheta_gauche'),
		'y_gauche': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'y_gauche'),
        
        'xtheta_droit': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'xtheta_droit'),
		'y_droit': fz.SetTrig.autoN(4, 0, THROTLE_MAX, True, 'y_droit'),

        
        'force_gauche': fz.SetTrig.autoN(7, 0, THROTLE_MAX, False, 'force_gauche'),
		'force_droite': fz.SetTrig.autoN(7, 0, THROTLE_MAX, False, 'force_droite')
    }

    f_rules_gauche = [ fz.Rule(fz.AND(rule_sets_xtheta_y['xtheta_gauche'][xtheta_i],rule_sets_xtheta_y['y_gauche'][y_i]), rule_sets_xtheta_y['force_gauche'][xtheta_y_rules_gauche[xtheta_i][y_i]]) for xtheta_i in range(4) for y_i in range(4) ]
    fz_sys_gauche = fz.System(f_rules_gauche)

    f_rules_droit = [ fz.Rule(fz.AND(rule_sets_xtheta_y['xtheta_droit'][xtheta_i],rule_sets_xtheta_y['y_droit'][y_i]), rule_sets_xtheta_y['force_droite'][xtheta_y_rules_droit[xtheta_i][y_i]]) for xtheta_i in range(4) for y_i in range(4) ]
    fz_sys_droit = fz.System(f_rules_droit)

    return {
        'x_gauche' : fz_sys_x_gauche,
        'x_droit' : fz_sys_x_droit,
        'theta_gauche' : fz_sys_theta_gauche,        
        'theta_droit' : fz_sys_theta_droit,
        'y_gauche' : fz_sys_y_gauche,        
        'y_droit' : fz_sys_y_droit,
        'xtheta_gauche' : fz_sys_xtheta_gauche,
        'xtheta_droit' : fz_sys_xtheta_droit,
        'gauche' : fz_sys_gauche,
        'droit' : fz_sys_droit
    }

def main():
	pygame.init()
	windowSize = (1300,700)
	simSize = (20,20)
	screen = pygame.display.set_mode(windowSize)
	pygame.display.set_caption("Rocket control")
	clock = pygame.time.Clock()
	fps = 60
	
	space = pymunk.Space()
	space.gravity = (0.0, 9.81)

	rocket = sim.Rocket(space, 0.5, 1).set_pos((15,15))
	target = sim.Target(Vec2d(10,10))
	objects = [
		sim.Ground(space, [(0,0),(0,simSize[1]), (simSize[0],simSize[1]), (simSize[0],0),(0,0)]),
		rocket,
		target
	]	

	leftSelected = True
	boosterStrength = [100,100]
	
	theta = 0
	dest_x = 10
	dest_y = 10
	dx = 0
	dy = 0
	dt = 0.016
	
	fz_sys = init_fuzzy()
	fuzzy = False
	
	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT : sys.exit(0)
			

		rocketpos = rocket.get_pos()
		old_theta = theta
		old_dx = dx
		old_dy = dy
		
		dx = -(dest_x - rocketpos.x)
		dy = -(dest_y - rocketpos.y)
		
		theta = rocket.get_angle() % (2*np.pi)
		if(theta > np.pi): theta = theta - 2*np.pi
		theta = -theta
		
		d_dx = (dx - old_dx) / (dt)
		d_dy = (dy - old_dy) / (dt)
		d_theta = (theta - old_theta) / (dt)
		
		x_gauche = fz_sys['x_gauche'].compute({'err_x': dx, 'derr_x': d_dx}, tag = 'force_gauche')
		x_droit = fz_sys['x_droit'].compute({'err_x': dx, 'derr_x': d_dx}, tag = 'force_droite')
		
		theta_gauche = fz_sys['theta_gauche'].compute({'err_theta': theta, 'derr_theta': d_theta}, tag = 'force_gauche')
		theta_droit = fz_sys['theta_droit'].compute({'err_theta': theta, 'derr_theta': d_theta}, tag = 'force_droite')
		
		y_gauche = fz_sys['y_gauche'].compute({'err_y': dy, 'derr_y': d_dy}, tag = 'force_gauche')
		y_droit = fz_sys['y_droit'].compute({'err_y': dy, 'derr_y': d_dy}, tag = 'force_droite')
		
		xtheta_gauche = fz_sys['xtheta_gauche'].compute({'x_gauche': x_gauche, 'theta_gauche': theta_gauche}, tag = 'force_gauche')
		xtheta_droit = fz_sys['xtheta_droit'].compute({'x_droit': x_droit, 'theta_droit': theta_droit}, tag = 'force_droite')
		
		force_gauche = fz_sys['gauche'].compute({'xtheta_gauche': xtheta_gauche, 'y_gauche': y_gauche}, tag = 'force_gauche')
		force_droite = fz_sys['droit'].compute({'xtheta_droit': xtheta_droit, 'y_droit': y_droit}, tag = 'force_droite')
		
		if(theta > np.pi / 2): force_droite = 0
		if(theta < -np.pi / 2): force_gauche = 0

		if fuzzy: rocket.apply_force(force_gauche, force_droite)
		print(f'x : {rocketpos.x}, y : {rocketpos.y}, theta : {theta}, force_gauche : {force_gauche}, force_droite : {force_droite}')
	
		keys = pygame.key.get_pressed()
		if keys[pygame.K_SPACE]: fuzzy = not(fuzzy)
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
	
		rocket.apply_damping(2,2)
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