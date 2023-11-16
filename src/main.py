# Example file showing a circle moving on screen
import numpy as np
import pygame

import fuzzy as fz

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
        'fz_sys_gauche' : fz_sys_xtheta_gauche,
        'fz_sys_droit' : fz_sys_xtheta_droit
    }

# pygame setupfg
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

systemes = init_fuzzy()

player_pos = pygame.Vector2(screen.get_width() / 2, screen.get_height() / 2)
player_angle = np.pi/2

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")

    pygame.draw.circle(screen, "white", player_pos, 40)
    pygame.draw.circle(screen, "WHITE",pygame.Vector2(player_pos.x -30, player_pos.y +30),30)
    keys = pygame.key.get_pressed()
    if keys[pygame.K_f]:
        player_pos.y += np.sin(player_angle + 3*np.pi/4)* 200 * dt
        player_pos.x += np.cos(player_angle+ 3*np.pi/4)* 200 * dt
        player_angle += np.pi/16 * dt
        pygame.draw.circle(screen, "red", pygame.Vector2(player_pos.x -30, player_pos.y +30), 10)
    if keys[pygame.K_g]:
        player_pos.y += np.sin(player_angle + 5*np.pi/4)* 200 * dt
        player_pos.x += np.cos(player_angle+ 5*np.pi/4)* 200 * dt
        player_angle -= np.pi/16 * dt
        pygame.draw.circle(screen, "red",pygame.Vector2(player_pos.x +30, player_pos.y +30) , 10)
        
    player_pos.y += 100 * dt

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()