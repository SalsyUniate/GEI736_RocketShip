# Example file showing a circle moving on screen
import numpy as np
import pygame

# pygame setupfg
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

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