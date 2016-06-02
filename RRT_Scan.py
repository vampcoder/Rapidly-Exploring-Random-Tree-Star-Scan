import pygame,time

pygame.init()


img = pygame.image.load('Images/obstacle1.png')
screen = pygame.display.set_mode(img.get_rect().size)
screen.blit(img, (0, 0))
pygame.display.flip()

time.sleep(300)

