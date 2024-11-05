
import pygame, os
   
background_colour = (0xFF, 0xFF, 0xFF) 
  
# Define the dimensions of 
# screen object(width,height) 
screen = pygame.display.set_mode((640, 480)) 

image = pygame.image.load(os.path.join('.', 'foo.png'))
imagerect = image.get_rect()
  
# Set the caption of the screen 
pygame.display.set_caption('Whiteboard') 
  
# Fill the background colour to the screen 
screen.fill(background_colour) 
  
# Update the display using flip 
pygame.display.flip() 
  
# Variable to keep our game loop running 
running = True
  
# game loop 
while running: 
    
# for loop through the event queue   
    for event in pygame.event.get(): 
      
        screen.fill((0xff, 0xff, 0xff))
        screen.blit(image, imagerect)
        pygame.display.flip()

        # Check for QUIT event       
        if event.type == pygame.QUIT: 
            running = False
