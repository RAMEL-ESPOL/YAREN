import pygame
from itertools import cycle
import ctypes

carpetaImgs = "rostro/"
IMAGEN_SIN_PALABRAS = carpetaImgs + "Initial.png"
IMAGEN_FELIZ = carpetaImgs + "Rutina 2.png"
IMAGEN_MUY_FELIZ =  carpetaImgs + "Rutina 3.png"
IMAGEN_TRISTE = carpetaImgs + "Rutina 4.png"
IMAGEN_CANSADO = carpetaImgs + "Rutina 5.png"
IMAGEN_ENOJADO = carpetaImgs + "Rutina 6.png"
IMAGEN_ASOMBRADO = carpetaImgs + "Rutina 7.png"

# Lista de rutas de imágenes
SECUENCIA_IMAGENES = [IMAGEN_SIN_PALABRAS, IMAGEN_ENOJADO, IMAGEN_ASOMBRADO]

# Posición de inicio de la ventana
window_position = [0, 0]

# Pasos para movimiento horizontal y vertical
pasosHorizontal = 5
pasosVertical = 5

def moverVentana():
    keys = pygame.key.get_pressed()
    
    if keys[pygame.K_LEFT]:
        window_position[0] -= pasosHorizontal
    if keys[pygame.K_RIGHT]:
        window_position[0] += pasosHorizontal
    if keys[pygame.K_UP]:
        window_position[1] -= pasosVertical
    if keys[pygame.K_DOWN]:
        window_position[1] += pasosVertical

def main():
    # Inicializa Pygame
    pygame.init()

    # Dimensiones de la ventana
    window_width = 800
    window_height = 480

    # Crea la ventana sin bordes
    window = pygame.display.set_mode((window_width, window_height), pygame.NOFRAME)

    timer = pygame.time.Clock()
    expressions = cycle(SECUENCIA_IMAGENES)
    current = next(expressions)
    pygame.time.set_timer(pygame.USEREVENT, 1000)
    
    # Bucle principal
    running = True
    while running:
        
        for e in pygame.event.get():
            if e.type == pygame.QUIT: 
                running = False
            if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                running = False
            if e.type == pygame.USEREVENT:
                current = next(expressions)
        
        # Mueve las ventanas por medio de teclas
        moverVentana()

        # Mueve la ventana
        ctypes.windll.user32.MoveWindow(pygame.display.get_wm_info()['window'], window_position[0], window_position[1], window_width, window_height, True)

        imagen = pygame.image.load(current)
        window.blit(imagen, (0, 0))
        
        # Muestra la ventana
        pygame.display.flip()
        
        timer.tick(60)
        pygame.display.update()

    # Finaliza Pygame
    pygame.quit()

if __name__ == "__main__":
    main()