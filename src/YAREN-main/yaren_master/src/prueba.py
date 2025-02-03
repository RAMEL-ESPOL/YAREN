#!/usr/bin/env python3
import pygame
import os
import time
import numpy as np
from threading import Thread
from queue import Queue
from video_generation import total_video_generation

from threading import Thread
import queue

def reproducir_video_con_hilos(video_clip, window_width=800, window_height=600):
    """
    Reproduce un video con audio utilizando pygame y hilos para optimizar el rendimiento.
    """
    pygame.init()
    window = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("Reproduciendo Video")

    # Extraer el audio como un archivo temporal
    audio_temp_path = "temp_audio.mp3"
    if video_clip.audio:
        video_clip.audio.write_audiofile(audio_temp_path, fps=44100)

    # Configurar pygame para el audio
    if os.path.exists(audio_temp_path):
        pygame.mixer.init()
        pygame.mixer.music.load(audio_temp_path)

    # Cola para frames procesados
    frame_queue = queue.Queue(maxsize=24)  # Mantén un buffer de 24 frames (1 segundo de video)

    # Función para procesar frames
    def procesar_frames():
        for frame in video_clip.iter_frames(fps=24, with_times=False):
            frame_surface = pygame.surfarray.make_surface(np.swapaxes(frame, 0, 1))
            frame_surface = pygame.transform.scale(frame_surface, (window_width, window_height))
            frame_queue.put(frame_surface)
        frame_queue.put(None)  # Fin de los frames

    # Hilo para procesamiento de frames
    frame_thread = Thread(target=procesar_frames)
    frame_thread.start()

    # Iniciar audio
    pygame.mixer.music.play()

    # Configurar el reloj para FPS
    timer = pygame.time.Clock()

    # Mostrar frames
    while True:
        frame_surface = frame_queue.get()
        if frame_surface is None:  # Fin del video
            break

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.mixer.music.stop()
                pygame.quit()
                return

        window.blit(frame_surface, (0, 0))
        pygame.display.update()
        timer.tick(24)

    # Finalizar
    pygame.mixer.music.stop()
    if os.path.exists(audio_temp_path):
        os.remove(audio_temp_path)


def reproducir_video(video_clip, window_width=800, window_height=600):
    """
    Reproduce un video con audio utilizando pygame, procesando imágenes de manera eficiente.
    """
    # Inicializa pygame
    pygame.init()
    window = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("Reproduciendo Video")

    # Extraer el audio a un archivo temporal
    audio_temp_path = "temp_audio.mp3"
    if video_clip.audio:
        video_clip.audio.write_audiofile(audio_temp_path, fps=44100)

    # Cargar y preparar audio
    if os.path.exists(audio_temp_path):
        pygame.mixer.init()
        pygame.mixer.music.load(audio_temp_path)

    # Iniciar reproducción de audio
    pygame.mixer.music.play()

    # Configurar el reloj para controlar FPS
    timer = pygame.time.Clock()

    # Reproducir los frames directamente
    for frame in video_clip.iter_frames(fps=24, with_times=False):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.mixer.music.stop()
                pygame.quit()
                return

        # Convertir el frame a una superficie de pygame
        frame_surface = pygame.surfarray.make_surface(np.swapaxes(frame, 0, 1))
        frame_surface = pygame.transform.scale(frame_surface, (window_width, window_height))
        window.blit(frame_surface, (0, 0))
        pygame.display.update()

        # Sincronizar con 24 FPS
        timer.tick(24)

    # Finalizar reproducción
    pygame.mixer.music.stop()
    if os.path.exists(audio_temp_path):
        os.remove(audio_temp_path)


if __name__ == "__main__":
    
    # Inicia el temporizador
    start_time = time.time()

    # Configuración inicial
    respuesta = "si estoy muy bien gracias por preguntar, dime en que puedo ayudarte hoy!"
    start_gen_time = time.time()
    video_clip = total_video_generation(respuesta)

    print(f"Tiempo de generación del video: {time.time() - start_gen_time:.2f} segundos")

    reproducir_video_con_hilos(video_clip, window_width=800, window_height=480)

    # Termina el temporizador y calcula la duración
    elapsed_time = time.time() - start_time
    print(f"Tiempo de ejecución: {elapsed_time:.2f} segundos")