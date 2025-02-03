from phonemizer.backend import EspeakBackend
import librosa
from gtts import gTTS
from moviepy.editor import ImageClip, concatenate_videoclips, AudioFileClip, CompositeVideoClip
import os
from pydub import AudioSegment, silence
import numpy as np
from main import morph_transition
from utils.transformation import read_points_from_file


# Configuración inicial
respuesta = "si super que vamos a hacer hoy?"  # Texto de entrada
audio_path = "audio.mp3"  # Nombre del archivo de audio
trimmed_audio_path = "trimmed_audio.mp3"  # Archivo de audio sin silencios
image_folder = "caritas/partes separadas sin fondo/bocas"  # Carpeta de imágenes
points_folder = "caritas/partes separadas sin fondo/bocas_points"  # Carpeta de puntos
output_video = "output_video.mp4"  # Video con imágenes sincronizadas
output_video_with_audio = "output_video_with_audio.mp4"  # Video final con imágenes y audio
eyes_folder = "caritas/partes separadas sin fondo/pares de ojos" 


phoneme_to_image = {
    "e": "6.png", "é": "6.png", "a": "6.png", "g": "6.png", "ɡ": "6.png", "x": "6.png", 
    "i": "1.png", "ɪ": "1.png", "s": "1.png", "θ": "1.png", "ʝ": "1.png", "j": "1.png", 
    "o": "3.png", "ó": "3.png", 
    "u": "2.png", 
    "b": "0.png", "m": "0.png", "p": "0.png", 
    "d": "5.png", "ð": "5.png", "k": "5.png", "l": "5.png", "n": "5.png", "ŋ": "5.png", "r": "5.png", "ɾ": "5.png", "t": "5.png", "β": "5.png", 
    "f": "4.png"
}


phoneme_to_points = {k: v.replace(".png", "_points.txt") for k, v in phoneme_to_image.items()}

shut_mouth = {"default": "13.png" }

shut_mouth_points = {k: v.replace(".png", "_points.txt") for k, v in shut_mouth.items()}

shut_mouth = {"default": "13.png", "tongue out mad": "10.png",
              "tongue our happy": "11.png", "kiss": "12.png",
              "pout": "14.png", "meh": "15.png",
              "cat": "16.png" }

shut_mouth_points = {k: v.replace(".png", "_points.txt") for k, v in shut_mouth.items()}

open_eyes = {"meh": "1.png", "mad": "2.png",
             "closed": "3.png", "sparkle closed": "4.png",
             "emotional": "5.png", "excited": "6.png",
             "open": "7.png", "dead": "8.png", "cool": "10.png"}


# Crear un bucle de ojos parpadeando
def create_eyes_blink_loop(eyes_folder, open_eyes, total_duration, fps=24):
    eye_states = ["open", "closed"]  # Ejemplo: abierto -> cerrado -> abierto
    state_durations = [0.6, 0.1]  # Duraciones en segundos para cada estado
    total_states_duration = sum(state_durations)
    loops_needed = int(np.ceil(total_duration / total_states_duration))
    
    # Crear la secuencia de clips de ojos
    eye_clips = []
    for _ in range(loops_needed):
        for state, duration in zip(eye_states, state_durations):
            eye_image = os.path.join(eyes_folder, open_eyes[state])
            if os.path.exists(eye_image):
                clip = ImageClip(eye_image, duration=duration)
                eye_clips.append(clip)
    
    # Combinar los clips y recortar al total_duration
    blinking_eyes = concatenate_videoclips(eye_clips, method="compose").subclip(0, total_duration)
    return blinking_eyes


# Convertir texto a fonemas
phonemizer = EspeakBackend(language='es')
phoneme_list = phonemizer.phonemize(respuesta.split(" "))
print("Fonemas:", " ".join(phoneme_list))  # Salida: "si supéɾ ke βamos a aθeɾ oj"

# Generar audio con gTTS
tts = gTTS(respuesta, lang='es')
tts.save(audio_path)

# Cargar el audio y eliminar silencios
audio = AudioSegment.from_file(audio_path)
trimmed_audio = silence.detect_nonsilent(audio, min_silence_len=200, silence_thresh=-40)

# Obtener la región activa (sin silencios)
if trimmed_audio:
    start_trim, end_trim = trimmed_audio[0][0], trimmed_audio[-1][1]
    audio_trimmed = audio[start_trim:end_trim]
    audio_trimmed.export(trimmed_audio_path, format="mp3")
    total_duration = len(audio_trimmed) / 1000.0  # Duración del audio sin silencios en segundos
else:
    raise ValueError("No se detectó contenido hablado en el audio.")

# Cargar el audio ajustado con librosa
y, sr = librosa.load(trimmed_audio_path)

# Calcular la energía en cada frame del audio
energy = librosa.feature.rms(y=y)

# Determinar el umbral de energía (por ejemplo, el 50% del promedio)
threshold = np.mean(energy) * 0.5

# Encontrar los frames donde la energía es mayor que el umbral
frames = np.where(energy[0] > threshold)[0]

# Convertir los frames a tiempos en segundos
times = librosa.frames_to_time(frames, sr=sr)

# Calcular la duración promedio de cada fonema
unit_duration = total_duration / len(phoneme_list)

# Asignar tiempos a cada fonema
fonema_durations = []
current_time = 0
for phoneme in phoneme_list:
    start_time = current_time
    end_time = current_time + unit_duration
    fonema_durations.append((phoneme, start_time, end_time))
    current_time = end_time

# Mostrar los fonemas con sus tiempos
print("\nFonemas y sus tiempos:")
for phoneme, start_time, end_time in fonema_durations:
    print(f"Fonema: {phoneme}, Inicio: {start_time:.2f}s, Fin: {end_time:.2f}s")

sequence = []
for phoneme, start_time, end_time in fonema_durations:
    duration_per_char = (end_time - start_time)/len(phoneme)
    for char in phoneme:
        if char in phoneme_to_image:
            image_file = os.path.join(image_folder, phoneme_to_image[char])
            points_file = os.path.join(points_folder, phoneme_to_points[char])
            
            if os.path.exists(image_file) and os.path.exists(points_file):
                sequence.append((image_file,points_file,duration_per_char))

duracion_of_transition = 0.7 
num_images = 2

if sequence:
    clips = []
    for i, (image, points, duration) in zip(range(len(sequence)), sequence):
        clip = ImageClip(image, duration=duration - duracion_of_transition)
        clips.append(clip)
        if 0 <= i < len(sequence) - 1:
            frames = morph_transition(image ,sequence[i+1][0] ,points ,sequence[i+1][1], num_images = num_images)
            for frame in frames:
                    # Convertir PIL Image a numpy array
                    frame_np = np.array(frame)
                    
                    # Crear un ImageClip a partir del array numpy
                    clip = ImageClip(frame_np, duration = duracion_of_transition/num_images)  # Duración de cada imagen
                    
                    # Agregar el clip a la lista
                    clips.append(clip)

    # Combinar los clips de la boca
    mouth_video = concatenate_videoclips(clips, method="compose")

    # Generar el bucle de ojos
    blinking_eyes = create_eyes_blink_loop(eyes_folder, open_eyes, mouth_video.duration)

    # Superponer los ojos sobre el video de la boca
    final_video = CompositeVideoClip([mouth_video.set_position(("center", "bottom")),
                                      blinking_eyes.set_position(("center", "top"))])

    # Añadir el audio
    audio_clip = AudioFileClip(audio_path)
    final_video = final_video.set_audio(audio_clip)

    # Guardar el video final
    final_video.write_videofile(output_video_with_audio, codec="libx264", audio_codec="aac", fps=24)
else:
    print("No se generó ninguna secuencia de imágenes.")

print(f"Video con audio y ojos parpadeando: {output_video_with_audio}")
                