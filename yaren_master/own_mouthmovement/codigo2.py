from gtts import gTTS
from moviepy.editor import ImageClip, concatenate_videoclips, AudioFileClip, CompositeVideoClip
import os
from pydub import AudioSegment, silence
import numpy as np
from main import morph_transition
from utils.transformation import read_points_from_file
import openai

# Configuración inicial
respuesta = "la vida es triste, y solitaria."  # Texto de entrada
audio_path = "audio.mp3"  # Nombre del archivo de audio
trimmed_audio_path = "trimmed_audio.mp3"  # Archivo de audio sin silencios
image_folder = "caritas/partes separadas sin fondo/bocas"  # Carpeta de imágenes
points_folder = "caritas/partes separadas sin fondo/bocas_points"  # Carpeta de puntos
output_video = "output_video.mp4"  # Video con imágenes sincronizadas
output_video_with_audio = "output_video_with_audio.mp4"  # Video final con imágenes y audio
eyes_folder = "caritas/partes separadas sin fondo/pares de ojos" 

# Diccionario para asociar letras con imágenes
phoneme_to_image = {
    'i': '1.png', 'u': '2.png', 'o': '3.png', 'j': '3.png',
    'f': '4.png', 'v': '4.png',
    'c': '5.png', 'd': '5.png', 'g': '5.png', 'k': '5.png',
    'n': '5.png', 's': '5.png', 't': '5.png', 'x': '5.png',
    'y': '1.png', 'z': '5.png', 'l': '5.png', 'r': '5.png',
    'a': '6.png', 'e': '6.png', 'q': '6.png',
    'b': '0.png', 'm': '0.png', 'p': '0.png',
    'h': '0.png'  # Asociar espacios con la imagen 13
}

# Diccionario para asociar letras con imágenes
phoneme_to_points = {k: v.replace(".png", "_points.txt") for k, v in phoneme_to_image.items()}

shut_mouth = {"default": "13.png", "tongue out mad": "10.png",
              "tongue our happy": "11.png", "kiss": "12.png",
              "pout": "14.png", "meh": "15.png",
              "cat": "16.png" }

shut_mouth_points = {k: v.replace(".png", "_points.txt") for k, v in shut_mouth.items()}

open_eyes = {"meh": "1.png", "mad": "2.png",
             "closed": "3.png", "sparkle closed": "4.png",
             "emotional": "5.png", "excited": "6.png",
             "open": "7.png", "dead": "8.png", "cool": "10.png"}



# Configuración de la API de OpenAI
#openai.api_key = os.getenv("OPENAI_API_KEY")
openai.api_key = "sk-Dn6iENzR1uW95JT4AFzuhN7GrtB_duDX9K4iubzBS8T3BlbkFJkVha-GIuv4FiF6TaUdKbbxwaQ3NqucC7dpIH0uoMIA"  # Coloca tu clave de OpenAI aquí
print(openai.api_key )

# Mapear emociones a expresiones de ojos y bocas
emocion_a_expresion = {
    "agradable-neutral": {"shut_mouth": "default", "open_eyes": "open"},
    "emocionado": {"shut_mouth": "cat", "open_eyes": "excited"},
    "aburrido": {"shut_mouth": "meh", "open_eyes": "meh"},
    "ligeramente deprimido": {"shut_mouth": "pout", "open_eyes": "meh"},
    # Agregar más emociones si es necesario
}

def detectar_emocion_y_expresion(texto):
    # Detectar la emoción usando OpenAI
    respuesta = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {
                "role": "system",
                "content": """Eres un asistente que identifica emociones en textos y las clasifica en las siguientes categorías: 
                                neutro, feliz, enojado, desagrado, tierno, silly, desinteres."""
            },
            {
                "role": "user",
                "content": f"Identifica la emoción predominante en este texto: \"{texto}\"."
            },
        ],
        max_tokens=10,
        temperature=0.7,
    )
    emocion = respuesta["choices"][0]["message"]["content"].strip().lower()

    # Seleccionar las expresiones correspondientes
    expresion = emocion_a_expresion.get(emocion, {"shut_mouth": "default", "open_eyes": "open"})
    shut_mouth_expression = expresion["shut_mouth"]
    open_eyes_expression = expresion["open_eyes"]

    return emocion, shut_mouth_expression, open_eyes_expression

# Crear un bucle de ojos parpadeando
def create_eyes_blink_loop(eyes_folder, open_eyes, total_duration, fps=24):
    eye_states = ["closed", "open"]  # Ejemplo: abierto -> cerrado -> abierto
    state_durations = [0.1, 2]  # Duraciones en segundos para cada estado
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

unit_duration = total_duration / len(respuesta)
duration_per_vocal = 0
# Convertir texto a secuencia de imágenes y duraciones
sequence = []
sequence_points=[]
durations = []
for char in respuesta:  # Incluir espacios en el texto
    duration_per_vocal += unit_duration
    if char in [" ", ".", "!", "?", "(", ")", ",",]:
        image_file = os.path.join(image_folder, shut_mouth["default"])
        if os.path.exists(image_file):
            sequence.append(image_file)
            # Suma las duraciones de las consonantes previas
            durations.append(duration_per_vocal)
            duration_per_vocal = 0
        points_file = os.path.join(points_folder, shut_mouth_points["default"])
        if os.path.exists(points_file):
            sequence_points.append(points_file)
    #if char in ["a","e","i","y","o","u"]:
    if char in phoneme_to_image:  # Buscar letra o espacio en el diccionario
        image_file = os.path.join(image_folder, phoneme_to_image[char])
        if os.path.exists(image_file):
            sequence.append(image_file)
            # Suma las duraciones de las consonantes previas
            durations.append(duration_per_vocal)
            duration_per_vocal = 0
    if char in phoneme_to_points:  # Buscar letra o espacio en el diccionario
        points_file = os.path.join(points_folder, phoneme_to_points[char])
        if os.path.exists(points_file):
            sequence_points.append(points_file)
    



# Crear el video con MoviePy
duracion_of_transition = unit_duration/6 
if sequence:
    clips = []
    for i in range(len(sequence)):
        image = sequence[i]
        duration = durations[i]
        clip = ImageClip(image, duration = duration - duracion_of_transition)  # Duración de cada imagen
        clips.append(clip)
        if 0 <= i < len(sequence) - 1:
            if sequence[i] != os.path.join(image_folder, shut_mouth["default"]) and sequence[i+1] != os.path.join(image_folder, shut_mouth["default"]):
                frames = morph_transition(sequence[i] ,sequence[i+1] ,sequence_points[i] ,sequence_points[i+1], num_images=2)
                for frame in frames:
                    # Convertir PIL Image a numpy array
                    frame_np = np.array(frame)
                    
                    # Crear un ImageClip a partir del array numpy
                    clip = ImageClip(frame_np, duration = duracion_of_transition)  # Duración de cada imagen
                    
                    # Agregar el clip a la lista
                    clips.append(clip)
            else:
                clip = ImageClip(image, duration = duracion_of_transition)  # Duración de cada imagen
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
                