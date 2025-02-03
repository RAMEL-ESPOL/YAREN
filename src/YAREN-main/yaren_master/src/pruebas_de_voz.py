import time
import os
from TTS.api import TTS
from concurrent.futures import ThreadPoolExecutor
from pydub import AudioSegment
from gtts import gTTS

# Carga el modelo una vez
tts = TTS(model_name="tts_models/es/css10/vits", progress_bar=False, gpu=False)

def ajustar_pitch(audio_path, pitch_semitonos, velocidad):
    """
    Ajusta el pitch sin modificar la velocidad del audio.
    Usa un archivo temporal para evitar errores de sobrescritura.
    """
    temp_audio_path = audio_path + ".temp.mp3"
    cambio_pitch = 2 ** (pitch_semitonos / 12.0)  # Calcula el cambio de tono
    comando = f"ffmpeg -y -i {audio_path} -filter:a rubberband=pitch={cambio_pitch}:tempo={velocidad} {temp_audio_path}"
    os.system(comando)

    if os.path.exists(audio_path):
        os.remove(audio_path)

    os.rename(temp_audio_path, audio_path)  # Renombra el archivo temporal al archivo original


def generar_audio_fragmento(texto, audio_path, pitch_semitonos=5, velocidad=1.2):
    """
    Genera el audio para un fragmento de texto y ajusta el pitch.
    """
    tts.tts_to_file(text=texto, file_path=audio_path)
    ajustar_pitch(audio_path, pitch_semitonos, velocidad)

def generar_audio_en_paralelo(texto, audio_base_path, pitch_semitonos=5, velocidad=1.2):
    """
    Divide el texto en oraciones y genera audio en paralelo con ajuste de pitch.
    """
    oraciones = texto.split(".")
    with ThreadPoolExecutor() as executor:
        futures = []
        for i, oracion in enumerate(oraciones):
            if oracion.strip():
                audio_path = f"{audio_base_path}_parte_{i + 1}.mp3"
                futures.append(executor.submit(generar_audio_fragmento, oracion.strip(), audio_path, pitch_semitonos))
        for future in futures:
            future.result()  # Espera a que terminen todos los hilos

def generar_audio(texto, audio_path, pitch_semitonos=5, velocidad=1.2):
    # Generar el audio
    tts.tts_to_file(text=texto, file_path=audio_path, split_by_sentences=False)
    ajustar_pitch(audio_path, pitch_semitonos, velocidad)
    print(f"Audio guardado en: {audio_path}")

def generar_audio_anterior(respuesta, audio_path):
    tts = gTTS(respuesta, lang='es')
    tts.save(audio_path)


if __name__ == "__main__":
    path = os.path.dirname(os.path.abspath(__file__)) + "/"
    audio_path, trimmed_audio_path = path + "audio.mp3", path + "trimmed_audio.mp3"
    audio_path_anterior = path + "audio_anterior.mp3"
    print(path)

    respuesta = "si estoy muy bien gracias por preguntar, dime en que puedo ayudarte hoy!"
    # Inicia el temporizador
    start_time = time.time()

    generar_audio(respuesta, audio_path, pitch_semitonos=4, velocidad=1.2)
    generar_audio_anterior(respuesta,audio_path_anterior)
    
    # Termina el temporizador y calcula la duración
    elapsed_time = time.time() - start_time
    print(f"Tiempo de ejecución: {elapsed_time:.2f} segundos")