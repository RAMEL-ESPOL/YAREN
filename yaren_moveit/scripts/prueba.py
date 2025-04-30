#!/usr/bin/env python3

import sys
import os
import json
import comandos

# Agrega la carpeta principal catkin_ws/src
directorio_principal = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.append(directorio_principal)

# Ahora puedes usar funciones o clases de `utilidad`
from asistente_local.utils import read_file
from asistente_local.asistente import Asistente

CONFIG_PARAMS = read_file(directorio_principal + "/" + "config", "yaml")

def main():

    model = CONFIG_PARAMS["stt"]["model_size"]
    record_timeout = CONFIG_PARAMS["stt"]["recording_time"]
    phrase_timeout = CONFIG_PARAMS["stt"]["silence_break"]
    energy_threshold = CONFIG_PARAMS["stt"]["sensibility"]
    wake_word = CONFIG_PARAMS["asistente"]["wake_word"]

    comandos = CONFIG_PARAMS["comandos"]
    prompt = CONFIG_PARAMS["prompt"]

    try:
        with open(directorio_principal+ "/" +"respuestas_basicas.json", "r", encoding='utf-8') as f:
            respuestas_basicas = json.load(f)
            respuestas_basicas = {key.lower(): value for key, value in respuestas_basicas.items()}
    except FileNotFoundError:
            print("El archivo 'respuestas_basicas.json' no se encontró.")
            respuestas_basicas = {}
    except json.JSONDecodeError:
            print("Error al decodificar el archivo JSON. Verifica el formato.")
            respuestas_basicas = {}
    
    va = Asistente(model, record_timeout, phrase_timeout, energy_threshold, wake_word,comandos,prompt,respuestas_basicas)
    while True:
        pose_num= va.listen_movement_comand()
        #96 + pose_num, porque 'a' es la posición 1, pero como pose_num incia desde 0
        pose = chr(97 + pose_num)  # 97 + pose_num, para compensar que pose_num incia desde 0 y no en 1
        print(pose)
        print(comandos[pose_num])
        comandos.move_group_python_interface(pose)
        #va.write_transcript()



if __name__ == "__main__":
    main()

#en caso de que no coja el mic poner esto en bash: rm -rf ~/.config/pulse/ 



