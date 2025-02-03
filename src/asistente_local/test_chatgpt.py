import openai
import sys
import os
from utils import read_file
import json
import PyPDF2


# Configuración de la clave de API
#openai.api_key = os.getenv("OPENAI_API_KEY")  # O reemplaza con tu clave directamente
openai.api_key = "sk-Dn6iENzR1uW95JT4AFzuhN7GrtB_duDX9K4iubzBS8T3BlbkFJkVha-GIuv4FiF6TaUdKbbxwaQ3NqucC7dpIH0uoMIA"  # Coloca tu clave de OpenAI aquí
print(openai.api_key )

# Agrega la carpeta principal catkin_ws/src
directorio_principal = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(directorio_principal)

CONFIG_PARAMS = read_file(directorio_principal + "/" +"config", "yaml")

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


# Variable donde se almacenará el texto completo
historia = ""

# Abrir el archivo PDF
with open(directorio_principal + "/" + "T-113027 UCHUARI YANDRI-VALDEZ RHANDALL.pdf", 'rb') as pdf_file:
    # Crear un lector de PDF
    pdf_reader = PyPDF2.PdfReader(pdf_file)
    
    # Iterar sobre todas las páginas del PDF
    for page_num in range(len(pdf_reader.pages)):
        page = pdf_reader.pages[page_num]
        historia += page.extract_text()

# Mostrar el contenido de la variable historia
#print(historia)


def prueba_chat_gpt(texto):
    try:
        respuesta = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",  # Usa el modelo más reciente disponible
                messages=[{
                            "role": "system",
                            "content": [
                                            {
                                                "type": "text",
                                                "text": "Tu nombre es YAREN , eres un robot humanoide diseñado para dar acompañamiento a niños hospitalarios Etc etc Necesito que me respondas en menos de 50 palabras."

                                            },{
                                                "type": "text",
                                                "text": json.dumps(respuestas_basicas, ensure_ascii=False, indent=4)

                                            },{
                                                "type": "text",
                                                "text": historia

                                            },
                                        ],
                            },{"role": "user", "content": texto}],
                temperature=0.5,
                max_tokens=150,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0.6
            )
        # Mostrar la respuesta
        return respuesta['choices'][0]['message']['content']
    except Exception as e:
        return f"Error en la llamada a la API: {e}"

# Prueba de la API
mensaje = "cuantos motores tienes"
respuesta = prueba_chat_gpt(mensaje)
print("Respuesta de ChatGPT:", respuesta)
