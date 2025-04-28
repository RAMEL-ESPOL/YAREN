import cv2
import numpy as np
import os

def create_video_from_images(initial_image_path, final_image_path, transition_frames_dir, output_video, duration=2, fps=30):
    # Leer la imagen inicial
    initial_image = cv2.imread(initial_image_path, cv2.IMREAD_UNCHANGED)
    if initial_image is None:
        print(f"Error: No se pudo cargar la imagen inicial {initial_image_path}.")
        return

    # Leer la imagen final
    final_image = cv2.imread(final_image_path, cv2.IMREAD_UNCHANGED)
    if final_image is None:
        print(f"Error: No se pudo cargar la imagen final {final_image_path}.")
        return

    # Leer las imágenes intermedias de la carpeta
    transition_frames = sorted(
        [os.path.join(transition_frames_dir, img) for img in os.listdir(transition_frames_dir) if img.endswith(".png")]
    )
    if not transition_frames:
        print(f"Error: No se encontraron imágenes intermedias en {transition_frames_dir}.")
        return

    # Calcular la cantidad total de frames necesarios
    total_frames = int(duration * fps)

    # Asegurar que las imágenes inicial y final tengan las mismas dimensiones que las intermedias
    transition_sample = cv2.imread(transition_frames[0], cv2.IMREAD_UNCHANGED)
    height, width = transition_sample.shape[:2]

    initial_image = cv2.resize(initial_image, (width, height), interpolation=cv2.INTER_AREA)
    final_image = cv2.resize(final_image, (width, height), interpolation=cv2.INTER_AREA)

    # Crear el video
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    # Fondo transparente (RGBA)
    transparent_background = np.zeros((height, width, 4), dtype=np.uint8)

    # Combinar fondo transparente con cada imagen y escribir al video
    def blend_with_background(image, background):
        # Si la imagen tiene canal alfa, usarla como máscara
        if image.shape[2] == 4:
            alpha = image[:, :, 3] / 255.0
            for c in range(3):  # Para cada canal RGB
                background[:, :, c] = (
                    image[:, :, c] * alpha + background[:, :, c] * (1 - alpha)
                )
            background[:, :, 3] = 255  # Mantener el canal alfa completamente opaco
        return background

    # Agregar la imagen inicial
    blended_initial = blend_with_background(initial_image.copy(), transparent_background.copy())
    video_writer.write(cv2.cvtColor(blended_initial, cv2.COLOR_BGRA2BGR))

    # Agregar las imágenes intermedias
    for frame_path in transition_frames:
        frame = cv2.imread(frame_path, cv2.IMREAD_UNCHANGED)
        if frame is None:
            print(f"Advertencia: No se pudo cargar el frame {frame_path}.")
            continue
        blended_frame = blend_with_background(frame.copy(), transparent_background.copy())
        video_writer.write(cv2.cvtColor(blended_frame, cv2.COLOR_BGRA2BGR))

    # Agregar la imagen final
    blended_final = blend_with_background(final_image.copy(), transparent_background.copy())
    video_writer.write(cv2.cvtColor(blended_final, cv2.COLOR_BGRA2BGR))

    video_writer.release()
    print(f"Video generado exitosamente: {output_video}")

# Rutas de los archivos e imágenes
initial_image_path = "6.png"
final_image_path = "15.png"
transition_frames_dir = "transition_frames_main_6 a 15"
output_video = "final_video_6a15.mp4"

# Crear el video
create_video_from_images(initial_image_path, final_image_path, transition_frames_dir, output_video)
