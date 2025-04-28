import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_contour_points(image_path, num_points=24):
    # Cargar la imagen con fondo transparente
    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

    # Asegurarnos de que la imagen tiene un canal alfa
    if image.shape[2] != 4:
        print("La imagen debe tener un canal alfa (transparencia).")
        return image, []

    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Usar Canny para detectar los bordes
    edges = cv2.Canny(gray, threshold1=100, threshold2=200)

    # Encontrar contornos
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Seleccionar el contorno más grande
    if len(contours) == 0:
        print("No se encontraron contornos.")
        return image, []

    largest_contour = max(contours, key=cv2.contourArea)

    # Ajustar una elipse al contorno más grande
    if len(largest_contour) >= 5:
        ellipse = cv2.fitEllipse(largest_contour)
        center, axes, angle = ellipse
        axis1, axis2 = axes

        # Generar puntos a lo largo de la elipse
        points = []
        for i in range(num_points):
            angle_rad = 2 * np.pi * i / num_points
            x = int(center[0] + axis1 * np.cos(angle_rad) * np.cos(np.radians(angle)) - axis2 * np.sin(angle_rad) * np.sin(np.radians(angle)))
            y = int(center[1] + axis1 * np.cos(angle_rad) * np.sin(np.radians(angle)) + axis2 * np.sin(angle_rad) * np.cos(np.radians(angle)))
            points.append((x, y))

        # Encontrar el punto más cercano a (400, 0)
        reference_point = (400, 0)
        distances = [np.linalg.norm(np.array(point) - np.array(reference_point)) for point in points]
        start_index = np.argmin(distances)

        # Reordenar los puntos comenzando desde el más cercano a (400, 0)
        ordered_points = points[start_index:] + points[:start_index]

        # Dividir puntos en dos segmentos según el eje y
        segment1 = [p for p in ordered_points if p[1] <= 240]  # De (400, 0) a (400, 480)
        segment2 = [p for p in ordered_points if p[1] > 240]   # De (400, 480) a (400, 0)

        # Combinar los segmentos para el orden deseado
        final_points = segment1 + segment2

        # Dibujar los puntos en la imagen
        image_copy = image.copy()
        for point in final_points:
            cv2.circle(image_copy, point, 5, (0, 0, 255), -1)
        cv2.ellipse(image_copy, ellipse, (255, 0, 0), 2)

        return image_copy, final_points
    else:
        print("No se pudo ajustar una elipse debido a un contorno insuficiente.")
        return image, []

# Ruta de la imagen
image_path = '3.png'
image_with_points, points = detect_contour_points(image_path, num_points=24)

# Mostrar la imagen y guardar los puntos
plt.imshow(cv2.cvtColor(image_with_points, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()

# Guardar los puntos en un archivo .txt
output_file = image_path.split('/')[-1].split('.')[0] + "_points.txt"
with open(output_file, 'w') as f:
    for point in points:
        f.write(f"{point[0]},{point[1]}\n")
print(f"Puntos guardados en: {output_file}")
