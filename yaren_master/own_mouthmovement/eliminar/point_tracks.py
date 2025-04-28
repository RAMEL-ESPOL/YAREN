import cv2
import matplotlib.pyplot as plt
import numpy as np

# Variable global para almacenar los puntos
points = []

# Umbral de proximidad para eliminar un punto (en píxeles)
proximity_threshold = 20

# Función para ordenar puntos en un orden específico (por ejemplo, por coordenada x)
def order_points(points):
    return sorted(points, key=lambda p: (p[1], p[0]))  # Orden por coordenada Y (vertical), luego por X

# Función de callback para capturar los puntos al hacer clic en la imagen
def click_event(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:  # Clic izquierdo, agregar punto
        points.append((x, y))
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Image", image)

    elif event == cv2.EVENT_RBUTTONDOWN:  # Clic derecho, eliminar punto cercano
        for point in points:
            if np.sqrt((x - point[0])**2 + (y - point[1])**2) < proximity_threshold:
                points.remove(point)
                break  # Eliminar solo el primer punto cercano
        # Redibujar la imagen y los puntos restantes
        image_copy = image.copy()
        for point in points:
            cv2.circle(image_copy, point, 5, (0, 0, 255), -1)
        cv2.imshow("Image", image_copy)

# Cargar la imagen
image_path = 'bocas/3.png'  # Cambia la ruta a tu imagen
image = cv2.imread(image_path)

# Mostrar la imagen y esperar clics
cv2.imshow("Image", image)
cv2.setMouseCallback("Image", click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Ordenar los puntos en un orden específico (por ejemplo, por coordenada Y, luego por X)
ordered_points = order_points(points)

# Mostrar los puntos seleccionados
print("Puntos seleccionados (ordenados):", ordered_points)

# Generar el path (una lista de puntos que pueden usarse como una ruta)
path = ordered_points

# Guardar los puntos en un archivo txt
txt_file = image_path.split('/')[-1].split('.')[0] + "_points.txt"
with open(txt_file, 'w') as f:
    for point in path:
        f.write(f"{point[0]},{point[1]}\n")

print(f"Puntos guardados en: {txt_file}")
