# Aplicación de inteligencia artificial a un robot humanoide para el tratamiento de estrés
## Ejecución del funcionamiento
### Primer paso
    Habilitar permisos 
```bash
    sudo chmod 666 /dev/ttyUSB0
```
### Segundo paso
    Lanzamiento completo del sistema
```bash
    roslaunch yaren_u2d2 yaren_full.launch
```

## Adición de movimientos
### Primer paso
    Habilitar permisos 
```bash
    sudo chmod 666 /dev/ttyUSB0
```
### Segundo paso
```bash
    roslaunch yaren_u2d2 yaren_communication_datos.launch
```
    
### Tercer paso


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

# Secuencias de Rostros

Se posee un conjunto de imagenes de los diferentes rostros que se pueden escoger para realizar una secuencia. Todos estos rostros se encuentran en [yaren_master/rostro](https://github.com/RAMEL-ESPOL/YAREN/tree/main/yaren_master/rostro).


## Importación de nuevas imagenes

En caso de querer nuevas imagenes aparte de agregarlas en la ruta correcta tambien se tiene que añadir el nombre con su respectiva extensión en el codigo de python [yaren_face.py](https://github.com/RAMEL-ESPOL/YAREN/blob/main/yaren_master/src/yaren_face.py).


## Agregar una imagen en el código

Dentro del archivo [yaren_face.py](https://github.com/RAMEL-ESPOL/YAREN/blob/main/yaren_master/src/yaren_face.py) agregar la siguiente línea:

```python
  IMAGEN_EXPRESION = carpetaImgs + "COLOCAR_NOMBRE_DE_LA_IMAGEN"
```
**No cambiar ni eliminar la variable carpetaImgs.**

| Parameter | Description                       |
| :-------- | :-------------------------------- |
| `carpetaImgs`| Ruta de la carpeta que contiene las imágenes |

## Generar una nueva secuencia de imágenes

Ingresar en la lista las imagenes que se deseen dentro de la secuencia, las imagenes se mostrarán en el orden que se coloquen.

Dentro del archivo [yaren_face.py](https://github.com/RAMEL-ESPOL/YAREN/blob/main/yaren_master/src/yaren_face.py) agregar la siguiente línea:

```python
  SECUENCIA_IMAGENES = [IMG_1, IMG_2, IMG_3, ...]
```

| Parameter | Description                       |
| :-------- | :-------------------------------- |
| `IMG_N`| Nombre de la imagen para la secuencia |

## Mover la ventana hacia la pantalla del YAREN

Para mover la ventana que contiene la secuencia de imágenes se tiene que colocar sobre la ventana y pulsar las siguientes teclas según sea el caso:

- Tecla 'w': Mover la ventana hacia arriba
- Tecla 's': Mover la ventana hacia abajo
- Tecla 'a': Mover la ventana hacia la izquierda
- Tecla 'd': Mover la ventana hacia la derecha
