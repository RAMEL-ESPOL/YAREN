### Descripción del paquete

Este paquete contiene los archivos necesarios para la puesta en escena del robot, abarcando tanto el control de sus movimientos como la visualización de su rostro.

### Estructura del paquete

```
- movements/
- rostro/
- scripts/
  ├── movement.py
  ├── voz_movement.py
  └── yaren_face.py
```

### Descripción de los archivos

- **`movement.py`**: Script que permite controlar las posiciones del robot mediante comandos por consola.
- **`voz_movement.py`**: Script que permite ejecutar movimientos del robot a través de comandos de voz. Para su funcionamiento, es necesario utilizar el paquete [AI-FaceSoftware](https://github.com/RAMEL-ESPOL/AI-FaceSoftware.git). Algunos de los movimientos utilizados en este script se leen desde archivos `.txt` ubicados en la carpeta `movements/`, los cuales han sido generados previamente mediante la interfaz de creación de movimientos.
- **`yaren_face.py`**: Script que reproduce en bucle una secuencia de imágenes que representan el rostro del robot. Las imágenes utilizadas se encuentran en la carpeta `rostro/`.

### Cómo agregar nuevos movimientos usando archivos `.txt` generados

Para agregar nuevos movimientos al archivo `voz_movement.py`, dirígete a la sección final del script, donde se definen los movimientos por número de comando. Puedes seguir el siguiente formato utilizando una estructura `elif`:

```python
elif self.number == '7':
    while not rospy.is_shutdown():
        file_path = movements_folder + "/baile_de_barney.txt"
        self.process_file(file_path)
```

Cada `self.number` representa un identificador de comando asociado a un movimiento específico. Para agregar uno nuevo, simplemente continúa con la numeración y especifica el nombre del archivo `.txt` correspondiente:

```python
elif self.number == '8':
    while not rospy.is_shutdown():
        file_path = movements_folder + "/nuevo_baile.txt"
        self.process_file(file_path)
```

Asegúrate de que el archivo `.txt` se encuentre en la carpeta `movements/` y haya sido creado con la interfaz de movimientos.
