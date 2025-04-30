### Descripción del paquete

Este paquete contiene los archivos necesarios para la comunicación y control de los motores del robot, así como para la reproducción del rostro animado mediante video. Se integran funciones de movimiento y visualización utilizando otros paquetes de soporte, como **AI-FaceSoftware**.

### Archivos de lanzamiento disponibles

Este paquete incluye dos archivos `launch` principales:

- **`yaren_full.launch`**:  
  Ejecuta la puesta en marcha completa del sistema.  
  - Inicia la comunicación con los motores mediante `yaren_communication.launch`.  
  - Envía los comandos de movimiento al robot.  
  - Reproduce el video del rostro del robot, conectándose al paquete [AI-FaceSoftware](https://github.com/RAMEL-ESPOL/AI-FaceSoftware.git). (primero descargar el paquete)

- **`yaren_communication_datos.launch`**:  
  Lanza una interfaz que permite editar manualmente las posiciones de los motores.

### Estructura del paquete

```
- launch/
  ├── yaren_communication_datos.launch
  ├── yaren_communication.launch
  └── yaren_full.launch    
- scripts/
  ├── u2d2_communication.py
  └── w_datos.py
```

### Descripción de los scripts

- **`u2d2_communication.py`**: Maneja la comunicación con los actuadores del robot a través del adaptador U2D2.
- **`w_datos.py`**: Script asociado a la interfaz gráfica para la edición de posiciones motoras lanzada por `yaren_communication_datos.launch`.
