# Lab 05 – Cinemática directa PhantomX Pincher X100

## 1. Descripción general

En este laboratorio se implementa y prueba la cinemática directa del robot PhantomX Pincher X100 usando ROS 2 y una interfaz gráfica en Python.  
El objetivo es que el robot físico y el modelo digital en RViz muestren el mismo comportamiento cuando se mueven las articulaciones, tanto con controles manuales como con poses predefinidas.

Se trabaja sobre el paquete `pincher_control`, que:
- Se conecta a los servomotores Dynamixel.
- Publica `joint_states` para RViz.
- Calcula la pose del TCP con un modelo DH.
- Ofrece una GUI con varias pestañas de control.


## 2. Hardware y software usados

- Robot **PhantomX Pincher X100** con 5 grados de libertad.
- Servomotores Dynamixel AX (configurados con:
  - `PROTOCOL_VERSION = 1.0`
  - `baudrate = 57600`
  - `dxl_ids = [1, 2, 3, 4, 5]`)
- Adaptador USB–Serial para la cadena Dynamixel (`/dev/ttyUSB0`).
- Computador con:
  - Ubuntu 22.04 en máquina virtual.
  - ROS 2 Humble.
  - Python 3.10.
- Paquetes ROS 2 relevantes:
  - `pincher_control`
  - `phantomx_pincher_description`
  - `phantomx_pincher_interfaces` (para el mensaje `PoseCommand`).


## 3. Estructura del paquete `pincher_control`

El archivo principal es `control_servo.py`. Dentro se encuentran dos bloques grandes:

- `class PincherController(Node)`  
  Nodo de ROS 2 que:
  - Abre el puerto serie.
  - Configura el torque, velocidad y posición inicial de los motores.
  - Publica en `/joint_states`.
  - Calcula la cinemática directa del TCP y la publica en `/tcp_pose`.
  - Publica un `Marker` con texto para ver XYZ y RPY en RViz.
  - (Opcional) Publica `PoseCommand` hacia MoveIt para control en espacio de la tarea.

- `class PincherGUI`  
  Interfaz gráfica en Tkinter con pestañas para:
  - Control por sliders.
  - Control por valores numéricos.
  - Poses predefinidas del laboratorio.
  - Visualización en RViz.
  - Información del laboratorio.
  - Control en espacio de la tarea (XYZ + RPY).


## 4. Interfaz gráfica – control articular

### 4.1 Pestaña 1 – Control por sliders

En esta pestaña se mueven las articulaciones en tiempo real con sliders en grados:

- Cada motor tiene:
  - Etiqueta `Motor i`.
  - Slider de ángulo en el rango de `MIN_ANGLE_DEG` a `MAX_ANGLE_DEG`.
  - Etiqueta con la posición actual en grados.
- Debajo hay un slider global de velocidad:
  - Si la velocidad es 0, no se envían comandos a los motores.
  - La velocidad se aplica a todos los motores por medio de `update_speed`.

El movimiento pasa por `degrees_to_dxl` y luego por `move_motor`, que:
- Actualiza `current_joint_positions` para que RViz se mantenga sincronizado.
- Envía los ticks al servo si hay hardware disponible.

### 4.2 Pestaña 2 – Control por valores manuales

En esta pestaña se mueven los motores escribiendo directamente el ángulo en grados:

- Para cada motor:
  - Campo de texto con el ángulo.
  - Botón "Mover Motor".
  - Etiqueta de estado ("Listo", "Enviado", errores de rango, etc.).
- Hay un botón "MOVER TODOS LOS MOTORES" que:
  - Lee todos los campos.
  - Convierte a ticks.
  - Envia el comando a cada servo, si están dentro de los límites.

La pestaña comparte el mismo control de velocidad que la pestaña 1, y actualiza los sliders cuando se envían nuevos comandos.


## 5. Visualización en RViz

### 5.1 Pestaña 3 – RViz

Esta pestaña sirve para lanzar y detener RViz desde la GUI:

- Botón **LANZAR RViz**:
  - Ejecuta:
    ```bash
    ros2 launch phantomx_pincher_description view.launch.py
    ```
  - Abre el modelo del PhantomX Pincher X100 y el `robot_state_publisher`.
- Botón **DETENER RViz**:
  - Termina el proceso lanzado.
  - Actualiza el estado de la interfaz.

Además, el nodo `PincherController`:

- Publica `JointState` en `/joint_states`.
- Publica `PoseStamped` del TCP en `/tcp_pose`.
- Publica un `Marker` con texto en `/tcp_pose_marker` para ver coordenadas y ángulos del efector final.


## 6. Poses del laboratorio y control global

### 6.1 Poses del laboratorio

En la pestaña de **Control por pose** se definen:

- Cinco poses "Lab" usadas en la guía del laboratorio:
  - Pose Lab 1: `[0, 0, 0, 0, 0]`.
  - Pose Lab 2: `[25, 25, 20, -20, 0]`.
  - Pose Lab 3: `[-35, 35, -30, 30, 0]`.
  - Pose Lab 4: `[85, -20, 55, 25, 0]`.
  - Pose Lab 5: `[80, -35, 55, -45, 0]`.
- Dos poses personalizadas:
  - Pose 6: `[10, 20, 30, 40, 50]`.
  - Pose 7: `[-10, -20, -30, -40, -50]`.

Cada botón llama a `start_pose_sequence`, que:

- Recorre los motores en orden (o en orden inverso, según la pose).
- Envía los ángulos uno por uno con retraso entre articulaciones.
- Actualiza el estado del botón y la etiqueta de estado general.

### 6.2 Botones HOME y parada de emergencia

En la parte inferior de la ventana hay dos botones globales:

- **HOME**:
  - Llama a `home_all_motors`.
  - Lleva todos los motores a `DEFAULT_GOAL`, que corresponde a 0° en la GUI.
  - Sincroniza sliders y campos de texto a 0.

- **PARADA DE EMERGENCIA**:
  - Llama a `emergency_stop`.
  - Desactiva el torque de todos los motores.
  - Cambia los mensajes de estado a "EMERGENCIA".
  - Para volver a mover el robot, se reactivan torques llamando a `home_all_motors` o a la lógica de reactivación.


## 7. Plano de planta y distribución de elementos

En esta sección se documenta cómo está organizado el montaje físico sobre la mesa:

- Posición del PhantomX Pincher X100 respecto al borde de la mesa.
- Ubicación del computador y la pantalla desde donde se opera la GUI.
- Ruta de los cables de alimentación y comunicación para que no interfieran con el movimiento.
- Zona segura alrededor del brazo para evitar golpes con objetos cercanos.

Espacio para imagen del plano de planta con el robot, la mesa y los elementos principales.


## 8. Resumen de funciones principales del código

Aquí se resumen las funciones principales usadas en este laboratorio y su propósito:

- `initialize_motors(...)`  
  Configura torque, velocidad y posición inicial de cada motor y deja al robot en HOME.

- `dxl_to_radians(...)` / `radians_to_dxl(...)`  
  Conversión entre ticks Dynamixel y radianes para manejar cinemática.

- `dxl_to_degrees(...)` / `degrees_to_dxl(...)`  
  Conversión entre ticks y grados para que la GUI sea intuitiva.

- `move_motor(...)`  
  Actualiza la posición interna de la articulación y envía el comando al servo (si hay hardware).

- `update_speed(...)` y `update_speed_single_motor(...)`  
  Ajustan la velocidad de todos los motores o de un motor específico.

- `home_all_motors()`  
  Lleva el brazo a HOME y deja el sistema en una postura de referencia.

- `emergency_stop()` / `reactivate_torque()`  
  Manejan la parada de emergencia y la posterior reactivación del sistema.

- `compute_tcp_fk()` y `update_tcp_pose()`  
  Calculan la cinemática directa del TCP con el modelo DH y publican su pose en ROS 2.

- `publish_joint_states()`  
  Publica `JointState` en `/joint_states` para sincronizar con RViz.

- `update_joints_timer()`  
  Actualiza en la GUI la barra inferior con XYZ, RPY y los ángulos `q1` a `q5`.

- `start_pose_sequence(...)` y `run_pose_step(...)`  
  Ejecutan las poses predefinidas del laboratorio, respetando velocidad y estado de emergencia.


## 9. Cómo ejecutar el laboratorio

**Compilación del paquete**

Desde el espacio de trabajo:

```bash
cd ~/Escritorio/KIT_Phantom_X_Pincher_ROS2/robotica-proyecto-final/phantom_ws
colcon build --packages-select pincher_control
source install/setup.bash

