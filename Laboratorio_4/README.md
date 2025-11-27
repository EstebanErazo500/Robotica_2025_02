
# Laboratorio No. 04

## Robótica de Desarrollo | Intro a ROS 2 con Turtlesim

* Edgar Esteban Erazo Lagos

# Introducción

Este laboratorio se construye a partir del repositorio compartido en GitHub por el grupo, donde se define la estructura del workspace y el flujo general de ejecución.

La idea no fue repetir ese contenido, sino tomarlo como base y reorganizar el trabajo para que el nodo desarrollado tuviera una lógica más clara y unificada.

Se respetan los mismos comandos de compilación y ejecución descritos en el README, manteniendo coherencia con la guía original.

> Sugerencia visual: captura del repositorio en GitHub mostrando el archivo move_turtle.py.


## 2. Enfoque del laboratorio

El objetivo práctico se centra en controlar la tortuga desde un único script, combinando dos comportamientos:

* Movimiento manual mediante las teclas de flecha.
* Dibujo automático de letras asociadas a los nombres de los integrantes.

A diferencia de dividir estos comportamientos en varios nodos, se optó por integrarlos para evitar conflictos sobre el tópico /turtle1/cmd_vel y mantener un manejo más coherente del flujo de control.

---

## 3. Nodo principal desarrollado

El nodo creado se denomina turtle_controller y se implementa en el archivo move_turtle.py.

Desde este nodo se gestionan:

* Publicación de velocidades en /turtle1/cmd_vel.
* Lectura de la pose actual desde /turtle1/pose.
* Uso del servicio /turtle1/teleport_absolute para reiniciar la posición tras completar cada letra.
* Captura de teclado en tiempo real mediante un hilo independiente.

> Imagen sugerida: fragmento del código donde se define la clase TurtleController.

---

## 4. Control manual con teclado

El movimiento con flechas se implementa sin utilizar paquetes externos de teleoperación, directamente desde el script.

Cada pulsación genera un pequeño desplazamiento:

* Flecha arriba: avance.
* Flecha abajo: retroceso.
* Flecha izquierda: giro antihorario.
* Flecha derecha: giro horario.

Esto permite un control fino de la trayectoria y facilita posicionar la tortuga antes de iniciar el dibujo de las letras.

> Imagen sugerida: trayectoria libre dibujada con flechas.

---

## 5. Dibujo de letras

Las letras implementadas corresponden a las iniciales del grupo: D, C, V, M, F y Q.

El procedimiento general para cada letra es:

* Guardar la pose inicial.
* Ejecutar movimientos rectos y giros calculados.
* Combinar tramos lineales y trayectorias circulares según la forma requerida.
* Retornar a la pose inicial mediante teletransporte.

Esto permite escribir múltiples letras manteniendo una referencia espacial estable.

---

## 6. Lógica geométrica empleada

Para estructurar el trazo se definieron funciones auxiliares reutilizables:

* move_distance: desplazamiento lineal controlado.
* turn_angle: rotación en sitio por ángulo definido.

A partir de ellas:

* C se representa como un semicírculo.
* D combina un trazo vertical con una curva lateral.
* V se ajustó para que sus brazos fueran simétricos respecto al eje inicial.
* Q se dibuja como un círculo completo más una cola inclinada.

Este enfoque modular simplifica la lectura y futura extensión del código.

---

## 7. Integración en un solo script

Frente a la versión inicial encontrada en GitHub, donde se evidencian fragmentos separados y código redundante, se reorganizó el contenido para que todo funcione desde un único nodo.

Esto evita la coexistencia de múltiples publicadores sobre el mismo tópico y mejora la previsibilidad del comportamiento del sistema.

El usuario solo debe ejecutar un comando para acceder tanto al control manual como al dibujo automático.

---

## 8. Proceso de prueba y ajuste

Las pruebas se realizaron de forma progresiva:

* Verificación del movimiento básico con flechas.
* Prueba individual de cada letra.
* Ajuste de longitudes, radios y ángulos hasta obtener formas reconocibles.
* Corrección de la V para lograr una forma más equilibrada visualmente.

El resultado final permite visualizar claramente cada letra en la interfaz de Turtlesim.

> Imagen sugerida: capturas mostrando varias letras dibujadas.

---

## 9. Resultados obtenidos

Se logró:

* Control preciso de la tortuga desde teclado.
* Generación de letras claras a partir de combinaciones de movimientos.
* Integración estable de tópicos y servicios de ROS 2.
* Un código más ordenado respecto a la versión base encontrada en GitHub.

Esto demuestra la correcta comprensión del modelo de comunicación de ROS 2 en un entorno gráfico simple.

---

## 10. Conclusiones

El laboratorio permitió evidenciar de forma práctica la interacción entre nodos, tópicos y servicios en ROS 2.

La reorganización del código base encontrado en GitHub facilitó una mejor comprensión de la lógica de control y redujo problemas de redundancia.

El sistema desarrollado constituye una base funcional sobre la cual se pueden construir comportamientos más complejos, como escritura automática de palabras completas o integración con otras interfaces de entrada.

---

Contenido listo para ser adaptado directamente a diapositivas, manteniendo coherencia con el trabajo real y sin replicar de forma literal el formato del repositorio original.
