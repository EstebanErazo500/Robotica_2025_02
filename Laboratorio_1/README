## Descripción del sistema y operaciones

**1. Objetivo general**
El propósito de este laboratorio fue diseñar e implementar un sistema automatizado para decorar un pastel usando RobotStudio con un robot ABB. Debía respetar las restricciones definidas: velocidades entre **100 y 1000 mm/s**, zona de precisión menor o igual a **z10**, retorno a la posición *HOME* con todas las articulaciones en 0°, manejo de dos entradas y dos salidas digitales, y control de la banda transportadora para despachar el pastel al final de la decoración.

---

**2. Arquitectura de la celda**
La celda consta de un robot ABB simulado (controlador IRC5), una herramienta denominada *Porta_Marcador* y un *WorkObject_2* situado sobre la banda o pastel.
El *Porta_Marcador* define el TCP y la orientación de escritura del marcador. El *WorkObject_2* fija las referencias del proceso: todas las trayectorias se definen respecto a ese sistema de coordenadas, lo que facilita que puedan cambiarse posiciones del pastel sin reprogramar cada punto.

Además, se definieron puntos clave para seguridad y funcionamiento:

* **Target_No_Golpear**, para evitar colisiones al entrar o salir del área de decoración.
* **Target_Pastel_Inicio**, punto de acceso a la zona donde se decorará.
* **Target_Mantenimiento**, lugar seguro para intervención del operador.
* **HOME** y **HOME_IR**, posiciones de retorno estándar (HOME_IR con articulaciones a 0°).

---

**3. Entradas, salidas y modo de operación**
Se usan dos señales de entrada digital:

* *DI_01* para iniciar el ciclo de producción,
* *DI_02* para cambiar al modo mantenimiento.

Las salidas digitales incluyen:

* *DO_01*: indicadora del robot activo,
* *DO_02*: indicadora de mantenimiento,
* *Conveyor_FWD*: activa la banda hacia el despacho,
* *Conveyor_INV*: queda como reserva.

En cada inicio de ciclo se ejecuta *Reset* sobre todas esas salidas (DO_01, DO_02, Conveyor_FWD, Conveyor_INV) para asegurar que el sistema empiece en estado seguro.

---

**4. Estrategia de homing y referencias**
El robot tiene dos modos de *homing*. Primero, *HOME_IR* es una posición articular con todas las articulaciones en 0°, ejecutado con *MoveAbsJ*. Luego, *HOME* es una posición cartesiana despejada usada para desplazamientos seguros.
La posición de mantenimiento (*Target_Mantenimiento*) se diseña para que el operador pueda intervenir con comodidad y sin riesgos. *Target_No_Golpear* actúa como punto intermedio para evitar que la herramienta cruce directamente zonas sensibles del pastel.

---

**5. Tool y WorkObject**
El *Porta_Marcador* recoge el desplazamiento geométrico hacia la punta del marcador y su orientación de escritura.
El *WorkObject_2* fija el sistema de coordenadas del pastel o banda, de modo que las trayectorias (Paths) usen siempre la misma referencia. Esto permite mover la pieza de trabajo, replicar el diseño o ajustar la posición sin tener que reprogramar los puntos del robot.

---

**6. Trayectorias de proceso**
Cada figura o letra se programa como un *Path_* independiente: *Path_D1*, *Path_A2*, *Path_V3*, etc.
Dentro de cada ruta se mezclan movimientos lineales (*MoveL*) para segmentos rectos y curvos (*MoveC*) para arcos, y *MoveJ* para accesos rápidos entre caminos.
Para garantizar calidad en el trazo se utiliza velocidad *v100* junto con zona *z1*. Para evitar cruces con lo ya decorado, después de cada figura el robot retorna a *Path_Pastel_Inicio*.

---

**7. Lógica del programa / flujo general**
El programa principal `main()` corre en bucle infinito (`WHILE TRUE`). Dentro de ese ciclo:

* Primero se resetean todas las salidas digitales.

* Si *DI_01 = 1* y *DI_02 = 0*, entra modo producción:
  • Activa *DO_01* y la banda para posicionar el pastel.
  • Detiene la banda y realiza la decoración usando la secuencia de *Paths*.
  • Al terminar activa la banda para despacho por unos segundos.
  • Regresa a *HOME* y *HOME_IR* y apaga las salidas.

* Si *DI_01 = 0* y *DI_02 = 1*, entra modo mantenimiento:
  • Se mueve a *HOME* y luego a *Target_Mantenimiento*.
  • Se enciende *DO_02*.
  • El programa espera (*WaitUntil DI_01=1 y DI_02=1*) hasta que el operador finalice el mantenimiento.
  • Cuando ambas señales están activas, vuelve al modo producción.

De esta forma el sistema nunca entra en conflicto entre modos y siempre retorna a estados seguros entre ciclos.

---

**8. Parámetros de movimiento y trazado**
Durante la decoración se usa velocidad *v100* y zona *z1* para maximizar precisión. Los movimientos de desplazamiento general usan *v1000* y zona *z100*.
El uso de *MoveC* permite suavizar las curvas, y *MoveJ* reduce el tiempo en desplazamientos sin contacto.

---

**9. Seguridad y colisiones**
Se implementan puntos de referencia seguros (*Target_No_Golpear*) para evitar que la herramienta cruce zonas del pastel o de la banda. Las posiciones de *HOME* garantizan despeje cuando el robot se mueve entre tareas. El modo mantenimiento se diseña para separar al robot de la zona de trabajo mientras el operador interviene.

---

**10. Reutilización y modularidad**
Al usar un único *WorkObject* para todas las trayectorias, el programa es adaptable: si la ubicación del pastel cambia, basta con ajustar el *WObj* sin recalibrar cada punto. Separar cada figura en procedimientos independientes mejora la legibilidad, el mantenimiento y las pruebas parciales.

---

**11. Fragmento representativo / pseudocódigo**

```rapid
loop:
  reset(DO_01, DO_02, Conveyor_FWD, Conveyor_INV)

  if DI_01=1 and DI_02=0 then
     set DO_01
     set Conveyor_FWD; wait(4); reset Conveyor_FWD
     Path_No_Golpear()
     Path_Pastel_Inicio()
     ejecutar todos los Paths de letras y motivos
     set Conveyor_FWD; wait(4); reset Conveyor_FWD
     Path_HOME()
     Path_HOME_IR()
     reset DO_01, DO_02

  elseif DI_01=0 and DI_02=1 then
     Path_HOME()
     Path_Mantenimiento()
     set DO_02
     waituntil DI_01=1 and DI_02=1

  endif
endloop
```

---

**. Validación frente a los requisitos del laboratorio**
Este programa cumple los requerimientos establecidos:

* Velocidades entre 100 y 1000 mm/s
* Precisión en zona *z1* para decoración
* Retorno con articulaciones en 0°
* Uso de dos entradas y dos salidas digitales
* Banda activa al inicio y al final
* Trayectorias referenciadas a un mismo *WorkObject* para máxima portabilidad


