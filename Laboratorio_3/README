# Laboratorio No. 03
# Robótica Industrial - Análisis y Operación del Manipulador EPSON T3-401S.

* Omar David Acosta Zambrano
* Edgar Esteban Erazo Lagos

**Introducción**

Los manipuladores industriales son sistemas clave en la automatización por su capacidad de ejecutar tareas con precisión y control en entornos productivos. Sin embargo, su desempeño depende en gran medida de sus configuraciones iniciales, modos de operación y del software que los gestiona.
En este laboratorio se analiza el manipulador EPSON T3-401S mediante el uso de EPSON RC+ 7.0, con el objetivo de comprender sus configuraciones home, modos de movimiento manual, niveles de velocidad y manejo de trayectorias dentro de su espacio de trabajo. De igual forma, se diseña un gripper neumático por vacío para la manipulación segura de huevos y se implementa una rutina que sigue el patrón de movimiento del caballo en ajedrez, lo cual exige una planificación estructurada de la trayectoria.
Finalmente, se comparan las herramientas EPSON RC+ 7.0, RoboDK y RobotStudio, identificando sus diferencias y alcances en simulación, programación y control de manipuladores industriales.

**Cuadro comparativo con especificaciones técnicas del EPSON T3-401S, Motoman MH6 y ABB IRB140**

| Ítem                                   | **EPSON T3-401S**                                | **Motoman MH6 (DX100/DX200)**                     | **ABB IRB 140 (IRC5)**                          |
|----------------------------------------|--------------------------------------------------|--------------------------------------------------|------------------------------------------------|
| **Tipo de manipulador**                | SCARA industrial                                  | Robot articulado industrial                      | Robot articulado industrial                    |
| **Grados de libertad / ejes**          | 4 ejes (SCARA)                                    | 6 ejes                                           | 6 ejes                                         |
| **Carga máxima (payload)**             | **3 kg**                                          | **6 kg**                                         | **6 kg**                                       |
| **Alcance máximo**                     | **400 mm**                                        | **1 422 mm**                                     | **810 mm**                                     |
| **Repetibilidad**                      | **±0.01 mm**                                      | **±0.08 mm**                                     | **±0.03 mm**                                   |
| **Velocidad máxima del TCP**           | Hasta **4400 mm/s**                               | Dependiente por eje (hasta 610°/s en J6)         | Dependiente por eje (hasta 450°/s en J6)       |
| **Tipo de movimiento**                 | Plano X-Y con eje Z vertical                      | Movimiento espacial 3D completo                 | Movimiento espacial 3D completo               |
| **Configuración estructural**          | SCARA rígido, alta precisión                      | Serial articulado                                | Serial articulado                              |
| **Montaje**                            | Sobre mesa o estructura fija                      | Piso, pared, techo, invertido                    | Piso, pared e invertido                        |
| **Aplicaciones típicas**               | Pick and place, ensamblaje fino                   | Manipulación, soldadura, carga/descarga          | Manipulación, ensamblaje, alta precisión      |
| **Espacio de trabajo**                 | Zona circular plana limitada                      | Volumen tridimensional amplio                    | Volumen tridimensional compacto               |
| **Software de control**                | EPSON RC+ 7.0                                    | MotoSim / RoboDK / DX100-DX200                   | RobotStudio (RAPID)                            |
| **Precisión en tareas repetitivas**    | Muy alta                                          | Alta                                             | Muy alta                                       |
| **Enfoque principal**                  | Alta velocidad y repetibilidad en plano           | Versatilidad industrial                          | Precisión y flexibilidad espacial             |

## Análisis técnico comparativo

El manipulador EPSON T3-401S, al ser de tipo SCARA, está optimizado para movimientos rápidos y precisos en el plano XY, lo que lo hace especialmente eficiente en tareas de ensamblaje ligero y manipulación repetitiva. Su estructura rígida le permite alcanzar una alta repetibilidad, característica clave para aplicaciones donde la precisión dimensional es prioritaria.
En contraste, el Motoman MH6 y el ABB IRB140 son robots articulados de seis ejes, capaces de operar en entornos tridimensionales con mayor libertad geométrica. Esto les permite ejecutar tareas que requieren orientación compleja del efector final, aunque con menor especialización en desplazamientos planos rápidos.
Mientras el EPSON prioriza velocidad y exactitud en trayectorias cortas y repetitivas, los otros dos manipuladores se enfocan en versatilidad espacial y mayor rango de aplicaciones industriales. Esta diferencia justifica su selección según el tipo de proceso, siendo el T3-401S el más adecuado para operaciones controladas de pick and place como las desarrolladas en este laboratorio.

