# Laboratorios de Robótica

´´´ mermaid
flowchart TD

    A[Acción del usuario en la interfaz de control]

    A --> B[Control por sliders o valores de las articulaciones]
    A --> C[Botones de poses de laboratorio y poses personalizadas]
    A --> D[Botón HOME]
    A --> E[Comando en espacio de la tarea XYZ y orientación del TCP]
    A --> F[Botón de parada de emergencia]

    B --> G[Calcular los ángulos objetivo de cada articulación en grados]
    C --> G
    D --> G

    %% Flujo físico: brazo real
    subgraph FISICO [Flujo físico del brazo real]
        G --> H[Convertir esos ángulos a valores de ticks Dynamixel]
        H --> I[Enviar los ticks por el puerto serie a la cadena de servos]
        I --> J[Los servos Dynamixel giran y el brazo físico cambia de postura]
    end

    %% Flujo virtual: modelo en RViz
    subgraph VIRTUAL [Flujo virtual en RViz]
        G --> K[Actualizar el modelo interno de ángulos en el nodo PincherController]
        K --> L[Enviar un mensaje JointState con esos ángulos para que ROS conozca la postura actual]
        L --> M[Si RViz está abierto, actualiza el modelo tridimensional del PhantomX con la misma postura]
    end

    %% Control en espacio de la tarea
    subgraph ESPACIO_TAREA [Control en espacio de la tarea]
        E --> N[Tomar X Y Z y orientación que el usuario escribe en la pestaña Espacio de la tarea]
        N --> O[Crear un mensaje PoseCommand con esos datos]
        O --> P[Publicar el mensaje PoseCommand hacia MoveIt]
        P --> Q[MoveIt planifica una trayectoria con esa pose y la ejecuta en el robot o en la simulación]
    end

    %% Parada de emergencia
    subgraph EMERGENCIA [Seguridad del sistema]
        F --> R[Desactivar el torque de todos los motores al pulsar Parada de emergencia] 


    ´´´
        R --> S[Marcar el sistema en estado de emergencia y bloquear nuevos movimientos hasta reactivar]
    end
