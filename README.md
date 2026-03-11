# Control de Velocidad de Motor DC con micro-ROS y ROS2

## Descripción

Este proyecto implementa un sistema de **control de velocidad para un motor DC con encoder** utilizando **micro-ROS en un ESP32** y **ROS2 en una computadora**.

El sistema permite:

- Recibir una referencia de velocidad desde ROS2
- Ejecutar un **controlador PID incremental en el microcontrolador**
- Medir la velocidad del motor mediante un **encoder incremental**
- Publicar la velocidad medida nuevamente a ROS2
- Generar señales de prueba (senoidal, cuadrada, triangular y step)
- Visualizar y analizar la respuesta del sistema en tiempo real

El objetivo es evaluar el desempeño del controlador ante diferentes señales de referencia.

---

# Arquitectura del Sistema

El sistema se divide en dos partes principales.

### Computadora (ROS2)

- Generación de señales de referencia
- Visualización de señales
- Registro de datos para análisis

### ESP32 (micro-ROS)

- Lectura del encoder
- Cálculo de velocidad
- Ejecución del controlador PID
- Generación de PWM para el motor

---

# Diagrama del Sistema

```
SetPoint Node (ROS2)
        │
        │  /set_point
        ▼
ESP32 (micro-ROS)
        │
        │  Control PID
        │
        ▼
Motor DC + Encoder
        │
        │
        ▼
ESP32
        │
        │  /motor_output
        ▼
ROS2 PC
        │
        ▼
rqt_plot / Data Logging
```

---

# Tópicos ROS2

| Tópico | Tipo | Descripción |
|------|------|------|
| `/set_point` | `std_msgs/Float32` | Referencia de velocidad normalizada |
| `/motor_output` | `std_msgs/Float32` | Velocidad medida normalizada |

Las señales están normalizadas en el rango:

```
-1 ≤ señal ≤ 1
```

donde:

```
motor_output = rpm / RPM_MAX
```

---

# Hardware Utilizado

- ESP32
- Motor DC con caja reductora
- Encoder incremental
- Driver de motor (puente H)
- Fuente de alimentación externa

### Conexión de Pines

| Señal | Pin ESP32 |
|------|------|
| Encoder A | GPIO 14 |
| Encoder B | GPIO 13 |
| PWM Motor | GPIO 27 |
| Dirección IN1 | GPIO 25 |
| Dirección IN2 | GPIO 26 |

---

# Medición de Velocidad

La velocidad del motor se calcula a partir del conteo de pulsos del encoder:

```
rpm = (pulseCount * 60) / (PULSES_PER_REV * Ts)
```

donde:

```
PULSES_PER_REV = 495
Ts = 0.1 s
```

Posteriormente se aplica un filtro exponencial:

```
rpm_filt = α * rpm_raw + (1 - α) * rpm_prev
```

---

# Controlador Implementado

Se implementó un **PID incremental discreto**.

```
u(k) = u(k-1)
       + Kp (e(k) - e(k-1))
       + Ki Ts e(k)
       + Kd/Ts (e(k) - 2e(k-1) + e(k-2))
```

donde:

```
e(k) = referencia - velocidad
```

Ganancias utilizadas:

```
Kp = 1.2
Ki = 0.6
Kd = 0
```

La señal de control se satura entre:

```
0 ≤ u ≤ 1
```

y posteriormente se convierte a PWM:

```
PWM = u * 255
```

---

# Generación de Señales de Prueba

Se implementó un nodo ROS2 en Python que genera distintas señales de referencia.

Tipos de señal disponibles:

- `sine`
- `square`
- `triangle`
- `step`

El tipo de señal puede cambiarse dinámicamente mediante parámetros.

Ejemplo:

```
ros2 param set /set_point_node signal_type sine
```

---

# Ejecución del Sistema

## 1. Ejecutar el micro-ROS agent

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

---

## 2. Ejecutar el nodo generador de señales

```
ros2 run motor_control set_point_node
```

---

## 3. Visualizar señales

```
rqt_plot
```

Graficar los tópicos:

```
/set_point/data
/motor_output/data
```

---

# Registro de Datos

Durante las pruebas se almacenan los datos en un archivo CSV con el formato:

```
time, setpoint, motor_output
```

Esto permite analizar posteriormente el desempeño del controlador.

---

# Consideraciones de Control

El sistema utiliza una frecuencia de control de:

```
10 Hz
```

La dinámica del motor se encuentra aproximadamente entre:

```
2 – 5 Hz
```

por lo que se cumple la regla práctica:

```
f_control ≥ 10 × f_dinamica
```

---

# Resultados Esperados

El sistema permite analizar:

- Seguimiento de referencia
- Error estacionario
- Respuesta a señales sinusoidales
- Respuesta a cambios bruscos (step y square)

Las respuestas se pueden observar en tiempo real mediante `rqt_plot` o mediante análisis posterior en Python/MATLAB.

---

# Trabajo Futuro

Posibles mejoras del sistema:

- Incrementar la frecuencia de control
- Implementar control con **feedforward**
- Implementar **anti-windup** en el PID
- Utilizar cuadratura completa del encoder (4x)
- Registrar datos con `rosbag2`

---

# Autor

José Eduardo Sánchez Martínez

Proyecto desarrollado como parte de un reto de control utilizando **ROS2 y micro-ROS**.
