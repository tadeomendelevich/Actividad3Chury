<div align="center">

# 🚗 Actividad 3 — Robot Autónomo con mbed

**Control de motores, encoders y navegación autónoma sobre plataforma ARM mbed**

[![Platform](https://img.shields.io/badge/Plataforma-mbed%20(Cortex--M)-000000?style=for-the-badge&logo=arm&logoColor=white)](https://os.mbed.com/)
[![Language](https://img.shields.io/badge/Lenguaje-C%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![WiFi](https://img.shields.io/badge/WiFi-ESP8266-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)
[![Protocol](https://img.shields.io/badge/Protocolo-UNER-0078D7?style=for-the-badge)]()

*Universidad Nacional de Entre Ríos (UNER) · Mecánica Racional — Actividad 3*

</div>

---

## 📋 Descripción

Firmware para un robot autónomo desarrollado sobre la plataforma **ARM mbed** como parte de la Actividad 3 de Mecánica Racional. El sistema implementa cuatro modos de navegación con control de motores por PWM, encoders de velocidad y comunicación serial/WiFi usando el protocolo UNER.

La arquitectura es completamente **no bloqueante**: ninguna tarea utiliza `wait()` o delays activos — la sincronización se maneja con flags de interrupción y timestamps.

---

## ✨ Modos de operación

| Modo | Descripción |
|---|---|
| **Modo 1** | Avance con encoder simple — movimiento hacia adelante controlado por un solo encoder |
| **Modo 2** | Avance sincronizado — corrección proporcional de velocidad entre dos motores para trayectoria recta |
| **Modo 3** | Rotación por ángulo — giro preciso en grados usando odometría de encoders duales |
| **Modo 4** | Trayectoria en L — secuencia: avanzar → girar 88° → avanzar en nueva dirección |

---

## 🔧 Hardware

| Componente | Función | Pin |
|---|---|---|
| **2× Motor DC** | Tracción con control PWM diferencial | PWM + DIR |
| **Servo motor** | Orientación / radar | PA_8 |
| **HC-SR04** | Sensor de distancia ultrasónico | Trigger: PB_13 / Echo: PB_12 |
| **IR × 3** | Sensores de línea — izquierda, centro, derecha | PA_0, PA_1, PA_2 |
| **Encoders × 2** | Medición de velocidad y odometría | PB_8, PB_9 |
| **4 LEDs + 4 botones** | I/O de usuario | GPIO |
| **ESP8266** | Comunicación WiFi UDP | UART |

---

## 🏗️ Arquitectura del firmware

```
Loop principal — arquitectura cooperativa sin RTOS
│
├── serialTask()      ──► Parser UNER (USART + WiFi) ──► comandos
│
├── distanceTask()    ──► HC-SR04 con ISR eco ──► distancia_cm
│
├── irSensorsTask()   ──► IR × 3 ──► posición sobre línea
│
├── speedTask()       ──► encoders por INT ──► RPM + odometría
│
├── servoTask()       ──► PWM 50 Hz ──► ángulo
│
└── Máquina de estados
        ├── Modo 1 — encoder simple ──► avance frontal
        ├── Modo 2 — encoders duales ──► corrección proporcional
        ├── Modo 3 — rotación ──► giros controlados por odometría
        └── Modo 4 — trayectoria en L ──► secuencia multietapa
```

---

## 📡 Protocolo de comunicación UNER

```
┌────────┬────────────────┬──────────┐
│ Header │    Payload     │ Checksum │
│ "UNER" │ Datos variables│  1 byte  │
└────────┴────────────────┴──────────┘
```

- **Canal 1:** USART (USB-serial)  
- **Canal 2:** WiFi UDP (módulo ESP8266)

Ambos canales son equivalentes — el mismo comando enviado por serial o WiFi produce el mismo comportamiento.

---

## 📶 Conectividad WiFi

El módulo ESP8266 se configura automáticamente al arrancar:

1. Conexión a la red con SSID/password almacenados en `wifi.cpp`
2. Apertura de socket UDP
3. Envío periódico de señal **ALIVE** para mantener la sesión
4. Recepción de comandos de control en tiempo real

---

## 📁 Estructura del proyecto

```
Actividad3Chury/
├── main.cpp          # Lógica principal, máquina de estados, loop cooperativo
├── config.h          # Pines, constantes de hardware y configuración general
├── debounce.cpp/h    # Filtro anti-rebote para botones
├── myDelay.cpp/h     # Utilidades de temporización sin bloqueo
├── wifi.cpp/h        # Driver ESP8266 — AT commands, UDP, reconexión
├── util.h            # Funciones auxiliares
└── Makefile          # Build con ARM mbed toolchain
```

---

## 🚀 Compilación

```bash
# Con mbed online compiler o localmente con ARM GCC:
make
```

---

<div align="center">

**Autor:** [Tadeo Mendelevich](https://github.com/tadeomendelevich)  
*Ingeniería en Mecatrónica — UNER*

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Tadeo_Mendelevich-0A66C2?style=flat&logo=linkedin)](https://www.linkedin.com/in/tadeo-mendelevich/)
[![GitHub](https://img.shields.io/badge/GitHub-tadeomendelevich-181717?style=flat&logo=github)](https://github.com/tadeomendelevich)

</div>
