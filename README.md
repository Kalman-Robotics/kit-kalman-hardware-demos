# Kit Kalman Hardware Demos

Repositorio de ejemplos y demostraciones de hardware para el Kit Kalman. Este proyecto contiene programas de prueba individuales para cada componente del kit de robótica.

- [Kit Kalman Hardware Demos](#kit-kalman-hardware-demos)
  - [Instrucciones](#instrucciones)
  - [Programas de Ejemplo](#programas-de-ejemplo)
    - [**1\_led\_rgb.cpp** - LED RGB](#1_led_rgbcpp---led-rgb)
    - [**3\_motor\_basic.cpp** - Control Básico de Motor](#3_motor_basiccpp---control-básico-de-motor)
    - [**4\_encoder\_basic.cpp** - Lectura de Encoder](#4_encoder_basiccpp---lectura-de-encoder)
    - [**4.5\_motor\_encoder.cpp** - Motor con Encoder](#45_motor_encodercpp---motor-con-encoder)
    - [**5\_lidar.cpp** - LiDAR LD19](#5_lidarcpp---lidar-ld19)
    - [**6\_imu.cpp** - IMU MPU6500 con Ángulos de Euler](#6_imucpp---imu-mpu6500-con-ángulos-de-euler)
  - [Conceptos Teóricos](#conceptos-teóricos)
    - [PWM (Pulse Width Modulation)](#pwm-pulse-width-modulation)
    - [Encoder en Cuadratura](#encoder-en-cuadratura)
    - [Filtro Complementario](#filtro-complementario)
    - [Comunicación I2C](#comunicación-i2c)


## Instrucciones

Este workspace contiene ejemplos independientes para probar y entender cada componente de hardware del kit. El archivo `src/main.cpp` está vacío intencionalmente, ya que el flujo de trabajo consiste en copiar el código de los programas numerados a `main.cpp` para compilar y cargar a la placa.

**Plataforma:** ESP32-S3-DevKitC-1  
**Framework:** Arduino  
**IDE:** PlatformIO


## Programas de Ejemplo

### **1_led_rgb.cpp** - LED RGB
**Descripción:**  
Prueba básica del LED RGB NeoPixel integrado en la placa. Cicla entre los colores rojo, verde y azul con un intervalo de 1 segundo.

**Hardware:**
- **Pin:** GPIO 48
- **Tipo:** Adafruit NeoPixel (GRBW)

**Funcionalidad:**
- Inicializa el LED con brillo bajo (5/255)
- Cambia colores en secuencia: Rojo → Verde → Azul
- Útil para verificar que la placa funciona correctamente


### **3_motor_basic.cpp** - Control Básico de Motor
**Descripción:**  
Control de motor DC N20 con driver usando señal PWM dinámica. Siguiente el datasheet se realiza el control de dirección, y mediante un pin se aplica PWM para variar la velocidad.

**Hardware:**
- **Pines:** GPIO 4 (IN1), GPIO 5 (IN2)
- **Motor:** N20 con reducción 150:1, RPM máximo 200

**Funcionalidad:**
- Control de dirección (adelante/atrás) y velocidad
- Estados: Forward (5s) → Stop (1s) → Backward (5s) → Stop (1s)
- Hard brake: ambos pines HIGH
- Soft brake: ambos pines LOW
- PWM de 10 bits (0-1023) a 20 kHz


### **4_encoder_basic.cpp** - Lectura de Encoder
**Descripción:**  
Lectura de encoder incremental en cuadratura para medir posición, velocidad (RPM) y ángulo de rotación del motor.

**Hardware:**
- **Pines:** GPIO 6 (Canal A), GPIO 7 (Canal B)
- **Encoder:** 7 PPR base × 150 reducción = 1050 PPR efectivos

**Funcionalidad:**
- Detección de dirección mediante interrupciones
- Conteo en cuadratura (4x resolución)
- Cálculos en tiempo real:
  - Contador de pulsos acumulativo
  - RPM (revoluciones por minuto)
  - Ángulo en grados
  - Delta de pulsos entre lecturas


### **4.5_motor_encoder.cpp** - Motor con Encoder
**Descripción:**  
Combina control de motor y lectura de encoder. Permite verificar el movimiento real del motor mientras se controla su velocidad y dirección.

**Hardware:**
- **Motor:** GPIO 4 (IN1), GPIO 5 (IN2)
- **Encoder:** GPIO 6 (Canal A), GPIO 7 (Canal B)

**Funcionalidad:**
- Control de motor con PWM dinámico (igual que `3_motor_basic.cpp`)
- Lectura simultánea de encoder (igual que `4_encoder_basic.cpp`)
- Estados: Forward (5s) → Stop (1s) → Backward (5s) → Stop (1s)
- Monitoreo en tiempo real de posición y velocidad


### **5_lidar.cpp** - LiDAR LD19
**Descripción:**  
Lectura de datos del sensor LiDAR LD19 mediante comunicación serial. Procesa paquetes de 12 puntos con distancia, intensidad y ángulo.

**Hardware:**
- **Puerto Serial:** Serial2
- **Pin RX:** GPIO 37
- **Baud Rate:** 230400 bps

**Protocolo del paquete:**
- **Encabezado:** `0x54 0x2C`
- **Datos:** 12 puntos por paquete
- **Por punto:**
  - Distancia: 2 bytes (mm)
  - Intensidad: 1 byte (0-255)
  - Ángulo: Interpolado entre ángulo inicial y final
- **Verificación:** CRC-8

**Funcionalidad:**
- Búsqueda automática del encabezado del paquete
- Validación de integridad con CRC
- Interpolación de ángulos para los 12 puntos
- Salida formateada de cada punto


### **6_imu.cpp** - IMU MPU6500 con Ángulos de Euler
**Descripción:**  
Lectura y procesamiento del sensor inercial MPU6500 (acelerómetro + giroscopio) para calcular ángulos de orientación (roll, pitch, yaw) usando un filtro complementario.

**Hardware:**
- **Comunicación:** I2C
- **Pines:** GPIO 48 (SDA), GPIO 47 (SCL)
- **Dirección I2C:** 0x68
- **Frecuencia:** 400 kHz

**Sensores:**
- **Acelerómetro:** ±2g (16384 LSB/g)
- **Giroscopio:** ±250°/s (131 LSB/°/s)

**Funcionalidad:**
1. **Lectura de datos crudos** (16 bits por eje)
2. **Conversión a unidades físicas** (g y °/s)
3. **Cálculo de ángulos del acelerómetro:**
   - Roll: `atan2(ay, az) × 180/π`
   - Pitch: `atan2(-ax, √(ay² + az²)) × 180/π`
4. **Integración del giroscopio:** `ángulo += velocidad_angular × dt`
5. **Filtro complementario:** `ángulo = 0.96×(ángulo + gyro_delta) + 0.04×accel_ángulo`

**Nota importante:**  
El yaw (rotación en Z) deriva con el tiempo ya que el MPU6500 no tiene magnetómetro. Para yaw preciso se requiere un sensor de 9 ejes (9-DOF).


## Conceptos Teóricos

### PWM (Pulse Width Modulation)
- **Frecuencia:** 20 kHz (inaudible)
- **Resolución:** 10 bits (0-1023)
- **Duty Cycle:** Porcentaje de tiempo en HIGH
- **Canales ESP32:** 16 canales independientes

### Encoder en Cuadratura
- **Señales:** A y B desfasadas 90°
- **Resolución:** 4× conteo por detección de flancos
- **Dirección:** Determinada por la relación de fase A-B

### Filtro Complementario
- **Alpha = 0.96:** 96% giroscopio, 4% acelerómetro
- **Propósito:** Combinar respuesta rápida (gyro) con estabilidad (accel)
- **Resultado:** Ángulos precisos sin deriva ni ruido

### Comunicación I2C
- **Velocidad:** 400 kHz (modo rápido)
- **Ventaja:** Solo 2 pines para múltiples dispositivos
- **Protocolo:** Master-Slave con direcciones de 7 bits
