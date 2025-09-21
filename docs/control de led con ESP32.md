# Control de LED con ESP32 mediante botón y Bluetooth  

## 1) Resumen  
**Nombre del proyecto:** Control de LED con ESP32  
**Equipo / Autor(es):** José Ismael Guerrero Román y Gerardo Esquivel De Luna  
**Curso / Asignatura:** Introducción a la Mecatrónica
**Fecha:** 19/09/25
**Descripción breve:** Se programó un microcontrolador ESP32 para controlar un LED en dos modos: encendido/apagado con un botón físico y encendido/apagado mediante comandos enviados por Bluetooth desde la terminal.  

---

## 2) Objetivos  
**General:**  
Implementar el control de un LED con ESP32 utilizando tanto entradas físicas (botón) como comunicación inalámbrica (Bluetooth).  

**Específicos:**  
- OE1: Programar el ESP32 para que al presionar un botón se encienda un LED y al soltarlo se apague.  
- OE2: Configurar la comunicación Bluetooth del ESP32 para recibir comandos desde la terminal.  
- OE3: Programar la lógica para que el LED se encienda con el comando `"on"` y se apague con el comando `"off"`.  
- OE4: Documentar el código y resultados con fotografías y videos como evidencia.  

---

## 3) Alcance y Exclusiones  
**Incluye:**  
- Programación del ESP32 en Arduino IDE (o entorno equivalente).  
- Implementación de control físico mediante botón.  
- Implementación de control remoto mediante Bluetooth con comandos `"on"` y `"off"`.  
- Evidencias en fotos y videos del funcionamiento.  

**No incluye:**  
- Control de más de un LED u otros actuadores.  
- Desarrollo de aplicaciones móviles dedicadas (solo terminal).  
- Integración con plataformas IoT externas.  

---

## 4) Requisitos  
**Software:**  
- Arduino IDE con soporte para ESP32.  
- Librerías de Bluetooth para ESP32.  
- Monitor serie o aplicación de terminal Bluetooth.  

**Hardware:**  
- ESP32.  
- LED + resistencia limitadora (330 Ω a 820 Ω).  
- Pulsador (botón).  
- Protoboard y cables de conexión.  
- Fuente de alimentación USB (5 V).  

**Conocimientos previos:**  
- Programación básica en Arduino/ESP32.  
- Conceptos de entradas digitales (lectura de botones).  
- Manejo de salidas digitales (encendido de LED).  
- Comunicación básica vía Bluetooth.  

