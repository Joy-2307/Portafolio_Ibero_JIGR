# Trabajos realizados Durante el primer semestre 
---
# Prácticas y Electrónica 
---
## Itervalos De un Led
[Video de Práctica](https://iberopuebla-my.sharepoint.com/:v:/g/personal/203563_iberopuebla_mx/Ef6GZFmCvP9Mv8V9N9eSk0cBmP1kUgcruvOYE_J4aUcBFQ?e=UAMhdn)

[calculo](recursos/imgs/555.png)
---

En este proyecto se implementó un circuito astable utilizando el temporizador integrado 555, con el objetivo de generar el parpadeo de un diodo LED dentro de un rango de entre 1 y 5 segundos. Para lograrlo se seleccionaron valores específicos de resistencias y un capacitor, además de un potenciómetro que permite ajustar el periodo de oscilación de manera práctica. El cálculo de los componentes se realizó a partir de las fórmulas teóricas del modo astable del 555, verificando que el tiempo de encendido y apagado del LED cumpliera con las condiciones establecidas. Finalmente, se integró la resistencia limitadora para proteger el LED y se comprobó el funcionamiento del circuito, obteniendo un parpadeo estable y regulable que confirma la correcta aplicación de los conceptos analizados en clase.

---

## Programando en ESP32
[Video de Práctica Y códigos](https://iberopuebla-my.sharepoint.com/:f:/g/personal/203563_iberopuebla_mx/EhkJHZAIqP9FguhG2vSfIz8BWP6sGHuCXWlVYsU_gA8Pzg?e=9fY8qf)
En esta práctica se programó un microcontrolador ESP32 para implementar dos formas de control de encendido y apagado de un LED, con el propósito de comprender tanto la interacción física mediante botones como la comunicación inalámbrica vía Bluetooth.

En el primer caso, se conectó un botón al ESP32 y se programó para que al presionarlo el LED se encendiera de manera inmediata, y al soltarlo, el LED se apagara. Este ejercicio permitió reforzar el manejo de entradas digitales y la lectura del estado de un pulsador, aplicando el concepto de lógica condicional en el código.

En el segundo caso, se utilizó la conectividad Bluetooth del ESP32 para recibir comandos enviados desde la terminal de un dispositivo externo. La lógica programada permitió que, al recibir el texto “on”, el LED se encendiera, y al recibir “off”, se apagara. Este ejercicio reforzó el uso de librerías de comunicación inalámbrica y la interpretación de cadenas recibidas por el puerto serie.

Durante la práctica se tomaron fotografías del código y videos del funcionamiento, que se adjuntan como evidencia del desarrollo. Los resultados obtenidos fueron exitosos en ambos casos, comprobando el correcto funcionamiento del hardware y la programación aplicada. Además, esta práctica permitió reconocer la versatilidad del ESP32 tanto para interacciones locales mediante botones como para aplicaciones remotas mediante comunicación Bluetooth.

---
