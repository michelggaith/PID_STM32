# PID_STM32
Este proyecto fue diseñado para el control mediante un PID de una planta de temperatura diseñada con un MOSFET TIP122
Inicialmente se encuentra la respuesta al escalon y posteriormente se calculan las constantes KP, Ki y KD
Se pueden modificar las constantes dependiendo de la planta utilizada.
Consejos del profesor:
1. Enviar el SETPOINT por puerto serie (Se recibe pero hay que reenviarlo para conservar "Memoria")
2. Los datos tienen que ser enviados utilizando una union y en codificacion HEXA
3. El programa funciona correctamente, puede ser utilizado para cualquier planta cambiando las constantes
4. Discretizacion utilizada mediante tustin
5. En mi repositorio encontraran la UI correspondiente a este proyecto.