# Silla de ruedas virtual controlada por EOG

Control de giro de una silla de ruedas de dos motores a partir de una señal electrooculográfica (EOG) horizontal. Mirada a la derecha: giro horario. Mirada a la izquierda: giro antihorario.

Proyecto universitario. Incluye el programa del Arduino Uno y una aplicación web para visualizar la señal y simular la silla.

## Contenido

| Archivo | Descripción |
|---|---|
| `eog_silla/eog_silla.ino` | Programa del Arduino Uno: lectura de A0 a 250 Hz, filtro notch de 50 Hz, detección de la mirada y salidas al puente H. |
| `silla_virtual.html` | Aplicación web autónoma (un solo archivo): traza de la señal, silla vista desde arriba y estado de los motores. |

## Cómo usarlo

1. Conectar la salida del circuito EOG (tras R5) a **A0** del Arduino, con 100 nF entre A0 y GND y masa común.
2. Abrir `eog_silla/eog_silla.ino` en el Arduino IDE y cargarlo en la placa.
3. Abrir `silla_virtual.html` en **Chrome o Edge de escritorio** (también se puede usar la versión publicada con GitHub Pages).
4. Presionar **Conectar Arduino**, elegir el puerto (115200 baudios) y esperar unos 10 s a que termine la calibración del reposo.
5. Sin hardware, presionar **Usar simulación** (botones o flechas del teclado).

Alternativa: Arduino IDE, Herramientas, Serial Plotter, a 115200 baudios.

## Parámetros principales

| Parámetro | Valor |
|---|---|
| Frecuencia de muestreo | 250 Hz |
| Ganancia total del circuito | 253 |
| Umbral de detección | 0,30 mV en los electrodos |
| Confirmación | 50 ms |
| Período refractario | 500 ms |

Los parámetros se cambian en la sección de configuración al inicio de `eog_silla.ino`.

## Comandos por el monitor serie

- `p`: parada de emergencia (bloquea las salidas)
- `r`: reanudar

## Seguridad

- El circuito **no tiene aislamiento galvánico**. Con electrodos colocados, no conectar el Arduino por USB a una computadora enchufada a la red: usar la notebook a batería o un aislador USB.
- Este programa es para simulación y pruebas de banco. Antes de mover una silla real hacen falta parada de emergencia física, finales de carrera y límite de velocidad.

## Autores

Bernay, Manuela		43883032
Saiki Alarcon, Esteban Nicolás	38330969
Soruco, José Augusto		44281813

10/11/2026
