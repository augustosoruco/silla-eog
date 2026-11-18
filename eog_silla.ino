// ---------- Referencia del ADC ----------
const bool  USAR_AREF_EXTERNA = false;
const float VREF_ADC          = 5.0;     // voltios

// ---------- Centro de la señal (nivel de reposo de la salida) ----------
// Valor inicial. Con CENTRO_AUTOMATICO el programa lo mide solo al arrancar
const float CENTRO_V = 1.90;
const bool  CENTRO_AUTOMATICO = true;
const unsigned long ARRANQUE_MS = 8000;  // espera a que el pasaaltos se asiente 
const unsigned long CALIB_MS    = 2000;  // tiempo de promedio para medir el centro

// ---------- Ganancia del circuito ----------
// INA333 (11) x etapa 4 (23) x filtro Sallen-Key (1) = 253
const float GANANCIA_TOTAL = 253.0;

// ---------- Detección de la mirada ----------
// Umbral en milivolts EN LOS ELECTRODOS. 
const float UMBRAL_MV = 0.30;

// Signo: +1 si al mirar a la DERECHA la salida SUBE; -1 si BAJA.
// Depende de qué electrodo está en VIN+ y cuál en VIN-. 
const int SIGNO_DERECHA = +1;

// La señal tiene que quedar pasada del umbral este tiempo para contar. 
// Menos tiempo = la silla responde antes, pero un parpadeo puede mover la silla un
// instante.
const unsigned long CONFIRMACION_MS = 50;

// Modo de giro:
//   true  -> la silla gira MIENTRAS se mantiene la mirada (se detiene sola al
//            volver al centro, o cuando la señal cae por debajo de la mitad
//            del umbral, o a los MAX_GIRO_MS).
//   false -> cada mirada da un giro de duración fija (DURACION_GIRO_MS).
const bool          GIRO_MIENTRAS_MIRA = true;
const unsigned long MAX_GIRO_MS        = 3000;
const unsigned long DURACION_GIRO_MS   = 1000;

// Después de cada giro se ignora la señal un tiempo (cubre el escalón de
// vuelta) y luego se espera a que la señal regrese al reposo.
const unsigned long REFRACTARIO_MS   = 500;
const unsigned long REPOSO_MS        = 150;   // tiempo estable cerca del cero
const unsigned long ESPERA_MAX_MS    = 5000;  // si no vuelve, se recentra

// Seguimiento lento del reposo (deriva del electrodo): constante de tiempo.
const float TAU_DERIVA_S = 30.0;

// ---------- Puente H (L298N o similar) ----------
const int PIN_ENA = 5;    // PWM motor izquierdo
const int PIN_IN1 = 6;    // motor izquierdo, sentido adelante
const int PIN_IN2 = 7;    // motor izquierdo, sentido atrás
const int PIN_ENB = 10;   // PWM motor derecho
const int PIN_IN3 = 8;    // motor derecho, sentido adelante
const int PIN_IN4 = 9;    // motor derecho, sentido atrás
const int VELOCIDAD_PWM = 150;   // 0 a 255

// Giro HORARIO sobre el eje: motor izquierdo adelante, motor derecho atrás.
// Giro ANTIHORARIO: motor izquierdo atrás, motor derecho adelante.

// ---------- Habilitación (hombre muerto) ----------
// Si es true, los motores solo funcionan mientras se mantiene apretado un
// pulsador entre el pin 2 y GND. Recomendado para cualquier prueba con motores.
const bool USAR_HABILITACION = false;
const int  PIN_HABILITAR = 2;

// ---------- Muestreo y filtros ----------
const float FS_HZ = 250.0;
const int   N_PROMEDIO = 8;              // lecturas por muestra (~0,9 ms)
const float FREC_RED = 50.0;             // 50 Hz en Argentina
const float Q_NOTCH  = 10.0;
const int   N_SUAVIZADO = 4;             // promedio móvil (máximo 16): 4 muestras = 16 ms
// ---------- Otros ----------
const long BAUDIOS = 115200;
const int  PIN_ENTRADA_ADC = A0;
const int  LED_AVISO = LED_BUILTIN;      // titila al calibrar, fijo si satura
const int  TELEMETRIA_CADA = 5;          // 1 línea cada 5 muestras = 50 por segundo

// =====================================================================
//  VARIABLES INTERNAS
// =====================================================================

const unsigned long PERIODO_US = (unsigned long)(1000000.0 / FS_HZ);
unsigned long t_anterior;

float nb0, nb1, nb2, na1, na2;
float xAnt1 = 0, xAnt2 = 0, yAnt1 = 0, yAnt2 = 0;

float bufSuav[16];
int   idxSuav  = 0;
float sumaSuav = 0;

enum Estado { ARRANQUE, CALIBRANDO, LISTO, CONFIRMANDO, GIRANDO, ESPERA_REPOSO, PARADA };
Estado estado = ARRANQUE;

float centroV = CENTRO_V;
float sumaCalib = 0;
long  nCalib = 0;

unsigned long t_inicio, t_estado, t_reposo_ini;
int   candidato = 0;       // +1 derecha, -1 izquierda
int   orden = 0;           // +1 horario, -1 antihorario, 0 detenido
bool  bloqueado = false;   // parada de emergencia por el puerto serie
unsigned long t_saturado = 0;
int   contTelemetria = 0;
float kDeriva;

// =====================================================================
//  Motores
// =====================================================================
void motores(int o) {
  orden = o;
  if (o == 0) {
    analogWrite(PIN_ENA, 0);  analogWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
    return;
  }
  bool izqAdelante = (o > 0);          // horario: izquierdo adelante
  digitalWrite(PIN_IN1, izqAdelante ? HIGH : LOW);
  digitalWrite(PIN_IN2, izqAdelante ? LOW  : HIGH);
  digitalWrite(PIN_IN3, izqAdelante ? LOW  : HIGH);   // derecho al revés
  digitalWrite(PIN_IN4, izqAdelante ? HIGH : LOW);
  analogWrite(PIN_ENA, VELOCIDAD_PWM);
  analogWrite(PIN_ENB, VELOCIDAD_PWM);
}
bool habilitado() {
  if (bloqueado) return false;
  if (!USAR_HABILITACION) return true;
  return digitalRead(PIN_HABILITAR) == LOW;   // apretado = habilitado
}

void reiniciarFiltros() {
  xAnt1 = xAnt2 = yAnt1 = yAnt2 = 0;
  for (int i = 0; i < 16; i++) bufSuav[i] = 0;
  sumaSuav = 0;
  idxSuav = 0;
}
void cambiarEstado(Estado e, unsigned long ahora) {
  estado = e;
  t_estado = ahora;
}

// =====================================================================
//  setup()
// =====================================================================
void setup() {
  Serial.begin(BAUDIOS);
  pinMode(LED_AVISO, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  if (USAR_HABILITACION) pinMode(PIN_HABILITAR, INPUT_PULLUP);
  motores(0);

  if (USAR_AREF_EXTERNA) analogReference(EXTERNAL);
  else                   analogReference(DEFAULT);
  analogRead(PIN_ENTRADA_ADC);   // lectura descartada

  // Notch (biquad estándar)
  float w0N   = 2.0 * PI * FREC_RED / FS_HZ;
  float alfaN = sin(w0N) / (2.0 * Q_NOTCH);
  float den0  = 1.0 + alfaN;
  nb0 =  1.0           / den0;
  nb1 = -2.0 * cos(w0N) / den0;
  nb2 =  1.0           / den0;
  na1 = -2.0 * cos(w0N) / den0;
  na2 = (1.0 - alfaN)   / den0;

  kDeriva = (1.0 / FS_HZ) / TAU_DERIVA_S;

  t_anterior = micros();
  t_inicio = millis();
  cambiarEstado(CENTRO_AUTOMATICO ? ARRANQUE : LISTO, t_inicio);
  Serial.println("# EOG silla: arrancando");
}

// =====================================================================
//  Comandos por el puerto serie
// =====================================================================
void leerComandos(unsigned long ahora) {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'p' || c == 'P') {
      bloqueado = true;
      motores(0);
      cambiarEstado(PARADA, ahora);
      Serial.println("# PARADA DE EMERGENCIA");
    } else if (c == 'r' || c == 'R') {
      bloqueado = false;
      cambiarEstado(ESPERA_REPOSO, ahora);   // vuelve esperando el reposo
      t_reposo_ini = 0;
      Serial.println("# REANUDADO");
    }
  }
}

// =====================================================================
//  loop()
// =====================================================================
void loop() {
  if (micros() - t_anterior < PERIODO_US) return;
  t_anterior += PERIODO_US;

  unsigned long ahora = millis();
  leerComandos(ahora);

  // ---- 1) Leer A0 y promediar ----
  long sumaLect = 0;
  for (int i = 0; i < N_PROMEDIO; i++) sumaLect += analogRead(PIN_ENTRADA_ADC);
  float cuentas = (float)sumaLect / N_PROMEDIO;

  // Saturación: señal pegada a 0 o al tope del ADC
  bool saturada = (cuentas < 5 || cuentas > 1018);
  if (saturada) {
    if (t_saturado == 0) t_saturado = ahora;
  } else {
    t_saturado = 0;
  }
  digitalWrite(LED_AVISO, saturada ? HIGH : LOW);

  // ---- 2) Cuentas a voltios ----
  float volts = cuentas * VREF_ADC / 1024.0;

  // ---- 3) Quitar el centro, notch y promedio móvil ----
  float x = volts - centroV;
  float y = nb0 * x + nb1 * xAnt1 + nb2 * xAnt2 - na1 * yAnt1 - na2 * yAnt2;
  xAnt2 = xAnt1;  xAnt1 = x;
  yAnt2 = yAnt1;  yAnt1 = y;

  sumaSuav -= bufSuav[idxSuav];
  bufSuav[idxSuav] = y;
  sumaSuav += y;
  idxSuav = (idxSuav + 1) % N_SUAVIZADO;
  float filtrada = sumaSuav / N_SUAVIZADO;

  // ---- 4) Milivolts en los electrodos ----
  float mV = filtrada * 1000.0 / GANANCIA_TOTAL;
  float s  = mV * SIGNO_DERECHA;     // s > 0: mirada a la derecha

  // Saturación sostenida: parar y esperar
  if (t_saturado != 0 && (ahora - t_saturado) > 200 && estado != PARADA) {
    if (orden != 0) motores(0);
    if (estado == GIRANDO || estado == CONFIRMANDO) {
      cambiarEstado(ESPERA_REPOSO, ahora);
      t_reposo_ini = 0;
    }
  }

  // ---- 5) Máquina de estados ----
  switch (estado) {

    case ARRANQUE:
      if (ahora - t_estado >= ARRANQUE_MS) {
        sumaCalib = 0; nCalib = 0;
        cambiarEstado(CALIBRANDO, ahora);
      }
      digitalWrite(LED_AVISO, ((ahora / 250) % 2) ? HIGH : LOW);   // titila
      break;

    case CALIBRANDO:
      sumaCalib += volts; nCalib++;
      if (ahora - t_estado >= CALIB_MS && nCalib > 0) {
        centroV = sumaCalib / nCalib;
        reiniciarFiltros();
        cambiarEstado(LISTO, ahora);
        Serial.print("# Centro medido: ");
        Serial.print(centroV, 3);
        Serial.println(" V");
      }
      digitalWrite(LED_AVISO, ((ahora / 125) % 2) ? HIGH : LOW);
      break;

    case LISTO:
      // El reposo sigue despacio la deriva del electrodo
      if (fabs(mV) < UMBRAL_MV * 0.5) centroV += (volts - centroV) * kDeriva;
      if (s > UMBRAL_MV)       { candidato = +1; cambiarEstado(CONFIRMANDO, ahora); }
      else if (s < -UMBRAL_MV) { candidato = -1; cambiarEstado(CONFIRMANDO, ahora); }
      break;

    case CONFIRMANDO: {
      bool sigue = (candidato > 0) ? (s > UMBRAL_MV) : (s < -UMBRAL_MV);
      if (!sigue) {
        cambiarEstado(LISTO, ahora);                    // pico corto: descartado
      } else if (ahora - t_estado >= CONFIRMACION_MS) {
        if (habilitado()) {
          motores(candidato);                           // +1 horario, -1 antihorario
          cambiarEstado(GIRANDO, ahora);
          Serial.println(candidato > 0 ? "# GIRO HORARIO" : "# GIRO ANTIHORARIO");
        } else {
          cambiarEstado(ESPERA_REPOSO, ahora);          // sin habilitación: no gira
          t_reposo_ini = 0;
          Serial.println("# Mirada detectada, pero las salidas estan deshabilitadas");
        }
      }
      break;
    }

    case GIRANDO: {
      bool terminar = !habilitado();
      if (GIRO_MIENTRAS_MIRA) {
        // sigue girando mientras la mirada se mantiene del mismo lado
        bool mantiene = (candidato > 0) ? (s > UMBRAL_MV * 0.5) : (s < -UMBRAL_MV * 0.5);
        if (!mantiene || (ahora - t_estado >= MAX_GIRO_MS)) terminar = true;
      } else {
        if (ahora - t_estado >= DURACION_GIRO_MS) terminar = true;
      }
      if (terminar) {
        motores(0);
        cambiarEstado(ESPERA_REPOSO, ahora);
        t_reposo_ini = 0;
      }
      break;
    }

    case ESPERA_REPOSO:
      if (ahora - t_estado >= REFRACTARIO_MS) {
        if (fabs(mV) < UMBRAL_MV * 0.5) {
          if (t_reposo_ini == 0) t_reposo_ini = ahora;
          if (ahora - t_reposo_ini >= REPOSO_MS) cambiarEstado(LISTO, ahora);
        } else {
          t_reposo_ini = 0;
        }
        if (ahora - t_estado >= REFRACTARIO_MS + ESPERA_MAX_MS) {
          centroV = volts;                              // recentrar
          reiniciarFiltros();
          cambiarEstado(LISTO, ahora);
        }
      }
      break;

    case PARADA:
      motores(0);
      break;
  }

  // ---- 6) Telemetría ----
  if (++contTelemetria >= TELEMETRIA_CADA) {
    contTelemetria = 0;
    Serial.print("Senal_mV:");   Serial.print(mV, 3);
    Serial.print(",Umbral:");    Serial.print(UMBRAL_MV, 2);
    Serial.print(",Umbral_neg:"); Serial.print(-UMBRAL_MV, 2);
    Serial.print(",Orden:");     Serial.println(orden);
  }
}

