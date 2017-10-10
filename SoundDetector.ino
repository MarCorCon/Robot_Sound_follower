/**
   Uso a los 2 micrófonos con los tiempos de llegada más cercanos para medir el ángulo.
   El tercer micrófono me sirve para saber si el sonido viene desde adelante o desde atrás.
   Se usa el angleOffset para corregir el ángulo dependiendo de los 2 micrófonos elegidos y si el sonido viene de adelante o desde atrás.

                   PI

                   m2


    -PI/2          O            PI/2

          m0                m1

                    0º

*/
short mic0 = 10;
short mic1 = 4;
short mic2 = 0;
const short led = 13;
const short tempSensor = 7; // termómetro usado para calcular la velocidad del sonido
const short umbral = 10; // diferencia en la lectura analógica para que se tome en consideración una lectura.
float ambient0 = 0;
float ambient1 = 0;
float ambient2 = 0;
double sound_speed;
const double mic_distance = 310000.0; // distancia entre micrófonos en um
double max_limit;
double min_limit;
// tiempo de llegada en us
unsigned long t0;
unsigned long t1;
unsigned long t2;
unsigned long timer; // el timer sirve para resetear las lecturas si pasa mucho tiempo después de la lectura de un micrófono y las sucesivas. Así se eliminan ciertos errores.
short count; // cuenta si se han realizado 3 lecturas

void setup() {
  Serial.begin(9600);
  blink();
  blink();
  pinMode(led, OUTPUT);
  set_sound_speed();
  max_limit = mic_distance / sound_speed;
  min_limit = mic_distance * cos(PI / 6) / sound_speed;
  count =  0;
  //  se mide el ruido ambiental en cada micrófono.
  int s0  = 0;
  int s1 = 0;
  int s2 = 0;
  for (int i = 0; i < 1000; ++i) {
    s0 += analogRead(mic0);
    s1 += analogRead(mic1);
    s2 += analogRead(mic2);
  }
  ambient0 = s0 / 1000.0 ;
  ambient1 = s1 / 1000.0 ;
  ambient2 = s2 / 1000.0 ;
  blink();
}
void loop() {
  if ( abs(analogRead(mic0) - ambient0) > umbral && t0 == 0) {
    t0 = micros();
    ++count;
    timer = t0 + max_limit;
  }
  if ( abs(analogRead(mic1) - ambient1) > umbral && t1 == 0) {
    t1 = micros();
    ++count;
    timer = t1 + max_limit;
  }
  if ( abs(analogRead(mic2) - ambient2) > umbral && t2 == 0) {
    t2 = micros();
    ++count;
    timer = t2 + max_limit;
  }
  if ( count && micros() > timer ) {
    reset();
  } else if (count == 3) {
    short frontRear = 1; // puede tener valores 1 o -1, para invertir el ángulo.
    // diferencias de tiempo de llegada del sonido entre los micrófonos
    int delay01 = t0 - t1;
    int delay12 =  t1 - t2;
    int delay20 = t2 - t0;
    if (abs(delay01) > 600 || abs(delay12) > 600 || abs(delay20) > 600) {
      short excluded; // micrófono excluído del computo del ángulo
      double angleOffset = 0;
      double angle = 0;
      // decido los 2 micrófonos que voy a utilizar:
      //micrófonos 0 y 1
      int recDelay = delay01;
      int minDist = abs(recDelay);
      excluded = 2;
      if (t2 < t0) {
        angleOffset = M_PI; // sonido desde atrás
        frontRear = -1;
      } else {
        angleOffset = 0; // sonido desde adelante
        frontRear = 1;
      }
      if (abs(delay12) < minDist) {
        recDelay = delay12;
        minDist = abs(delay12);
        excluded = 0;
        if (t0 < t1) {
          angleOffset = - M_PI / 3.0;
          frontRear = -1;
        } else {
          angleOffset = 2.0 * M_PI / 3.0;
          frontRear = 1;
        }
      }
      if (abs(delay20) < minDist) {
        recDelay = t2 - t0;
        // minDist = abs(t2 - t0);
        excluded = 1;
        if (t1 < t2) {
          angleOffset = M_PI / 3.0;
          frontRear = -1;
        } else {
          angleOffset = -2.0 * M_PI / 3.0;
          frontRear = 1;
        }
      }
      set_sound_speed();
      // Calculo el ángulo
      angle = asin((recDelay * sound_speed) / mic_distance) * frontRear + angleOffset;
      // rango del ángulo [ PI y -PI)
      if (angle > M_PI) {
        angle -= 2 * M_PI;
      } else if (angle <= - M_PI) {
        angle += 2 * M_PI;
      }
      Serial.print(angle);
    }
    delay(2000);
    reset();
  }
}

/**
   Parpadeo del led
*/
void blink() {
  for (short i = 0; i < 10; ++i) {
    digitalWrite(led, HIGH);
    delay(50);
    digitalWrite(led, LOW);
    delay(50);
  }
}

/**
   defino velocidad del sonido en función de la temperatura
*/
void set_sound_speed() {
  int tempSensorRead = analogRead(tempSensor);
  float tempVolt = 3.3 / 1023 * tempSensorRead;
  float temp = tempVolt * 100 - 50;
  sound_speed = sqrt( 1.4 * 286.9 * (273.15 + temp)); // sqrt( k * R * T)
}

/**
   reinicio los los tiempos y el contador
*/
void reset() {
  t0 = 0;
  t1 = 0;
  t2 = 0;
  count = 0;
}

