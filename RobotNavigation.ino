#include <Magabot.h>
#include <Servo.h>
#define WORLD_SIZE 5.0 // tamaño del mapa
#define MAP_PRECISION 0.1 // precisión del mapa
#define MAP_SIZE round(WORLD_SIZE/MAP_PRECISION) // lado del mapa
#define ROI 0.4 // radio de influencia de los obstáculos
#define KFORCE 15 // coeficiente de la repulsión
#define MIN_REPULSION_RADIUS 0.2 // radio de repulsión del mínimo local
#define LOCAL_MIN_RADIUS 0.2 // radio de detección del mínimo local
#define POINT_SEARCH_RADIUS 0.4 // radio de búsqueda del próximo punto
#define SONAR_DISTANCE 0.5  // distancia de lectura de los sónares
#define MAX_SPEED 12.0 // velocidad linear máxima
#define MAX_ANGULAR_SPEED 8.0 // velocidad angular máxima
#define MAGABOT_WIDTH 0.34   // distancia entre ruedas
#define WHEEL_RADIUS 0.045 // radio de las ruedas 
#define servoPin 22 // pin del servo del puntador
/**
      D = 2PI * r * ticks  / ticksPerTurn
        Dc = (Dr + Dl)/2
       a' = a + (Dr - Dl)/L
       x' = x + Dc * cos(a')
       y' = y + Dc * sin(a')

		^  X+
		|  0
		|
  PI/2  /000\
  <--- |00000|--->
  y+    \000/     y- -PI/2
		|
		|
  PI      v  X-  -PI

  rango del ángulo [ PI y -PI)
 
*/

/**
   estructura que sirve para identificar unas coordenadas
*/
struct MapPoint {
  uint8_t x;
  uint8_t y;
};

/**
   estructura que sirve para identificar un punto en el sistema de refernecia
*/
struct WorldPoint {
  double x;
  double y;
};

/**
   estructura que sirve para indicar una posición en el espacio de referencia,
   inclusa la orientación. Se usa tamién para realizar transformaciones.
*/
struct Position {
  double x;
  double y;
  double theta;
};



Servo myServo;
float sonarReadings [5]; // array que almacena las lecturas de los sónares
Magabot robot; // define el objeto de la clase Magabot del paquete Magabot.cpp
uint8_t direction_map [MAP_SIZE][MAP_SIZE] ; // matriz que sirve para crear campos potenciales que atraen hacia el objetivo
uint8_t obstacles_map [MAP_SIZE][MAP_SIZE] ; // matriz que sirve para crear campos potenciales que repelen de los obstáculos
Position sonarTransform[5]; // posicones de los sónares respecto al centro del robot
double soundAngle; // ángulo de la fuente del sonido respecto al punto odom
double gradient; // gradiente de soundAngle
Position robotPosition; // Posición del robot respecto a odom
WorldPoint mapCenter; // punto central del mapa respecto a odom
long ledsTimer; // timer para apagar los leds
bool moving; // determina si el robot está en movimiento
String readString; // string donde se almacenta la lectura desde serial
WorldPoint goal; // punto más lejano en el mapa en dirección hacia el sonido.
bool ping; // el sonar ha enviado el sonido
uint8_t active_sonar; // identificador del sonar activo
WorldPoint nextPoint; // objetivo local


void setup() { // OK
  Serial.begin(19200); // Serial de debug
  Serial1.begin(9600); // Serial para recibir el ángulo.
  Wire.begin();
  for (float rd : sonarReadings) {
    rd = 0;
  }
  stopRobot();
  setSonarsTransform();
  moving = false;
  myServo.attach(servoPin);
  ledsTimer = millis();
}

/**
   el robot frena y se resetean el mapa de obstáculos y la posición de odom.
*/
void stopRobot() { // ok
  robot.actuateMotors(0, 0);
  blinkBlue();
  robotPosition = {0, 0, 0};
  resetMapCenter();
  resetObstaclesMap();
  moving = false;
}

/**
   función que define las transformaciones estáticas entre cada sonar y el centro del robot.
*/
void setSonarsTransform() { //OK
  sonarTransform[0] = {0.11, 0.1, PI / 4};
  sonarTransform[1] = {0.12, 0.05, PI / 6};
  sonarTransform[2] = {0.13, 0, 0};
  sonarTransform[3] = {0.12, -0.05, -PI / 6};
  sonarTransform[4] = {0.11, -0.1, -PI / 4};
}

void loop() { //OK
  checkLEDs();
  checkSerial();

  if (moving) {
    checkBumpers();
    updateRobotPosition();
    checkBorder();
    detectObstacles();
    do {
      nextPoint = getNextPoint();

    } while (checkLocalMinimum(nextPoint));

    pointGoal();
    moveTo(nextPoint);

  }
}

/**
   función que apaga los leds
*/
void checkLEDs() { //OK
  if (ledsTimer < millis()) {
    robot.actuateLEDs(0, 0, 0);
  }
}

/**
   función que escucha si se ha recibido un ángulo por serial.
   En caso afirmativo, si el robot está quieto, inicia el movimiento hacia la dirección recibida.
   Si el robot está en movimiento, éste se para.
*/
void checkSerial() { //OK
  while (Serial1.available()) {
    char c = Serial1.read();
    readString += c;
    delay(2);
  }
  if (readString.length() > 0) {
    if (moving) {
      stopRobot();
    } else {
      blinkRed();
      soundAngle = readString.toDouble();
      gradient = tan(soundAngle);
      movePointer(soundAngle);
      setDirectionMap();
      moving = true;
    }
    readString = "";
  }
}

/**
   Controla si se ha activado algún bumper.
*/
void checkBumpers() { //OK
  if ( robot.readBumpers()) {
    stopRobot();
  }
}

/**
   Función que calcula el desplazamiento linear  (Dc) y el desplazamento angular (Da),
   en base al desplazamiento de cada rueda (Dr, Dl). Luego actualiza la posición del robot respecto a odom.
   Las formulas empleadas son las siguientes:
      Dc = (Dr + Dl)/2
      Da = (Dr - Dl)/L
       a' = a + Da
       x' = x + Dc * cos(Da)
      y' = y + Dc * sin(Da)
*/
void updateRobotPosition() { //OK
  float wheelsRotation[2];
  robot.readClicks(wheelsRotation);
  double deltaLeft = wheelsRotation[0] * WHEEL_RADIUS;
  double deltaRight = wheelsRotation[1] * WHEEL_RADIUS;
  double deltaLinear = (deltaRight + deltaLeft) / 2.0;
  double deltaTheta = (deltaRight - deltaLeft) / MAGABOT_WIDTH;
  robotPosition.theta += deltaTheta;
  robotPosition.x += deltaLinear * cos(robotPosition.theta);
  robotPosition.y += deltaLinear * sin(robotPosition.theta);
}

/**
   Función que comprueba si el robot esá cerca del borde del mapa.
   En caso afirmativo, se vuelve a crear un mapa alrededor del robot.
*/
void checkBorder() { //OK
  MapPoint robotMapPoint = getMapPoint({robotPosition.x, robotPosition.y});
  if (robotMapPoint.x >= MAP_SIZE - 5 || robotMapPoint.y >= MAP_SIZE - 5 || robotMapPoint.x <= 4 || robotMapPoint.y <= 4) {
    robot.actuateMotors(0, 0);
    blinkGreen();
    resetMapCenter();
    setDirectionMap();
    resetObstaclesMap();
  }
}

/**
    Detecta obstáculos. Si algún sonar detecta obstáculos, se crea repulsión alrededor de cada uno.
    Es necesario transformar las lecturas de los sónares primero respecto a la posición del robot, y loego hacia odom.
*/
void detectObstacles() { //OK
  uint8_t i = robot.getSonarReadings(sonarReadings);
  if (i >= 0 && sonarReadings[i] >= 0.05 && sonarReadings[i] <= SONAR_DISTANCE) {
    WorldPoint obst_robot = transformPoint(sonarTransform[i], {sonarReadings[i], 0});
    WorldPoint obst_odom = transformPoint(robotPosition, obst_robot);
    addObstacle(obst_odom);
  }
}

/**
   busca el próximo punto hacia donde moverse en un radio POINT_SEARCH_RADIUS
*/
WorldPoint getNextPoint() { //OK
  WorldPoint nextPoint = {robotPosition.x, robotPosition.y};
  MapPoint robotMapPoint = getMapPoint(nextPoint);
  uint16_t nextPointRepulsion = getForce(robotMapPoint);
  for (uint8_t i = robotMapPoint.x - ceil(POINT_SEARCH_RADIUS / MAP_PRECISION); i < robotMapPoint.x + ceil(POINT_SEARCH_RADIUS / MAP_PRECISION); i++) { // desde 1m menos del obstaculo en el eje x
    for (uint8_t j = robotMapPoint.y - ceil(POINT_SEARCH_RADIUS / MAP_PRECISION); j < robotMapPoint.y + ceil(POINT_SEARCH_RADIUS / MAP_PRECISION); j++) { // desde 1m menos del obstaculo en el eje y
      if (i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE) { // compruebo que no se salga del rango del mapa
        WorldPoint thisPoint = getWorldPoint({i, j});
        if (euclideanDistance(thisPoint, {robotPosition.x, robotPosition.y}) <= POINT_SEARCH_RADIUS) {
          uint16_t tempRepulsion = getForce({i, j});
          if (tempRepulsion < nextPointRepulsion) {
            nextPoint = thisPoint;
            nextPointRepulsion = tempRepulsion;
          }
        }
      }
    }
  }
  return nextPoint;
}

/**
   En el caso de que el próximo punto se encuentre en un radio LOCAL_MIN_RADIUS alrededor del centro del robot,
    se crea repulsión cada medio segundo en un radio MIN_REPULSION_RADIUS alrededor del robot.
*/

bool checkLocalMinimum( WorldPoint nextPoint) { //OK
  bool isMin = false;
  if (euclideanDistance(nextPoint, {robotPosition.x, robotPosition.y}) <= LOCAL_MIN_RADIUS) {
    blinkPurple();
    MapPoint robotMapPoint = getMapPoint({robotPosition.x, robotPosition.y});
    for (uint8_t i = robotMapPoint.x - ceil(MIN_REPULSION_RADIUS / MAP_PRECISION); i < robotMapPoint.x + ceil(MIN_REPULSION_RADIUS / MAP_PRECISION); ++i) {
      for (uint8_t j = robotMapPoint.y - ceil(MIN_REPULSION_RADIUS / MAP_PRECISION); j < robotMapPoint.y + ceil(MIN_REPULSION_RADIUS / MAP_PRECISION); ++j) {
        if (i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE) {
          MapPoint tempMapPoint = {i, j};
          WorldPoint tempWorldPoint = getWorldPoint(tempMapPoint);
          double distance = euclideanDistance(tempWorldPoint, {robotPosition.x, robotPosition.y});
          if (obstacles_map[i][j] < 254 && distance <= MIN_REPULSION_RADIUS) {
            ++obstacles_map[i][j];
          }
        }
      }
    }
    isMin = true;
  }
  return isMin;
}

/**
   el puntador apunta hacia el punto objetivo
*/
void pointGoal() { //OK
  double goalAngle = atan2(goal.y - robotPosition.y, goal.x - robotPosition.x) - robotPosition.theta;
  movePointer(goalAngle);
}

/**
    Función para mover el robot y orientar el puntador en dirección del movimiento.
*/
void moveTo(WorldPoint nextPoint) { //OK
  //ángulo del objetivo local, respecto al robot
  double nextPointAngle = atan2(nextPoint.y - robotPosition.y, nextPoint.x - robotPosition.x) - robotPosition.theta;
  int angularSpeed = MAX_ANGULAR_SPEED * sin(nextPointAngle);
  int linearSpeed = MAX_SPEED - MAX_SPEED * (abs(angularSpeed) / MAX_ANGULAR_SPEED); // limitador de velocidad en base al ángulo del próximo objetivo.
  robot.actuateMotors(linearSpeed - angularSpeed, linearSpeed + angularSpeed);
}

/**
   devuelve la suma de las dos matrices en el punto indicado.
*/
uint16_t getForce(MapPoint p) { //ok
  return (uint16_t) direction_map [p.x][p.y] + (uint16_t)obstacles_map [p.x][p.y];
}

/**
   Mueve el servo del puntador
*/
void movePointer(double angle) { //ok
  if (angle < -PI) {
    angle += 2 * PI;
  } else if (angle > PI) {
    angle -= 2 * PI;
  }

  double servoAngle = round(((angle * 180 / PI) + 180) / 2);
  if (servoAngle < 0) {
    servoAngle = 0;
  } else if (angle > 179) {
    servoAngle = 179;
  }
  myServo.write(servoAngle);
}



/**
   Se pone repulsión alrededor de un obstáculo en el radio ROI
*/
void addObstacle(WorldPoint obstacle_worldPoint) {
  MapPoint obstacle_mapPoint = getMapPoint(obstacle_worldPoint);
  obstacles_map[obstacle_mapPoint.x][obstacle_mapPoint.y] = 255;
  for (uint8_t i = obstacle_mapPoint.x - ceil(ROI / MAP_PRECISION); i < obstacle_mapPoint.x + ceil(ROI / MAP_PRECISION); i++) {
    for (uint8_t j = obstacle_mapPoint.y - ceil(ROI / MAP_PRECISION); j < obstacle_mapPoint.y + ceil(ROI / MAP_PRECISION); j++) {
      if (i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE) { // compruebo que no se salga del rango del mapa
        double dist = euclideanDistance(getWorldPoint({i, j}), obstacle_worldPoint);
        //        if (i != obstacle_mapPoint.x && j != obstacle_mapPoint.y && dist <= ROI) {
        if ( dist != 0 && dist <= ROI) {

          uint16_t force16 = round((ROI - dist) * KFORCE / dist) ;
          uint8_t force;
          if (force16 <= 255) {
            force = force16;
          } else {
            force = 255;
          }

          if (obstacles_map[i][j] < force) {
            obstacles_map[i][j] = force;
          }
        }
      }
    }
  }

}
/**
   función que recibe un punto p y lo transforma, dependiendo de la posición de pos respecto al orígen
*/

WorldPoint transformPoint(Position pos, WorldPoint p) //ok
{
  double s = sin(pos.theta);
  double c = cos(pos.theta);
  // aplico rotación
  double xnew = p.x * c - p.y * s;
  double ynew = p.x * s + p.y * c;
  // traslado el punto
  xnew += pos.x;
  ynew += pos.y;
  return {xnew, ynew};
}



void resetObstaclesMap() { //
  for (uint8_t i = 0; i < MAP_SIZE; i++) {
    for (uint8_t j = 0; j < MAP_SIZE; j++) {
      obstacles_map[i][j] = 0;
    }
  }
}



/**
   distencia euclidea entre dos puntos
*/
double euclideanDistance(WorldPoint p1, WorldPoint p2) { //OK
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

/**
   Función que crea la atracción hacia el sonido.
   Usa como objetivo el punto más lejano en el mapa en dirección del sonido.
   Para cada punto del mapa, calcula la distancia en dm del punto al objetivo y lo suma en la matriz direction_map .
    Distancia de P(x0, x1) a recta y = mx
    d = abs(y0 - m * x0)/ sqrt(1+ pow(m,2))
*/
void setDirectionMap() {

  double max_x = mapCenter.x + WORLD_SIZE / 2.0;
  double min_x = mapCenter.x - WORLD_SIZE / 2.0;
  double max_y = mapCenter.y + WORLD_SIZE / 2.0;
  double min_y = mapCenter.y - WORLD_SIZE / 2.0;

  if (abs(soundAngle) < PI / 2.0) {
    goal = {max_x, max_x * gradient  };
  } else if ( abs(soundAngle) > PI / 2.0) {
    goal = {min_x, min_x * gradient };
  } else if ( soundAngle > 0) {
    goal = {max_y / gradient , max_y};
  } else {
    goal = {min_y / gradient , min_y};
  }

  for (uint8_t i = 0; i < MAP_SIZE; i++) {
    for (uint8_t j = 0; j < MAP_SIZE; j++) {

      WorldPoint p = getWorldPoint({i, j});
      direction_map[i][j] = round(10 * euclideanDistance(p, goal));
    }
  }
}

/**
   recibe unas coordenadas del mapa y devuelve el punto central respecto a odom.
*/

WorldPoint getWorldPoint(MapPoint mPoint) {
  WorldPoint wPoint;
  wPoint.x = ((mPoint.x - MAP_SIZE / 2.0) * MAP_PRECISION) + mapCenter.x ;
  wPoint.y = ((mPoint.y - MAP_SIZE / 2.0) * MAP_PRECISION) + mapCenter.y;

  return wPoint;
}

/**
   recibe un punto respecto a odom y devuelve las coordenadas de este punto en el mapa.
*/
MapPoint getMapPoint(WorldPoint wPoint) {
  struct MapPoint mapPoint;
  mapPoint.x = round((wPoint.x - mapCenter.x ) / MAP_PRECISION) + MAP_SIZE / 2;
  mapPoint.y = round((wPoint.y - mapCenter.y ) / MAP_PRECISION) + MAP_SIZE / 2;
  return mapPoint;
}

/**
   función que pone como centro del mapa a la posición actual del robot respecto a odom
*/
void resetMapCenter() {
  mapCenter = {robotPosition.x, robotPosition.y};
}


/**
   enciende los leds de color rojo
*/
void blinkRed() {
  robot.actuateLEDs(255, 0, 0);
  ledsTimer = millis() + 1000;
}

/**
   enciende los leds de color verde
*/
void blinkGreen() {
  robot.actuateLEDs(0, 255, 0);
  ledsTimer = millis() + 1000;
}

/**
   enciende los leds de color azul
*/
void blinkBlue() {
  robot.actuateLEDs(0, 0, 255);
  ledsTimer = millis() + 1000;
}

/**
   enciende los leds de color violeta
*/
void blinkPurple() {
  robot.actuateLEDs(255, 0, 255);
  ledsTimer = millis() + 500;
}

/**
   Métodos de test
*/
void printPosition() {
  Serial.print(robotPosition.x * 100);
  Serial.print("\t");
  Serial.print(robotPosition.y * 100);
  Serial.print("\t");
  Serial.print(robotPosition.theta);
  Serial.println("");
}


void sonarsTest() {
  robot.getSonarReadings(sonarReadings);
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(sonarReadings[i]);
    Serial.print("\t");

  }
  Serial.println("");
}


void printDirectionMap() {
  for (uint8_t i = MAP_SIZE - 1; i >= 0; --i) {
    for (uint8_t j = MAP_SIZE - 1; j >= 0; --j) {
      Serial.print(direction_map[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
}
void printObstaclesMap() {
  MapPoint r = getMapPoint({robotPosition.x, robotPosition.y});
  obstacles_map[r.x][r.y] = 8;
  Serial.println("");
  Serial.println("");

  for (uint8_t i = MAP_SIZE - 1; i >= 0; --i) {
    for (uint8_t j = MAP_SIZE - 1; j >= 0; --j) {
      Serial.print(obstacles_map[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  moving = false;

}
void printPointsInMap() {
  MapPoint r = getMapPoint({robotPosition.x, robotPosition.y});
  obstacles_map[r.x][r.y] = 8;
  for (uint8_t i = MAP_SIZE - 1; i >= 0; --i) {
    for (uint8_t j = MAP_SIZE - 1; j >= 0; --j) {
      WorldPoint wp = getWorldPoint({i, j});
      Serial.print("[");
      Serial.print(wp.x );
      Serial.print(" , ");
      Serial.print("] ");
    }
    Serial.println("");
  }
  moving = false;

}
void printAllMaps() {
  MapPoint r = getMapPoint({robotPosition.x, robotPosition.y});
  obstacles_map[r.x][r.y] = 8;
  Serial.println("");
  Serial.println("");

  for (uint8_t i = MAP_SIZE - 1; i >= 0; --i) {
    for (uint8_t j = MAP_SIZE - 1; j >= 0; --j) {
      Serial.print((int)obstacles_map[i][j] + (int)direction_map[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  moving = false;

}

void straightMotorsTest(double distance) {
  while (robotPosition.x < distance) {

    robot.actuateMotors(5, 5);
    updateRobotPosition();

  }
  robot.actuateMotors(0, 0);
  updateRobotPosition();

  printPosition();


}
void squareTest(double side, bool clockwise) {
  WorldPoint p0 = {0, 0};
  WorldPoint p1 = {side, 0};
  WorldPoint p2;
  WorldPoint p3;

  if (clockwise) {
    WorldPoint p1 = {side, 0};
    p2 = {side, - side};
    p3 = {0, -side};
  }
  else {
    p2 = {side, side};
    p3 = {0, side};
  }

  WorldPoint  corners [] = {p1, p2, p3, p0};
  for (WorldPoint c : corners) {
    int maxspeed = 5;
    while (euclideanDistance({robotPosition.x, robotPosition.y}, c) > 0.05) {
      updateRobotPosition();
      double nextPointAngle = atan2(c.y - robotPosition.y, c.x - robotPosition.x) - robotPosition.theta;

      int angularSpeed = ceil(MAX_ANGULAR_SPEED * sin(nextPointAngle));
      int linearSpeed = maxspeed - maxspeed * (abs(angularSpeed) / MAX_ANGULAR_SPEED); // limitador de velocidad en base al ángulo del próximo objetivo.
      robot.actuateMotors(linearSpeed - angularSpeed, linearSpeed + angularSpeed);
      updateRobotPosition();
    }
  }
  robot.actuateMotors(0, 0);
  updateRobotPosition();

  printPosition() ;

  while (true) {

  }
}

