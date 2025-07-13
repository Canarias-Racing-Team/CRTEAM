#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

#define DEBUG true  // habilita/deshabilita los mensajes por el monitor serie
#define NUM_SENSORES 1     // número de acelerómetros
#define TIEMPO_REPOSO 100  // tiempo en ms en reposo antes de resetear velocidad

struct DatosMPU {
  float ax, ay, az;  // aceleración corregida
  float gx, gy, gz;  // giroscopio
  float ux, uy,
      uz;  // umbrales de ruido (se puede usar si no se corrige gravedad)
};

struct Velocidad {
  float vx, vy, vz;
};

MPU6050 sensores[NUM_SENSORES];
Madgwick filtros[NUM_SENSORES];
DatosMPU datos[NUM_SENSORES + 1];  // +1 para promedio
DatosMPU calibracion[NUM_SENSORES];
Velocidad velocidad;

uint8_t direcciones[NUM_SENSORES] = {0x68, 0x69}; // direcciones I2C de los sensores
unsigned long tiempoReposo = 0;
bool enReposo = false;
unsigned long tAnterior;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  debug("Inicializando...");
  calibrar(NUM_SENSORES);
  debug("Calibrado completo.");

  tAnterior = millis();
}

void loop() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    leer_datos(i);
    corregir_gravedad(i);  // Corrige aceleración quitando gravedad
  }

  obtener_promedio();
  calcular_velocidad();
  enviar_datos();
  delay(100);  // 10 Hz
}

void leer_datos(int i) {
  int16_t ax, ay, az, gx, gy, gz;
  sensores[i].getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float aScale = 9.81 / 16384.0;   // factor de escala del acelerómetro
  float gScale = 250.0 / 32768.0;  // factor de escala del giroscopio

  // Lectura bruta escalada y calibrada
  datos[i].ax = ax * aScale - calibracion[i].ax;
  datos[i].ay = ay * aScale - calibracion[i].ay;
  datos[i].az = az * aScale - calibracion[i].az;

  datos[i].gx = gx * gScale;
  datos[i].gy = gy * gScale;
  datos[i].gz = gz * gScale;

  // Actualiza filtro Madgwick
  filtros[i].updateIMU(datos[i].gx * DEG_TO_RAD, datos[i].gy * DEG_TO_RAD,
                       datos[i].gz * DEG_TO_RAD, datos[i].ax, datos[i].ay,
                       datos[i].az);

  // Sin umbrales aquí — los quitamos al corregir gravedad

  debug("Sensor " + String(i) + " ax=" + String(datos[i].ax, 2) +
        " ay=" + String(datos[i].ay, 2) + " az=" + String(datos[i].az, 2));
}

void corregir_gravedad(int i) {
  // Usa orientación del filtro para eliminar la gravedad
  float q0 = filtros[i].q0;
  float q1 = filtros[i].q1;
  float q2 = filtros[i].q2;
  float q3 = filtros[i].q3;

  // Rotación inversa del vector de aceleración (del sistema a mundo)
  float gx = 2 * (q1 * q3 - q0 * q2);
  float gy = 2 * (q0 * q1 + q2 * q3);
  float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // Corregir aceleración: quitar la gravedad en el eje Z
  datos[i].ax -= gx * 9.81;
  datos[i].ay -= gy * 9.81;
  datos[i].az -= gz * 9.81;
}

void obtener_promedio() {
  DatosMPU &avg = datos[NUM_SENSORES];
  avg.ax = avg.ay = avg.az = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    avg.ax += datos[i].ax;
    avg.ay += datos[i].ay;
    avg.az += datos[i].az;
  }
  avg.ax /= NUM_SENSORES;
  avg.ay /= NUM_SENSORES;
  avg.az /= NUM_SENSORES;
}

void calcular_velocidad() {
  unsigned long tActual = millis();
  float dt = (tActual - tAnterior) / 1000.0;
  tAnterior = tActual;

  DatosMPU &a = datos[NUM_SENSORES];

  float magnitud = sqrt(a.ax * a.ax + a.ay * a.ay + a.az * a.az);

  // Detectar reposo con tolerancia pequeña
  if (magnitud < 0.1) {
    if (!enReposo) {
      tiempoReposo = tActual;
      enReposo = true;
    } else if (tActual - tiempoReposo > TIEMPO_REPOSO) {
      reset_velocidad();
      debug("Reposo detectado, velocidad reseteada.");
    }
  } else {
    enReposo = false;
  }

  // Integración de velocidad
  velocidad.vx += a.ax * dt;
  velocidad.vy += a.ay * dt;
  velocidad.vz += a.az * dt;
}

void reset_velocidad() {
  velocidad.vx = 0;
  velocidad.vy = 0;
  velocidad.vz = 0;
}

void enviar_datos() {
  float v_total =
      sqrt(velocidad.vx * velocidad.vx + velocidad.vy * velocidad.vy +
           velocidad.vz * velocidad.vz);

  String msg = "Velocidad (m/s): X=" + String(velocidad.vx, 2);
  msg += " Y=" + String(velocidad.vy, 2);
  msg += " Z=" + String(velocidad.vz, 2);
  msg += " | Total (km/h)=" + String(v_total * 3.6, 2);
  debug(msg);
}

void calibrar(int num) {
  for (int i = 0; i < num; i++) {
    sensores[i] = MPU6050(direcciones[i]);
    sensores[i].initialize();

    debug("Conectando sensor: " + String(i));
    if (!sensores[i].testConnection()) {
      debug("Error conectando sensor " + String(i));
      while (true);  // detener
    }

    const int N = 1000;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float umbralx = 0, umbraly = 0, umbralz = 0;

    for (int j = 0; j < N; j++) {
      int16_t ax, ay, az;
      sensores[i].getAcceleration(&ax, &ay, &az);
      sum_ax += ax;
      sum_ay += ay;
      sum_az += az;

      if (umbralx < abs(ax)) umbralx = abs(ax);
      if (umbraly < abs(ay)) umbraly = abs(ay);
      if (umbralz < abs(az)) umbralz = abs(az);

      delay(1);
    }

    float scale = 9.81 / 16384.0;
    calibracion[i].ax = (sum_ax / N) * scale;
    calibracion[i].ay = (sum_ay / N) * scale;
    calibracion[i].az = (sum_az / N) * scale;

    calibracion[i].ux = max(umbralx * scale - fabs(calibracion[i].ax), 0.05);
    calibracion[i].uy = max(umbraly * scale - fabs(calibracion[i].ay), 0.05);
    calibracion[i].uz = max(umbralz * scale - fabs(calibracion[i].az), 0.05);

    debug("Sensor calibrado: " + String(i));
  }
}

void debug(String msg) {
  if (DEBUG) {
    Serial.print("[");
    Serial.print(millis() / 1000.0, 3);
    Serial.print("s] ");
    Serial.println(msg);
  }
}
