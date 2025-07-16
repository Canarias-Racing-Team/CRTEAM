#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <mrf24j.h>

#define DEBUG true
#define NUM_SENSORES 1
#define TIEMPO_REPOSO 300  // ms para mejor estabilidad

struct DatosMPU {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float ux = 0, uy = 0, uz = 0;
};

struct Velocidad {
  float vx = 0, vy = 0, vz = 0;
};

struct FiltroAcel {
  float ax_f = 0, ay_f = 0, az_f = 0;
};

FiltroAcel filtrosAcel[NUM_SENSORES];
MPU6050 sensores[NUM_SENSORES];
Madgwick filtros[NUM_SENSORES];
DatosMPU calibracion[NUM_SENSORES];
Velocidad velocidad;
DatosMPU datos[NUM_SENSORES + 1];  // Incluye espacio para promedio

const uint8_t direcciones[NUM_SENSORES] = {0x68};
unsigned long tiempoReposo = 0;
bool enReposo = false;
unsigned long tAnterior = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  debug("Inicializando...");
  calibrar(NUM_SENSORES);
  debug("Calibrado completo.");
  tAnterior = millis();
}

void loop() {
  for (int i = 0; i < NUM_SENSORES; ++i) {
    leer_datos(i);
    corregir_gravedad(i);
  }
  obtener_promedio();
  calcular_velocidad();
  enviar_datos();
  delay(100);
}

void leer_datos(int i) {
  int16_t ax, ay, az, gx, gy, gz;
  sensores[i].getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  constexpr float aScale = 9.81f / 16384.0f;
  constexpr float gScale = 250.0f / 32768.0f;
  datos[i].ax = ax * aScale - calibracion[i].ax;
  datos[i].ay = ay * aScale - calibracion[i].ay;
  datos[i].az = az * aScale - calibracion[i].az;
  datos[i].gx = gx * gScale;
  datos[i].gy = gy * gScale;
  datos[i].gz = gz * gScale;
  filtros[i].updateIMU(datos[i].gx * DEG_TO_RAD, datos[i].gy * DEG_TO_RAD,
                       datos[i].gz * DEG_TO_RAD, datos[i].ax, datos[i].ay,
                       datos[i].az);
  debug("Sensor " + String(i) + " ax=" + String(datos[i].ax, 3) +
        " ay=" + String(datos[i].ay, 3) + " az=" + String(datos[i].az, 3));
}

void corregir_gravedad(int i) {
  float q[4];
  filtros[i].getQuaternion(q, q + 1, q + 2, q + 3);
  // Vector gravedad estimado (normalizado)
  float gx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  float gy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  float mag_g = gx * gx + gy * gy + gz * gz;
  if (mag_g > 0.0000001f) {
    float invMag = 1.0f / sqrtf(mag_g);
    gx *= invMag;
    gy *= invMag;
    gz *= invMag;
  }
  // Corregir aceleración restando gravedad
  float ax_corr = datos[i].ax - gx * 9.81f;
  float ay_corr = datos[i].ay - gy * 9.81f;
  float az_corr = datos[i].az - gz * 9.81f;
  // Filtro pasa-bajo eficiente
  constexpr float alpha = 0.85f;
  FiltroAcel &fa = filtrosAcel[i];
  fa.ax_f += (ax_corr - fa.ax_f) * (1 - alpha);
  fa.ay_f += (ay_corr - fa.ay_f) * (1 - alpha);
  fa.az_f += (az_corr - fa.az_f) * (1 - alpha);
  datos[i].ax = fa.ax_f;
  datos[i].ay = fa.ay_f;
  datos[i].az = fa.az_f;
  // Debug solo si hay aceleración significativa
  if (fabs(datos[i].ax) > 0.05f || fabs(datos[i].ay) > 0.05f ||
      fabs(datos[i].az) > 0.05f) {
    debug("Acel sin gravedad (filtrada): ax=" + String(datos[i].ax, 4) +
          " ay=" + String(datos[i].ay, 4) + " az=" + String(datos[i].az, 4));
  }
}

void obtener_promedio() {
  DatosMPU &avg = datos[NUM_SENSORES];
  avg.ax = avg.ay = avg.az = 0;
  for (int i = 0; i < NUM_SENSORES; ++i) {
    avg.ax += datos[i].ax;
    avg.ay += datos[i].ay;
    avg.az += datos[i].az;
  }
  if (NUM_SENSORES > 0) {
    avg.ax /= NUM_SENSORES;
    avg.ay /= NUM_SENSORES;
    avg.az /= NUM_SENSORES;
  }
}

void calcular_velocidad() {
  unsigned long tActual = millis();
  float dt = (tActual - tAnterior) / 1000.0f;
  tAnterior = tActual;
  DatosMPU &a = datos[NUM_SENSORES];
  float magnitud = sqrt(a.ax * a.ax + a.ay * a.ay + a.az * a.az);
  debug("Magnitud acel: " + String(magnitud, 4));
  // Detección de reposo
  if (magnitud < 0.2f) {
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
  constexpr float threshold = 0.15f;
  // Solo integramos aceleraciones significativas
  if (fabs(a.ax) > threshold) velocidad.vx += a.ax * dt;
  if (fabs(a.ay) > threshold) velocidad.vy += a.ay * dt;
  if (fabs(a.az) > threshold) velocidad.vz += a.az * dt;
}

void reset_velocidad() { velocidad = Velocidad(); }

void enviar_datos() {
  float v_total =
      sqrt(velocidad.vx * velocidad.vx + velocidad.vy * velocidad.vy +
           velocidad.vz * velocidad.vz);
  String msg = "Velocidad (m/s): X=" + String(velocidad.vx, 3);
  msg += " Y=" + String(velocidad.vy, 3);
  msg += " Z=" + String(velocidad.vz, 3);
  msg += " | Total (km/h)=" + String(v_total * 3.6, 3);
  debug(msg);
}

void calibrar(int num) {
  for (int i = 0; i < num; ++i) {
    sensores[i] = MPU6050(direcciones[i]);
    sensores[i].initialize();
    debug("Conectando sensor: " + String(i));
    if (!sensores[i].testConnection()) {
      debug("Error conectando sensor " + String(i));
      while (true);
    }
    constexpr int N = 1000;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float umbralx = 0, umbraly = 0, umbralz = 0;
    for (int j = 0; j < N; ++j) {
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
    constexpr float scale = 9.81f / 16384.0f;
    calibracion[i].ax = (sum_ax / N) * scale;
    calibracion[i].ay = (sum_ay / N) * scale;
    calibracion[i].az = (sum_az / N) * scale;
    calibracion[i].ux = max(umbralx * scale - fabs(calibracion[i].ax), 0.05f);
    calibracion[i].uy = max(umbraly * scale - fabs(calibracion[i].ay), 0.05f);
    calibracion[i].uz = max(umbralz * scale - fabs(calibracion[i].az), 0.05f);
    debug("Sensor calibrado: " + String(i));
  }
}

void debug(const String &msg) {
  if (DEBUG) {
    Serial.print("[");
    Serial.print(millis() / 1000.0, 3);
    Serial.print("s] ");
    Serial.println(msg);
  }
}