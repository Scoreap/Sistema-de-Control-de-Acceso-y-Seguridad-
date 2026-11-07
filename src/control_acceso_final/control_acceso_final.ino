/*
  Arquitectura del Computador - Proyecto final, Variante A
  Arduino UNO / ATmega328P
  RFID RC522 + EEPROM (formato compatible con fase1.ino)
  74HC595 + 8 LEDs, teclado 4x4, LCD I2C 16x2,
  servo SG90 y buzzer ACTIVO.

  Teclas: * borrar, # confirmar PIN, D cerrar puerta,
          C apagar alarma (despues de 3 PIN incorrectos).

  IMPORTANTE: el teclado usa D0 y D1. Desconectar temporalmente
  esas DOS lineas al cargar el sketch por USB; reconectar luego.
  Este programa final NO usa Serial.

  Las credenciales se deben registrar antes con fase1.ino usando
  REG 1 dddd / REG 2 dddd, y se conservan en EEPROM.

  Ajustar ANGULO_CERRADO / ANGULO_ABIERTO a la puerta real.
  Verificar alimentaciones: RC522 a 3.3V y niveles SPI adaptados;
  servo con fuente apropiada y GND comun; buzzer con consumo apto.
*/


#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <string.h>

// ------------------ PINES SEGUN NUESTRA TABLA ------------------
const byte PIN_BUZZER = 2;
const byte PIN_SERVO  = 4;
const byte RFID_RST = 6;
const byte PIN_DATA  = 7;
const byte PIN_CLOCK = 8;
const byte PIN_LATCH = 9;
const byte RFID_SS = 10; // SPI MOSI=11, MISO=12, SCK=13

// Teclado: F1/F2/F3/F4 -> D0/D1/D3/D5; C1..C4 -> A0..A3.
const byte FILAS[4] = {0, 1, 3, 5};
const byte COLUMNAS[4] = {A0, A1, A2, A3};
const char TECLAS[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// Confirmar la direccion I2C de tu modulo; la mas habitual es 0x27.
LiquidCrystal_I2C lcd(0x27, 16, 2);
MFRC522 rfid(RFID_SS, RFID_RST);
Servo puerta;

const byte ANGULO_CERRADO = 0;
const byte ANGULO_ABIERTO = 90;
const uint32_t MS_INICIO = 1500;
const uint32_t MS_VALIDACION = 350;
const uint32_t MS_MENSAJE = 1600;
const uint32_t MS_REBOTE = 35;
const byte MAX_INTENTOS = 3;

// ------- FORMATO EEPROM: IDENTICO AL DE fase1.ino -------
namespace Access {
  const byte USER_COUNT = 2;
  const byte RECORD_SIZE = 19;
  const byte MARKER = 0xA7;
  const byte VERSION = 1;
  struct User {
    byte length;
    byte uid[10];
    char pin[4]; // NO termina en '\0'; siempre comparar 4 bytes.
  };
  bool validLength(byte n) { return n == 4 || n == 7 || n == 10; }
  bool validPin(const char *pin) {
    for (byte i = 0; i < 4; ++i)
      if (pin[i] < '0' || pin[i] > '9') return false;
    return true;
  }
  uint16_t crc16(const byte *data, byte count) {
    uint16_t crc = 0xFFFF;
    for (byte i = 0; i < count; ++i) {
      crc ^= static_cast<uint16_t>(data[i]) << 8;
      for (byte j = 0; j < 8; ++j)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
  }
  bool decode(const byte *record, User &user) {
    if (record[0] != MARKER || record[1] != VERSION ||
        !validLength(record[2])) return false;
    uint16_t expected = static_cast<uint16_t>(record[17]) |
                        (static_cast<uint16_t>(record[18]) << 8);
    if (crc16(record + 1, 16) != expected || !validPin((const char *)(record + 13)))
      return false;
    user.length = record[2];
    memcpy(user.uid, record + 3, 10);
    memcpy(user.pin, record + 13, 4);
    return true;
  }
}

bool leerUsuario(byte slot, Access::User &usuario) {
  if (slot >= Access::USER_COUNT) return false;
  byte raw[Access::RECORD_SIZE];
  for (byte i = 0; i < Access::RECORD_SIZE; ++i)
    raw[i] = EEPROM.read(slot * Access::RECORD_SIZE + i);
  return Access::decode(raw, usuario);
}

// ---------------- MAQUINA DE ESTADOS Y 8 LEDS ----------------
enum Estado : byte {
  INICIO, ESPERANDO_RFID, VALIDANDO_RFID, ESPERANDO_PIN,
  VALIDANDO_PIN, AUTORIZADO, DENEGADO, ALARMA, PUERTA_ABIERTA
};
Estado estado = INICIO;
uint32_t desdeEstado = 0;

void actualizarLEDs(byte mascara) {
  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLOCK, MSBFIRST, mascara);
  digitalWrite(PIN_LATCH, HIGH);
}
void cambiarEstado(Estado nuevo) {
  estado = nuevo;
  desdeEstado = millis();
  byte bit = (nuevo == PUERTA_ABIERTA) ? 5 : (byte)nuevo;
  actualizarLEDs((byte)(1U << bit));
}

// ----------------------- TECLADO -----------------------
// Barrido sin librerias extra: filas activas en LOW,
// columnas con resistencia pull-up interna del ATmega328P.
char teclaInstantanea() {
  for (byte f = 0; f < 4; ++f) {
    digitalWrite(FILAS[f], LOW);
    delayMicroseconds(8);
    for (byte c = 0; c < 4; ++c) {
      if (digitalRead(COLUMNAS[c]) == LOW) {
        digitalWrite(FILAS[f], HIGH);
        return TECLAS[f][c];
      }
    }
    digitalWrite(FILAS[f], HIGH);
  }
  return 0;
}

// Un unico evento por pulsacion, cuando la tecla se estabiliza.
char leerTecla() {
  static char anterior = 0, estable = 0;
  static uint32_t desdeCambio = 0;
  char actual = teclaInstantanea();
  if (actual != anterior) {
    anterior = actual;
    desdeCambio = millis();
  }
  if ((uint32_t)(millis() - desdeCambio) >= MS_REBOTE && actual != estable) {
    estable = actual;
    if (estable != 0) return estable;
  }
  return 0;
}

// ----------------------- PANTALLA ----------------------
void mostrar(const __FlashStringHelper *arriba, const __FlashStringHelper *abajo) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(arriba);
  lcd.setCursor(0, 1); lcd.print(abajo);
}
char pinIngresado[5] = {0};
byte cantidadPIN = 0;
byte intentos = 0;
Access::User usuarioActual = {};
byte uidPendiente[10] = {0};
byte longitudPendiente = 0;
bool alarmaPendiente = false;

void mostrarPIN() {
  lcd.setCursor(0, 0);
  lcd.print(F("INGRESE SU PIN  "));
  lcd.setCursor(0, 1);
  for (byte i = 0; i < 4; ++i)
    lcd.print(i < cantidadPIN ? '*' : '_');
  lcd.print(F("            "));
}

void resetearPIN() {
  memset(pinIngresado, 0, sizeof(pinIngresado));
  cantidadPIN = 0;
}

void esperarTarjeta() {
  digitalWrite(PIN_BUZZER, LOW);
  intentos = 0;
  alarmaPendiente = false;
  resetearPIN();
  memset(&usuarioActual, 0, sizeof(usuarioActual));
  cambiarEstado(ESPERANDO_RFID);
  mostrar(F("CONTROL ACCESO"), F("PRESENTE TARJETA"));
}

void solicitarPIN() {
  resetearPIN();
  cambiarEstado(ESPERANDO_PIN);
  mostrarPIN();
}

bool leerTarjetaNueva() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return false;
  longitudPendiente = rfid.uid.size;
  bool valida = Access::validLength(longitudPendiente);
  if (valida) memcpy(uidPendiente, rfid.uid.uidByte, longitudPendiente);
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return valida;
}

bool usuarioRegistrado() {
  Access::User candidato;
  for (byte i = 0; i < Access::USER_COUNT; ++i) {
    if (leerUsuario(i, candidato) && candidato.length == longitudPendiente &&
        memcmp(candidato.uid, uidPendiente, longitudPendiente) == 0) {
      usuarioActual = candidato;
      return true;
    }
  }
  return false;
}

void procesarTeclaPIN(char tecla) {
  if (tecla >= '0' && tecla <= '9') {
    if (cantidadPIN < 4) {
      pinIngresado[cantidadPIN++] = tecla;
      pinIngresado[cantidadPIN] = '\0';
      mostrarPIN();
    }
  } else if (tecla == '*') {
    if (cantidadPIN) --cantidadPIN;
    pinIngresado[cantidadPIN] = '\0';
    mostrarPIN();
  } else if (tecla == '#' && cantidadPIN == 4) {
    cambiarEstado(VALIDANDO_PIN);
    mostrar(F("VALIDANDO PIN"), F("ESPERE..."));
  }
}

void abrirPuerta() {
  puerta.write(ANGULO_ABIERTO);
  cambiarEstado(AUTORIZADO);
  mostrar(F("ACCESO"), F("AUTORIZADO"));
}

void mostrarDenegado(bool porPIN) {
  alarmaPendiente = porPIN && intentos >= MAX_INTENTOS;
  cambiarEstado(DENEGADO);
  if (porPIN) mostrar(F("ACCESO DENEGADO"), F("PIN INCORRECTO"));
  else mostrar(F("TARJETA INVALIDA"), F("ACCESO DENEGADO"));
}

void activarAlarma() {
  cambiarEstado(ALARMA);
  digitalWrite(PIN_BUZZER, HIGH);
  mostrar(F("ACCESO BLOQUEADO"), F("ALARMA ACTIVADA"));
}

void setup() {
  // No Serial.begin(): los pines D0/D1 pertenecen al teclado.
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(RFID_SS, OUTPUT);
  digitalWrite(RFID_SS, HIGH);

  for (byte f = 0; f < 4; ++f) {
    digitalWrite(FILAS[f], HIGH); // evitar pulsos LOW al habilitar OUTPUT
    pinMode(FILAS[f], OUTPUT);
  }
  for (byte c = 0; c < 4; ++c) pinMode(COLUMNAS[c], INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  mostrar(F("SISTEMA ACCESO"), F("INICIANDO..."));

  puerta.attach(PIN_SERVO);
  puerta.write(ANGULO_CERRADO);
  cambiarEstado(INICIO);

  SPI.begin();
  rfid.PCD_Init();
  // EEPROM no se borra ni se reescribe al iniciar.
}

void loop() {
  const uint32_t ahora = millis();
  const char tecla = leerTecla();

  switch (estado) {
    case INICIO:
      if ((uint32_t)(ahora - desdeEstado) >= MS_INICIO) esperarTarjeta();
      break;

    case ESPERANDO_RFID:
      if (leerTarjetaNueva()) {
        cambiarEstado(VALIDANDO_RFID);
        mostrar(F("VALIDANDO RFID"), F("ESPERE..."));
      }
      break;

    case VALIDANDO_RFID:
      if ((uint32_t)(ahora - desdeEstado) >= MS_VALIDACION) {
        if (usuarioRegistrado()) solicitarPIN();
        else mostrarDenegado(false);
      }
      break;

    case ESPERANDO_PIN:
      if (tecla) procesarTeclaPIN(tecla);
      break;

    case VALIDANDO_PIN:
      if ((uint32_t)(ahora - desdeEstado) >= MS_VALIDACION) {
        if (memcmp(pinIngresado, usuarioActual.pin, 4) == 0) {
          abrirPuerta();
        } else {
          ++intentos;
          if (intentos >= MAX_INTENTOS) {
            mostrarDenegado(true);
          } else {
            cambiarEstado(DENEGADO);
            mostrar(F("PIN INCORRECTO"), F("INTENTE DE NUEVO"));
            alarmaPendiente = false;
          }
        }
      }
      break;

    case DENEGADO:
      if ((uint32_t)(ahora - desdeEstado) >= MS_MENSAJE) {
        if (alarmaPendiente) activarAlarma();
        else if (intentos > 0) solicitarPIN();
        else esperarTarjeta();
      }
      break;

    case AUTORIZADO:
      if ((uint32_t)(ahora - desdeEstado) >= MS_MENSAJE) {
        cambiarEstado(PUERTA_ABIERTA); // mismo LED 6 de AUTORIZADO
        mostrar(F("PUERTA ABIERTA"), F("D = CERRAR"));
      }
      // La orden de cierre tambien funciona en el mensaje inicial.
      if (tecla == 'D') {
        puerta.write(ANGULO_CERRADO);
        esperarTarjeta();
      }
      break;

    case PUERTA_ABIERTA:
      if (tecla == 'D') {
        puerta.write(ANGULO_CERRADO);
        esperarTarjeta();
      }
      break;

    case ALARMA:
      // No hay temporizador: suena indefinidamente hasta presionar C.
      if (tecla == 'C') esperarTarjeta();
      break;
  }
}
