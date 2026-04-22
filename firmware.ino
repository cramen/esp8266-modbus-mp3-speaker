/*
 * DFPlayer Mini + ESP8266 NodeMCU — Modbus RTU Slave + OTA по запросу
 *
 * Wi-Fi ВЫКЛЮЧЕН в нормальном режиме работы.
 * Для включения OTA: записать 1 в Holding Register 12 (HR_OTA_MODE).
 * После этого ESP поднимает точку доступа, Modbus отключается.
 * Прошиваем через http://192.168.4.1
 * После прошивки или по таймауту (5 мин) — перезагрузка в нормальный режим.
 *
 * ──────────────────────────────────────────────
 * Схема подключения (с RS-485, MAX485):
 *   ESP8266 D5 (GPIO14)   → MAX485 DE+RE
 *   MAX485 A/B            ↔ Шина RS-485
 *   ESP8266 D7 (GPIO13)   → резистор 1кОм → DFPlayer RX
 *   ESP8266 D6 (GPIO12)   ← DFPlayer TX
 *
 * ──────────────────────────────────────────────
 * Карта Holding Registers:
 *   Addr 0  — RESERVED
 *   Addr 1  — PLAY_CONTROL   RW  1=play
 *   Addr 2  — LOOP_CONTROL   RW  1=loop
 *   Addr 3  — STOP_CONTROL   RW  1=стоп
 *   Addr 4  — TRACK_NUMBER   RW  1..9999
 *   Addr 5  — VOLUME         RW  0..30
 *   Addr 6  — EQ_MODE        RW  0..5
 *   Addr 7  — CFG_BAUD_INDEX RW  0..7
 *   Addr 8  — CFG_PARITY     RW  0..2
 *   Addr 9  — CFG_STOPBITS   RW  1..2
 *   Addr 10 — CFG_SLAVE_ID   RW  1..247
 *   Addr 11 — REBOOT         RW  1=reboot
 *   Addr 12 — OTA_MODE       RW  1=включить Wi-Fi и OTA
 */

#include <SoftwareSerial.h>
#include <DFPlayerMini_Fast.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Updater.h>

// ──────────────────────────────────────────────
//  RS-485
// ──────────────────────────────────────────────
#define USE_RS485
#define RS485_DE_RE_PIN  14    // D5 = GPIO14
// #define WAKEUP_PIN  4   // D2 = GPIO4 — дублирует RX

// ──────────────────────────────────────────────
//  DFPlayer пины
// ──────────────────────────────────────────────
#define DF_RX_PIN        12    // GPIO12 = D6
#define DF_TX_PIN        13    // GPIO13 = D7
#define DF_BAUD          9600

#define STATUS_POLL_MS   500

// ──────────────────────────────────────────────
//  Значения по умолчанию
// ──────────────────────────────────────────────
#define DEFAULT_BAUD_INDEX  7      // 115200
#define DEFAULT_PARITY      0      // None
#define DEFAULT_STOPBITS    2      // 2 стоп-бита
#define DEFAULT_SLAVE_ID    11

// ──────────────────────────────────────────────
//  EEPROM
// ──────────────────────────────────────────────
#define EEPROM_SIZE         32
#define EEPROM_MAGIC_ADDR   0
#define EEPROM_BAUD_ADDR    2
#define EEPROM_PARITY_ADDR  4
#define EEPROM_STOP_ADDR    6
#define EEPROM_SLAVEID_ADDR 8
#define EEPROM_VOLUME_ADDR  10
#define EEPROM_EQ_ADDR      12
#define EEPROM_MAGIC_VALUE  0xA51A

// ──────────────────────────────────────────────
//  Coil регистры
// ──────────────────────────────────────────────
#define COIL_START_ADDR   1
#define COIL_END_ADDR     200
#define NUM_COILS         200

// ──────────────────────────────────────────────
//  Карта регистров
// ──────────────────────────────────────────────
#define NUM_HOLD_REGS    13     // [0]=резерв, [1..12]=рабочие
#define NUM_INPUT_REGS   9

// Holding register индексы — воспроизведение
#define HR_PLAY_CONTROL  1
#define HR_LOOP_CONTROL  2
#define HR_STOP_CONTROL  3
#define HR_TRACK         4
#define HR_VOLUME        5
#define HR_EQ            6

// Holding register индексы — конфигурация Modbus
#define HR_CFG_BAUD      7
#define HR_CFG_PARITY    8
#define HR_CFG_STOPBITS  9
#define HR_CFG_SLAVE_ID  10
#define HR_REBOOT        11
#define HR_OTA_MODE      12    // НОВЫЙ: запись 1 = перейти в режим OTA

#define HR_PLAY_FIRST    HR_PLAY_CONTROL
#define HR_PLAY_LAST     HR_EQ
#define HR_CFG_FIRST     HR_CFG_BAUD
#define HR_CFG_LAST      HR_OTA_MODE

#define STATUS_STOPPED   0
#define STATUS_PLAYING   1
#define STATUS_PAUSED    2

// ──────────────────────────────────────────────
//  OTA настройки
// ──────────────────────────────────────────────
const char* ota_ssid     = "cr_dr_bl";
const char* ota_password = "password";

// Режим работы
enum DeviceMode {
  MODE_NORMAL,    // Modbus + DFPlayer, Wi-Fi выключен
  MODE_OTA        // Wi-Fi включён, Modbus отключён
};

volatile DeviceMode currentMode = MODE_NORMAL;

// volatile bool modbusActivity = false;

// ──────────────────────────────────────────────
//  Таблица скоростей
// ──────────────────────────────────────────────
const unsigned long baudTable[] = {
  1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
};
#define BAUD_TABLE_SIZE  (sizeof(baudTable) / sizeof(baudTable[0]))

// ──────────────────────────────────────────────
//  Объекты
// ──────────────────────────────────────────────
SoftwareSerial dfSerial(DF_RX_PIN, DF_TX_PIN);
DFPlayerMini_Fast dfPlayer;
ESP8266WebServer* otaServer = nullptr;

// ──────────────────────────────────────────────
//  Буферы регистров
// ──────────────────────────────────────────────
uint16_t holdRegs[NUM_HOLD_REGS] = {
  0,                     // [0]  HR_RESERVED
  0,                     // [1]  HR_PLAY_CONTROL
  0,                     // [2]  HR_LOOP_CONTROL
  0,                     // [3]  HR_STOP_CONTROL
  1,                     // [4]  HR_TRACK
  20,                    // [5]  HR_VOLUME
  0,                     // [6]  HR_EQ
  DEFAULT_BAUD_INDEX,    // [7]  HR_CFG_BAUD
  DEFAULT_PARITY,        // [8]  HR_CFG_PARITY
  DEFAULT_STOPBITS,      // [9]  HR_CFG_STOPBITS
  DEFAULT_SLAVE_ID,      // [10] HR_CFG_SLAVE_ID
  0,                     // [11] HR_REBOOT
  0                      // [12] HR_OTA_MODE
};

uint16_t inputRegs[NUM_INPUT_REGS] = {0};
uint16_t prevHoldRegs[NUM_HOLD_REGS];

#define COIL_QUEUE_SIZE  8
uint16_t coilTrackQueue[COIL_QUEUE_SIZE];
volatile uint8_t coilQueueHead = 0;
volatile uint8_t coilQueueTail = 0;

uint16_t activeBaudIndex;
uint16_t activeParity;
uint16_t activeStopBits;
uint16_t activeSlaveId;

unsigned long lastStatusPoll = 0;
bool dfReady = false;

#define MODBUS_BUF_SIZE   128
uint8_t  mbBuf[MODBUS_BUF_SIZE];
uint8_t  mbLen = 0;
unsigned long lastByteTime = 0;
#define MODBUS_TIMEOUT_US 4000

// ──────────────────────────────────────────────
//  OTA HTML страница
// ──────────────────────────────────────────────
const char uploadPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>OTA Update</title>
  <style>
    body { font-family: Arial; background: #1a1a2e; color: #eee;
           display: flex; justify-content: center; align-items: center;
           min-height: 100vh; margin: 0; }
    .c { background: #16213e; padding: 40px; border-radius: 12px;
         text-align: center; max-width: 420px; width: 90%; }
    h1 { color: #e94560; }
    p { color: #aaa; font-size: 14px; }
    input[type="file"] { margin: 20px 0; padding: 10px; background: #0f3460;
         border: 2px dashed #e94560; border-radius: 8px; color: #eee;
         width: 100%; box-sizing: border-box; }
    .btn { color: white; border: none; padding: 12px 40px;
         border-radius: 8px; font-size: 16px; cursor: pointer;
         width: 100%; margin: 5px 0; display: block; }
    .btn-flash { background: #e94560; }
    .btn-flash:hover { background: #c73652; }
    .btn-reboot { background: #0f3460; margin-top: 20px; }
    .btn-reboot:hover { background: #1a4a8a; }
    .bar-bg { background: #0f3460; border-radius: 8px; overflow: hidden;
         height: 24px; margin-top: 15px; display: none; }
    .bar { background: #e94560; height: 100%; width: 0%; transition: width 0.3s;
         text-align: center; line-height: 24px; font-size: 12px; }
    #status { margin-top: 10px; }
    .warn { color: #f39c12; font-size: 12px; margin-top: 20px; }
    hr { border: 0; border-top: 1px solid #2a3a5e; margin: 25px 0; }
  </style>
</head>
<body>
  <div class="c">
    <h1>&#128268; OTA Update</h1>
    <p>Modbus отключён. Выберите .bin файл прошивки.</p>
    <p>Таймаут: 5 минут, затем автоперезагрузка.</p>

    <form id="f" method="POST" action="/update" enctype="multipart/form-data">
      <input type="file" name="firmware" accept=".bin" required id="file">
      <button type="submit" class="btn btn-flash" id="btn">Прошить</button>
    </form>

    <div class="bar-bg" id="pbg"><div class="bar" id="bar">0%</div></div>
    <div id="status"></div>

    <hr>

    <button class="btn btn-reboot" id="rebootBtn">
      &#x21bb; Перезагрузить без обновления
    </button>

    <div class="warn">
      После прошивки или перезагрузки устройство вернётся<br>
      в нормальный режим. Wi-Fi отключится, Modbus возобновится.
    </div>
  </div>

  <script>
    // Загрузка прошивки с прогрессом
    document.getElementById('f').addEventListener('submit', function(e) {
      e.preventDefault();
      var f = document.getElementById('file').files[0];
      if (!f) return;
      var fd = new FormData();
      fd.append('firmware', f);
      var x = new XMLHttpRequest();
      var bar = document.getElementById('bar');
      var pbg = document.getElementById('pbg');
      var st = document.getElementById('status');
      var btn = document.getElementById('btn');
      pbg.style.display = 'block';
      btn.disabled = true;
      btn.textContent = 'Загрузка...';
      document.getElementById('rebootBtn').disabled = true;
      x.upload.addEventListener('progress', function(e) {
        if (e.lengthComputable) {
          var p = Math.round(e.loaded / e.total * 100);
          bar.style.width = p + '%';
          bar.textContent = p + '%';
        }
      });
      x.addEventListener('load', function() {
        if (x.status === 200) {
          st.textContent = 'Прошивка загружена! Перезагрузка...';
          bar.style.width = '100%';
          bar.style.background = '#2ecc71';
        } else {
          st.textContent = 'Ошибка: ' + x.responseText;
          bar.style.background = '#e74c3c';
          btn.disabled = false;
          btn.textContent = 'Прошить';
          document.getElementById('rebootBtn').disabled = false;
        }
      });
      x.addEventListener('error', function() {
        st.textContent = 'Ошибка соединения';
        btn.disabled = false;
        btn.textContent = 'Прошить';
        document.getElementById('rebootBtn').disabled = false;
      });
      x.open('POST', '/update');
      x.send(fd);
    });

    // Перезагрузка без обновления
    document.getElementById('rebootBtn').addEventListener('click', function() {
      this.disabled = true;
      this.textContent = 'Перезагрузка...';
      document.getElementById('btn').disabled = true;
      document.getElementById('status').textContent = 'Перезагрузка в нормальный режим...';
      fetch('/reboot', { method: 'POST' })
        .then(function() {})
        .catch(function() {});
    });
  </script>
</body>
</html>
)rawliteral";

// void ICACHE_RAM_ATTR onRxWakeup() {
//   modbusActivity = true;
// }

// ──────────────────────────────────────────────
//  Очередь coil-треков
// ──────────────────────────────────────────────
void coilQueuePush(uint16_t trackNum) {
  uint8_t nextHead = (coilQueueHead + 1) % COIL_QUEUE_SIZE;
  if (nextHead != coilQueueTail) {   // не переполнена
    coilTrackQueue[coilQueueHead] = trackNum;
    coilQueueHead = nextHead;
  }
}

bool coilQueuePop(uint16_t &trackNum) {
  if (coilQueueTail == coilQueueHead) return false;
  trackNum = coilTrackQueue[coilQueueTail];
  coilQueueTail = (coilQueueTail + 1) % COIL_QUEUE_SIZE;
  return true;
}

// ──────────────────────────────────────────────
//  Расчёт CRC16 (Modbus)
// ──────────────────────────────────────────────
uint16_t modbusCalcCRC(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else               { crc >>= 1; }
    }
  }
  return crc;
}

// Отправка ответа с CRC
void modbusSendResponse(uint8_t *resp, uint8_t len) {
  uint16_t crc  = modbusCalcCRC(resp, len);
  resp[len]     = crc & 0xFF;
  resp[len + 1] = crc >> 8;

  digitalWrite(RS485_DE_RE_PIN, HIGH); // переключаемся на передачу
  delayMicroseconds(100);
  Serial.write(resp, len + 2);
  Serial.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // обратно в режим приёма
}

// Отправка ошибки
void modbusSendError(uint8_t func, uint8_t code) {
  uint8_t resp[5];
  resp[0] = activeSlaveId;
  resp[1] = func | 0x80;
  resp[2] = code;
  modbusSendResponse(resp, 3);
}

// ──────────────────────────────────────────────
//  FC01 — чтение Coils (всегда возвращает 0)
// ──────────────────────────────────────────────
void modbusHandleFC01(uint8_t *req) {
  uint16_t startAddr = ((uint16_t)req[2] << 8) | req[3];
  uint16_t quantity  = ((uint16_t)req[4] << 8) | req[5];

  // Проверяем диапазон: допустимые адреса 1..20
  if (quantity == 0 || quantity > 2000 ||
      startAddr < COIL_START_ADDR ||
      startAddr + quantity - 1 > COIL_END_ADDR) {
    modbusSendError(0x01, 0x02);
    return;
  }

  // Количество байт данных (coils упакованы по 8 в байт)
  uint8_t byteCount = (quantity + 7) / 8;

  uint8_t resp[3 + byteCount + 2];
  resp[0] = activeSlaveId;
  resp[1] = 0x01;
  resp[2] = byteCount;

  // Все coils = 0, заполняем нулями
  for (uint8_t i = 0; i < byteCount; i++) {
    resp[3 + i] = 0x00;
  }

  modbusSendResponse(resp, 3 + byteCount);
}

// ──────────────────────────────────────────────
//  FC05 — запись одного Coil
//  Значение 0xFF00 = ON (запуск трека), 0x0000 = OFF (игнорируем)
// ──────────────────────────────────────────────
void modbusHandleFC05(uint8_t *req) {
  uint16_t addr  = ((uint16_t)req[2] << 8) | req[3];
  uint16_t value = ((uint16_t)req[4] << 8) | req[5];

  // Проверяем адрес: допустимые 1..20
  if (addr < COIL_START_ADDR || addr > COIL_END_ADDR) {
    modbusSendError(0x05, 0x02);
    return;
  }

  // Проверяем значение: должно быть 0xFF00 или 0x0000
  if (value != 0xFF00 && value != 0x0000) {
    modbusSendError(0x05, 0x03);
    return;
  }

  // Если записано ON — ставим трек в очередь
  if (value == 0xFF00) {
    coilQueuePush(addr);  // номер трека = адрес coil
  }

  // Echo-ответ (повтор запроса)
  uint8_t resp[8];
  resp[0] = activeSlaveId;
  resp[1] = 0x05;
  resp[2] = req[2];
  resp[3] = req[3];
  resp[4] = req[4];
  resp[5] = req[5];
  modbusSendResponse(resp, 6);
}

// ──────────────────────────────────────────────
//  FC15 (0x0F) — запись нескольких Coils
// ──────────────────────────────────────────────
void modbusHandleFC15(uint8_t *req) {
  uint16_t startAddr = ((uint16_t)req[2] << 8) | req[3];
  uint16_t quantity  = ((uint16_t)req[4] << 8) | req[5];
  uint8_t  byteCount = req[6];

  // Проверяем диапазон
  if (quantity == 0 || quantity > 2000 ||
      startAddr < COIL_START_ADDR ||
      startAddr + quantity - 1 > COIL_END_ADDR ||
      byteCount != (quantity + 7) / 8) {
    modbusSendError(0x0F, 0x02);
    return;
  }

  // Обрабатываем каждый coil
  for (uint16_t i = 0; i < quantity; i++) {
    uint8_t byteIdx = i / 8;
    uint8_t bitIdx  = i % 8;
    bool coilOn = (req[7 + byteIdx] >> bitIdx) & 0x01;

    if (coilOn) {
      uint16_t trackNum = startAddr + i;
      coilQueuePush(trackNum);
    }
  }

  // Ответ: slave, func, startAddr(2), quantity(2)
  uint8_t resp[8];
  resp[0] = activeSlaveId;
  resp[1] = 0x0F;
  resp[2] = startAddr >> 8;
  resp[3] = startAddr & 0xFF;
  resp[4] = quantity >> 8;
  resp[5] = quantity & 0xFF;
  modbusSendResponse(resp, 6);
}

// ──────────────────────────────────────────────
//  FC03 — чтение Holding Registers
// ──────────────────────────────────────────────
void modbusHandleFC03(uint8_t *req) {
  uint16_t startAddr = ((uint16_t)req[2] << 8) | req[3];
  uint16_t quantity  = ((uint16_t)req[4] << 8) | req[5];

  if (quantity == 0 || quantity > 125 ||
      startAddr + quantity > NUM_HOLD_REGS) {
    modbusSendError(0x03, 0x02);
    return;
  }
  uint8_t resp[3 + quantity * 2 + 2];
  resp[0] = activeSlaveId;
  resp[1] = 0x03;
  resp[2] = quantity * 2;
  for (uint16_t i = 0; i < quantity; i++) {
    resp[3 + i * 2]     = holdRegs[startAddr + i] >> 8;
    resp[3 + i * 2 + 1] = holdRegs[startAddr + i] & 0xFF;
  }
  modbusSendResponse(resp, 3 + quantity * 2);
}

// ──────────────────────────────────────────────
//  FC04 — чтение Input Registers
// ──────────────────────────────────────────────
void modbusHandleFC04(uint8_t *req) {
  uint16_t startAddr = ((uint16_t)req[2] << 8) | req[3];
  uint16_t quantity  = ((uint16_t)req[4] << 8) | req[5];

  if (quantity == 0 || quantity > 125 ||
      startAddr + quantity > NUM_INPUT_REGS) {
    modbusSendError(0x04, 0x02);
    return;
  }
  uint8_t resp[3 + quantity * 2 + 2];
  resp[0] = activeSlaveId;
  resp[1] = 0x04;
  resp[2] = quantity * 2;
  for (uint16_t i = 0; i < quantity; i++) {
    resp[3 + i * 2]     = inputRegs[startAddr + i] >> 8;
    resp[3 + i * 2 + 1] = inputRegs[startAddr + i] & 0xFF;
  }
  modbusSendResponse(resp, 3 + quantity * 2);
}

// ──────────────────────────────────────────────
//  FC06 — запись одного Holding Register
// ──────────────────────────────────────────────
void modbusHandleFC06(uint8_t *req) {
  uint16_t addr  = ((uint16_t)req[2] << 8) | req[3];
  uint16_t value = ((uint16_t)req[4] << 8) | req[5];

  if (addr >= NUM_HOLD_REGS) {
    modbusSendError(0x06, 0x02);
    return;
  }
  holdRegs[addr] = value;

  // Echo-ответ (6 байт данных + 2 CRC)
  uint8_t resp[8];
  resp[0] = activeSlaveId;
  resp[1] = 0x06;
  resp[2] = req[2];
  resp[3] = req[3];
  resp[4] = req[4];
  resp[5] = req[5];
  modbusSendResponse(resp, 6);
}

// ──────────────────────────────────────────────
//  FC16 — запись нескольких Holding Registers
// ──────────────────────────────────────────────
void modbusHandleFC16(uint8_t *req) {
  uint16_t startAddr = ((uint16_t)req[2] << 8) | req[3];
  uint16_t quantity  = ((uint16_t)req[4] << 8) | req[5];

  if (quantity == 0 || quantity > 123 ||
      startAddr + quantity > NUM_HOLD_REGS) {
    modbusSendError(0x10, 0x02);
    return;
  }
  for (uint16_t i = 0; i < quantity; i++) {
    holdRegs[startAddr + i] = ((uint16_t)req[7 + i * 2] << 8)
                            |            req[8 + i * 2];
  }
  uint8_t resp[8];
  resp[0] = activeSlaveId;
  resp[1] = 0x10;
  resp[2] = startAddr >> 8;
  resp[3] = startAddr & 0xFF;
  resp[4] = quantity >> 8;
  resp[5] = quantity & 0xFF;
  modbusSendResponse(resp, 6);
}

// ──────────────────────────────────────────────
//  Обработка принятого пакета
// ──────────────────────────────────────────────
void modbusProcess() {
  if (mbLen < 4) { mbLen = 0; return; }

  // Проверка адреса
  if (mbBuf[0] != activeSlaveId) {
    mbLen = 0;
    return;
  }

  // Проверка CRC
  uint16_t crcCalc = modbusCalcCRC(mbBuf, mbLen - 2);
  uint16_t crcRecv = (uint16_t)mbBuf[mbLen - 2] |
                     ((uint16_t)mbBuf[mbLen - 1] << 8);
  if (crcCalc != crcRecv) {
    mbLen = 0;
    return;
  }

  uint8_t fc = mbBuf[1];
  switch (fc) {
    case 0x01: modbusHandleFC01(mbBuf); break;   // Read Coils
    case 0x03: modbusHandleFC03(mbBuf); break;   // Read Holding Registers
    case 0x04: modbusHandleFC04(mbBuf); break;   // Read Input Registers
    case 0x05: modbusHandleFC05(mbBuf); break;   // Write Single Coil
    case 0x06: modbusHandleFC06(mbBuf); break;   // Write Single Register
    case 0x0F: modbusHandleFC15(mbBuf); break;   // Write Multiple Coils
    case 0x10: modbusHandleFC16(mbBuf); break;   // Write Multiple Registers
    default:   modbusSendError(fc, 0x01); break;  // Illegal Function
  }
  mbLen = 0;
}


// ──────────────────────────────────────────────
//  EEPROM
// ──────────────────────────────────────────────
void eepromWriteU16(int addr, uint16_t value) {
  EEPROM.write(addr,     (uint8_t)(value >> 8));
  EEPROM.write(addr + 1, (uint8_t)(value & 0xFF));
}

uint16_t eepromReadU16(int addr) {
  return ((uint16_t)EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
}

void loadConfigFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  uint16_t magic = eepromReadU16(EEPROM_MAGIC_ADDR);

  if (magic == EEPROM_MAGIC_VALUE) {
    activeBaudIndex = eepromReadU16(EEPROM_BAUD_ADDR);
    activeParity    = eepromReadU16(EEPROM_PARITY_ADDR);
    activeStopBits  = eepromReadU16(EEPROM_STOP_ADDR);
    activeSlaveId   = eepromReadU16(EEPROM_SLAVEID_ADDR);
    holdRegs[HR_VOLUME] = eepromReadU16(EEPROM_VOLUME_ADDR);
    holdRegs[HR_EQ]     = eepromReadU16(EEPROM_EQ_ADDR);

    if (activeBaudIndex >= BAUD_TABLE_SIZE)        activeBaudIndex = DEFAULT_BAUD_INDEX;
    if (activeParity > 2)                          activeParity    = DEFAULT_PARITY;
    if (activeStopBits < 1 || activeStopBits > 2)  activeStopBits  = DEFAULT_STOPBITS;
    if (activeSlaveId < 1  || activeSlaveId > 247) activeSlaveId   = DEFAULT_SLAVE_ID;
    holdRegs[HR_VOLUME] = constrain(holdRegs[HR_VOLUME], 0, 30);
    holdRegs[HR_EQ]     = constrain(holdRegs[HR_EQ], 0, 5);
  } else {
    activeBaudIndex = DEFAULT_BAUD_INDEX;
    activeParity    = DEFAULT_PARITY;
    activeStopBits  = DEFAULT_STOPBITS;
    activeSlaveId   = DEFAULT_SLAVE_ID;

    eepromWriteU16(EEPROM_MAGIC_ADDR,   EEPROM_MAGIC_VALUE);
    eepromWriteU16(EEPROM_BAUD_ADDR,    activeBaudIndex);
    eepromWriteU16(EEPROM_PARITY_ADDR,  activeParity);
    eepromWriteU16(EEPROM_STOP_ADDR,    activeStopBits);
    eepromWriteU16(EEPROM_SLAVEID_ADDR, activeSlaveId);
    eepromWriteU16(EEPROM_VOLUME_ADDR,  holdRegs[HR_VOLUME]);
    eepromWriteU16(EEPROM_EQ_ADDR,      holdRegs[HR_EQ]);
    EEPROM.commit();
  }

  holdRegs[HR_CFG_BAUD]     = activeBaudIndex;
  holdRegs[HR_CFG_PARITY]   = activeParity;
  holdRegs[HR_CFG_STOPBITS] = activeStopBits;
  holdRegs[HR_CFG_SLAVE_ID] = activeSlaveId;
}

void saveConfigToEEPROM() {
  eepromWriteU16(EEPROM_BAUD_ADDR,    holdRegs[HR_CFG_BAUD]);
  eepromWriteU16(EEPROM_PARITY_ADDR,  holdRegs[HR_CFG_PARITY]);
  eepromWriteU16(EEPROM_STOP_ADDR,    holdRegs[HR_CFG_STOPBITS]);
  eepromWriteU16(EEPROM_SLAVEID_ADDR, holdRegs[HR_CFG_SLAVE_ID]);
  eepromWriteU16(EEPROM_MAGIC_ADDR,   EEPROM_MAGIC_VALUE);
  EEPROM.commit();
}

void savePlayerParamsToEEPROM() {
  eepromWriteU16(EEPROM_VOLUME_ADDR,  holdRegs[HR_VOLUME]);
  eepromWriteU16(EEPROM_EQ_ADDR,      holdRegs[HR_EQ]);
  EEPROM.commit();
}

SerialConfig buildSerialConfig(uint16_t parity, uint16_t stopBits) {
  if (parity == 1 && stopBits == 1) return SERIAL_8E1;
  if (parity == 1 && stopBits == 2) return SERIAL_8E2;
  if (parity == 2 && stopBits == 1) return SERIAL_8O1;
  if (parity == 2 && stopBits == 2) return SERIAL_8O2;
  if (parity == 0 && stopBits == 2) return SERIAL_8N2;
  return SERIAL_8N1;
}

// ──────────────────────────────────────────────
//  DFPlayer управление
// ──────────────────────────────────────────────
uint16_t getTrackNum() {
  return constrain(holdRegs[HR_TRACK], 1, 9999);
}

void startPlayback() { dfPlayer.play(getTrackNum()); }

void updateVolume() {
  holdRegs[HR_VOLUME] = constrain(holdRegs[HR_VOLUME], 0, 30);
  dfPlayer.volume(holdRegs[HR_VOLUME]);
}

void updateEQ() {
  holdRegs[HR_EQ] = constrain(holdRegs[HR_EQ], 0, 5);
  dfPlayer.EQSelect(holdRegs[HR_EQ]);
}

void startLoop() { dfPlayer.loop(getTrackNum()); }

void restore() {
  dfPlayer.reset();
  delay(100);
  updateVolume();
  updateEQ();
}

void applyDFPlayerChanges() {

  bool needSave = false;

  // Громкость
  if (holdRegs[HR_VOLUME] != prevHoldRegs[HR_VOLUME]) {
    updateVolume();
    needSave = true;
  }

  // Эквалайзер
  if (holdRegs[HR_EQ] != prevHoldRegs[HR_EQ]) {
    updateEQ();
    needSave = true;
  }

  // Команда воспроизведения
  if (holdRegs[HR_PLAY_CONTROL]) {
    startPlayback();
    holdRegs[HR_PLAY_CONTROL] = 0;
  }

  // Команда loop
  if (holdRegs[HR_LOOP_CONTROL]) {
    startLoop();
    holdRegs[HR_LOOP_CONTROL] = 0;
  }

  // Команда остановки
  if (holdRegs[HR_STOP_CONTROL]) {
    dfPlayer.stop();
    restore();
    holdRegs[HR_STOP_CONTROL] = 0;
  }

  // Обработка очереди Coil-треков (запускаем последний из очереди)
  uint16_t coilTrack = 0;
  uint16_t tmp;
  while (coilQueuePop(tmp)) {
    coilTrack = tmp;  // берём последний добавленный
  }
  if (coilTrack > 0 && dfReady) {
    dfPlayer.play(coilTrack);
  }

  // Обновляем теневые копии воспроизведения
  for (int i = HR_PLAY_FIRST; i <= HR_PLAY_LAST; i++) {
    prevHoldRegs[i] = holdRegs[i];
  }

  if (needSave) {
    savePlayerParamsToEEPROM();
  }
}

// ──────────────────────────────────────────────
//  Переключение в режим OTA
// ──────────────────────────────────────────────
void enterOTAMode() {
  currentMode = MODE_OTA;

  // Останавливаем DFPlayer
  if (dfReady) {
    dfPlayer.stop();
    // dfPlayer.sleep();
  }

  // === ПОЛНОСТЬЮ ОТКЛЮЧАЕМ UART0 (Modbus) ===
  Serial.end();

  // Переводим пины UART в безопасное состояние
  // чтобы не мешать шине RS-485
  pinMode(1, INPUT);   // GPIO1 = TX — в высокоимпедансное
  pinMode(3, INPUT);   // GPIO3 = RX — в высокоимпедансное

  // DE/RE в LOW — приёмник (не занимаем шину)
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // Пробуждаем Wi-Fi
  WiFi.forceSleepWake();
  delay(10);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ota_ssid, ota_password);

  // Создаём веб-сервер
  otaServer = new ESP8266WebServer(80);

  // Главная страница
  otaServer->on("/", HTTP_GET, []() {
    otaServer->send(200, "text/html", uploadPage);
  });

  // Загрузка прошивки
  otaServer->on("/update", HTTP_POST,
    []() {
      if (Update.hasError()) {
        otaServer->send(500, "text/plain",
          String("Error: ") + Update.getErrorString());
      } else {
        otaServer->send(200, "text/plain", "OK. Rebooting...");
        delay(1000);
        ESP.restart();
      }
    },
    []() {
      HTTPUpload& upload = otaServer->upload();
      if (upload.status == UPLOAD_FILE_START) {
        uint32_t maxSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        Update.begin(maxSpace);
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        Update.write(upload.buf, upload.currentSize);
      } else if (upload.status == UPLOAD_FILE_END) {
        Update.end(true);
      }
      yield();
    }
  );

  // === НОВЫЙ ЭНДПОИНТ: перезагрузка без обновления ===
  otaServer->on("/reboot", HTTP_POST, []() {
    otaServer->send(200, "text/plain", "Rebooting...");
    delay(500);
    ESP.restart();
  });

  otaServer->begin();
}


// ──────────────────────────────────────────────
//  Обработка конфигурационных регистров
// ──────────────────────────────────────────────
void applyConfigChanges() {
  bool changed = false;

  if (holdRegs[HR_CFG_BAUD] != prevHoldRegs[HR_CFG_BAUD]) {
    if (holdRegs[HR_CFG_BAUD] >= BAUD_TABLE_SIZE)
      holdRegs[HR_CFG_BAUD] = prevHoldRegs[HR_CFG_BAUD];
    else changed = true;
  }
  if (holdRegs[HR_CFG_PARITY] != prevHoldRegs[HR_CFG_PARITY]) {
    if (holdRegs[HR_CFG_PARITY] > 2)
      holdRegs[HR_CFG_PARITY] = prevHoldRegs[HR_CFG_PARITY];
    else changed = true;
  }
  if (holdRegs[HR_CFG_STOPBITS] != prevHoldRegs[HR_CFG_STOPBITS]) {
    if (holdRegs[HR_CFG_STOPBITS] < 1 || holdRegs[HR_CFG_STOPBITS] > 2)
      holdRegs[HR_CFG_STOPBITS] = prevHoldRegs[HR_CFG_STOPBITS];
    else changed = true;
  }
  if (holdRegs[HR_CFG_SLAVE_ID] != prevHoldRegs[HR_CFG_SLAVE_ID]) {
    if (holdRegs[HR_CFG_SLAVE_ID] < 1 || holdRegs[HR_CFG_SLAVE_ID] > 247)
      holdRegs[HR_CFG_SLAVE_ID] = prevHoldRegs[HR_CFG_SLAVE_ID];
    else changed = true;
  }

  if (changed) saveConfigToEEPROM();

  // Reboot
  if (holdRegs[HR_REBOOT] != prevHoldRegs[HR_REBOOT] && holdRegs[HR_REBOOT] == 1) {
    delay(50);
    ESP.restart();
  }
  holdRegs[HR_REBOOT] = 0;

  // OTA MODE — переключение
  if (holdRegs[HR_OTA_MODE] != prevHoldRegs[HR_OTA_MODE] && holdRegs[HR_OTA_MODE] == 1) {
    // Отправляем Modbus-ответ до переключения (уже отправлен в FC06/FC16)
    delay(50);
    enterOTAMode();
    return;  // выходим, дальше работает OTA loop
  }
  holdRegs[HR_OTA_MODE] = 0;

  for (int i = HR_CFG_FIRST; i <= HR_CFG_LAST; i++)
    prevHoldRegs[i] = holdRegs[i];
}

// ──────────────────────────────────────────────
//  setup
// ──────────────────────────────────────────────
void setup() {
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // ВАЖНО: Wi-Fi полностью выключен при старте
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  // Отключаем любой отладочный вывод в UART
  Serial.setDebugOutput(false);

  loadConfigFromEEPROM();

  unsigned long baud = baudTable[activeBaudIndex];
  SerialConfig serialCfg = buildSerialConfig(activeParity, activeStopBits);
  Serial.begin(baud, serialCfg);
  Serial.setTimeout(1);

  // Пин-детектор активности на шине
  // pinMode(WAKEUP_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), onRxWakeup, FALLING);

  dfSerial.begin(DF_BAUD);
  delay(1000);

  if (dfPlayer.begin(dfSerial)) {
    dfReady = true;
    delay(200);
    dfPlayer.startDAC();
    delay(200);
  }

  memcpy(prevHoldRegs, holdRegs, sizeof(holdRegs));

  if (dfReady) {
    dfPlayer.volume(holdRegs[HR_VOLUME]);
    dfPlayer.EQSelect(holdRegs[HR_EQ]);
  }

  currentMode = MODE_NORMAL;
}

// ──────────────────────────────────────────────
//  loop
// ──────────────────────────────────────────────
void loop() {

  // ===== РЕЖИМ OTA =====
  if (currentMode == MODE_OTA) {
    // Serial отключён, Modbus не слушаем
    // Только обрабатываем HTTP
    if (otaServer) {
       otaServer->handleClient();
    }
    return;  // НИЧЕГО больше не делаем
  }

    // Нет активности и нет незавершённого пакета — спим
    // if (!modbusActivity && mbLen == 0) {
    //   delay(1);
    //   return;    // Почти ничего не делаем
    // }
    // modbusActivity = false;

  // ===== НОРМАЛЬНЫЙ РЕЖИМ =====
  while (Serial.available()) {
    if (mbLen < MODBUS_BUF_SIZE) {
      mbBuf[mbLen++] = (uint8_t)Serial.read();
    } else {
      Serial.read();
    }
    lastByteTime = micros();
  }

  if (mbLen > 0 && (micros() - lastByteTime) > MODBUS_TIMEOUT_US) {
      modbusProcess();
      applyDFPlayerChanges();
      applyConfigChanges();
  }
}
