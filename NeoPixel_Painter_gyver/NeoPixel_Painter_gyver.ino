/*
   РАСШИФРОВКА НАДПИСЕЙ НА ДИСПЛЕЕ:
   Strt - (Start) начало работы, инициализация SD карты. Эту надпись можно успеть
   прочитать только при запуске в режиме загрузки
   PreP - (Perparing) подготовка к преобразованию изображений
   LOAd - (Loading) преобразование и загрузка изображений
   read - (Reading) чтение преобразованных изображений с карты
   rdy  - (Ready) готов к работе
   Proc - (Processing) показываем анимацию

   НАСТРОЙКИ:
   Br** - (Brightness) настройка яркости (в процентах, 10-95)
   Fr** - (Frame) номер изображения, которое будет показано при нажатии на кнопку
   SP** - (Speed) скорость анимации (в процентах, 0-95)

   ОШИБКИ:
   Sder - (SD error) ошибка подключения SD карты (карта не подключена, карта сломана, карта не подходит)
   FrEr - (Frame Error) ошибка при чтении картинки. Возникает на разных стадиях, подробнее видно с компьютера
   Ferr - (Fatal error) критическая ошибка, работа остановлена. Возникает на разных стадиях, подробнее видно с компьютера
*/

// ------------------------------ НАСТРОЙКИ ------------------------------
#define N_LEDS       144    // количество светодиодов (максимум 170 !!!)
#define FLIP_H 0            // зеркально отражать изображения по горизонтали
#define CURRENT_MAX 3500    // максимальный ток (авто-ограничение по расчёту яркости)
#define DEBUG 0             // отправка информации в порт, 1-вкл, 0-выкл
// ПРИ ВЫКЛЮЧЕННОМ DEBUG СИСТЕМА РАБОТАЕТ ГОРАЗДО БЫСТРЕЕ!!
// ------------------------------ НАСТРОЙКИ ------------------------------

// ----------------------- ПОДКЛЮЧЕНИЕ -----------------------
#define TRIGGER 2  // кнопка запуска анимации

#define CLK_ENC 3  // ПИН CLK ЭНКОДЕРА
#define DT_ENC 4   // ПИН DT ЭНКОДЕРА
// CLK и DT можно менять местами, чтобы инвертировать направление

#define SW_ENC 5   // ПИН SW ЭНКОДЕРА

#define DIO 6      // ДИСПЛЕЙ
#define CLK 7      // ДИСПЛЕЙ

#define LED_PIN 8  // пин DIN ленты

// SD card module: CLK (SCK) -> D13, MOSI -> D11, MISO -> D12, CS -> D10
#define CARD_SELECT   10    // CS пин карты
// ----------------------- ПОДКЛЮЧЕНИЕ -----------------------

// ---- БИБЛИОТЕКИ ----
#include <SdFat.h>
#include <avr/pgmspace.h>
#include "./gamma.h"
#include "TM1637.h"
#include "EEPROMex.h"
TM1637 tm1637(CLK, DIO);
// ---- БИБЛИОТЕКИ ----

// ----------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------
byte NORM_STEP = 1;   // шаг изменения переменной norm_counter при вращении
#define HOLD_STEP 5   // шаг изменения переменной hold_counter при нажатии, удерживании и вращении

#define OVERHEAD 150 // Extra microseconds for loop processing, etc.
uint8_t           sdBuf[512],  // One SD block (also for NeoPixel color data)
                  pinMask;     // NeoPixel pin bitmask
uint16_t          maxLPS,      // Max playback lines/sec
                  nFrames = 0, // Total # of image files
                  frame   = 0; // Current image # being painted
uint32_t          firstBlock,  // First block # in temp working file
                  nBlocks;     // Number of blocks in file
SdFat             sd;          // SD filesystem
volatile uint8_t *port;        // NeoPixel PORT register
boolean bmpProcess(char *inName, char *outName, uint8_t *brightness);

uint32_t block    = 0;     // Current block # within file
boolean  stopFlag = false; // If set, stop playback loop
uint32_t lastBlock;
char     infile[13];
SdFile   tmp;

char norm_counter, hold_counter;
boolean DT_now, DT_last, SW_state, turn_flag, SWstate, SWlast;
byte anim_speed;
// ----------------------- ДЛЯ РАЗРАБОТЧИКОВ -----------------------

void setup() {
  Serial.begin(57600);
  if (EEPROM.read(0) != 1) {  // если это САМЫЙ ПЕРВЫЙ запуск
    EEPROM.write(0, 1);       // запомнить, что первый запуск был
    EEPROM.writeByte(1, 50);      // ячейка 1 - яркость (по умолчанию)
    EEPROM.writeByte(3, 50);      // ячейка 3 - скорость (по умолчанию)
    if (DEBUG) Serial.println("first");
    norm_counter = 50;
    anim_speed = 50;
  } else {
    norm_counter = EEPROM.readByte(1);  // читаем яркость
    anim_speed = EEPROM.readByte(3);    // читаем скорость
  }

  tm1637.init();  // инициализация
  tm1637.set(7);  // яркость, 0 - 7 (минимум - максимум)
  tm1637.displayByte(_S, _t, _r, _t);

  pinMode(CLK_ENC, INPUT);
  pinMode(DT_ENC, INPUT);
  pinMode(SW_ENC, INPUT_PULLUP);
  DT_last = digitalRead(CLK);

  uint8_t  b, startupTrigger, minBrightness;
  char     infile[13], outfile[13];
  boolean  found;
  uint16_t i, n;
  SdFile   tmp;
  uint32_t lastBlock;

  pinMode(TRIGGER, INPUT_PULLUP);        // Enable pullup on trigger button
  startupTrigger = digitalRead(SW_ENC);      // Poll startup trigger ASAP

  pinMode(LED_PIN, OUTPUT);              // Enable NeoPixel output
  digitalWrite(LED_PIN, LOW);            // Default logic state = low
  port    = portOutputRegister(digitalPinToPort(LED_PIN));
  pinMask = digitalPinToBitMask(LED_PIN);
  memset(sdBuf, 0, N_LEDS * 3);          // Clear LED buffer
  show();                                // Init LEDs to 'off' state

  if (DEBUG) Serial.print(F("Initializing SD card..."));
  if (!sd.begin(CARD_SELECT, SPI_FULL_SPEED)) {
    tm1637.displayByte(_S, _d, _e, _r);  // надпись Sder
    error(F("failed. Things to check:\n"
            "* is a card is inserted?\n"
            "* Is your wiring correct?\n"
            "* did you edit CARD_SELECT to match the SD shield or module?"));
  }
  if (DEBUG) Serial.println(F("OK"));

#if 0
  if (!volume.init(&card)) {
    tm1637.displayByte(_S, _d, _e, _r);  // надпись Sder
    error(F("Could not find FAT16/FAT32 partition.\n"
            "Make sure the card is formatted."));
  }
  root.openRoot(&volume);
#endif
  if (startupTrigger == LOW) { // если при запуске кнопка НАЖАТА
    while (digitalRead(SW_ENC) == LOW); // ждём отпускания кнопки
    tm1637.displayByte(_P, _r, _e, _P);

    minBrightness = 255;
    do {
      sprintf(infile, "frame%03d.bmp", nFrames);
      b = 255; // Assume frame at full brightness to start...
      // ...it's then modified by the bmpProcess() function here to the
      // actual brightness limit the UBEC can sustain for *this image*.
      if (found = bmpProcess(infile, NULL, &b)) {
        nFrames++;
        if (b < minBrightness) minBrightness = b;
      }
    } while (found && (nFrames < 1000));

    if (DEBUG) Serial.print(nFrames);
    if (DEBUG) Serial.print(" frames\nbrightness = ");
    if (DEBUG) Serial.println(minBrightness);

    tm1637.displayByte(0, _b);
    tm1637.displayByte(1, _r);
    NORM_STEP = 5;
    tm1637.displayInt(norm_counter);
    for (;;) {
      encoderTick();
      if (turn_flag) {
        norm_counter = constrain(norm_counter, 10, 95);
        tm1637.displayInt(norm_counter);
        turn_flag = false;
      }
      if (digitalRead(SW_ENC) == LOW) break;
    }
    NORM_STEP = 1;
    minBrightness = map(norm_counter, 0, 95, 1, minBrightness);
    EEPROM.updateByte(1, norm_counter);      // записываем новую яркость в память
    norm_counter = 0;
    tm1637.displayByte(_L, _O, _A, _d);

    for (i = 0; i < nFrames; i++) {
      sprintf(infile , "frame%03d.bmp", i);
      sprintf(outfile, "frame%03d.tmp", i);
      b = minBrightness; // Reset b to safe limit on each loop iteration
      bmpProcess(infile, outfile, &b);
    }

  } else {  // если кнопка НЕ НАЖАТА при старте - используем преобразованные картинки!
    tm1637.displayByte(_r, _e, _a, _d);
    do { // Scan for files to get nFrames
      sprintf(infile, "frame%03d.tmp", nFrames);
      if (found = tmp.open(infile, O_RDONLY)) {
        if (tmp.contiguousRange(&firstBlock, &lastBlock)) {
          nFrames++;
        }
        tmp.close();
      }
    } while (found);

  } // end startupTrigger test

  for (i = 0; i < nFrames; i++) { // Scan all files
    sprintf(infile, "frame%03d.tmp", i);
    tmp.open(infile, O_RDONLY);
    tmp.contiguousRange(&firstBlock, &lastBlock);
    nBlocks = tmp.fileSize() / 512;
    tmp.close();
    n = (uint16_t)(1000000L /                         // 1 uSec /
                   (((benchmark(firstBlock, nBlocks) * 21) / 20) + // time + 5% +
                    (N_LEDS * 30L) + OVERHEAD));                   // 30 uSec/pixel
    if (n > maxLPS) maxLPS = n;
  }
  if (maxLPS > 400) maxLPS = 400; // NeoPixel PWM rate is ~400 Hz
  if (DEBUG) Serial.print(F("Max lines/sec: "));
  if (DEBUG) Serial.println(maxLPS);

  // Set up Timer1 for 64:1 prescale (250 KHz clock source),
  // fast PWM mode, no PWM out.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  // Timer0 interrupt is disabled for smoother playback.
  // This means delay(), millis(), etc. won't work after this.
  TIMSK0 = 0;

  if (DEBUG) Serial.println(F("Setup done"));

  tm1637.displayByte(_r, _d, _y, _empty);
  delay(500);
  hold_counter = anim_speed;
  norm_counter = 0;
}

// Обработчик ошибки. Полностью вешает систему
static void error(const __FlashStringHelper *ptr) {
  if (DEBUG) Serial.println(ptr); // Show message
  tm1637.displayByte(_F, _e, _r, _r);
  for (;;);            // and hang
}

void encoderTick() {
  SWstate = digitalRead(SW_ENC);
  if (SWstate != SWlast) {
    SWlast = SWstate;
    turn_flag = true;                    // флаг что был поворот ручки энкодера
  }
  DT_now = digitalRead(CLK_ENC);          // читаем текущее положение CLK
  if (DT_now != DT_last) {            // если предыдущее и текущее положение CLK разные, значит был поворот
    if (digitalRead(DT_ENC) != DT_now) {  // если состояние DT отличается от CLK, значит крутим по часовой стрелке
      if (SWstate) norm_counter += NORM_STEP;
      else hold_counter += HOLD_STEP;
    } else {                          // если совпадают, значит против часовой
      if (SWstate) norm_counter -= NORM_STEP;
      else hold_counter -= HOLD_STEP;
    }
    turn_flag = true;                    // флаг что был поворот ручки энкодера
  }
  DT_last = DT_now;                   // обновить значение
}

void buffering() {
  block    = 0;     // Current block # within file
  stopFlag = false; // If set, stop playback loop

  // Get existing contiguous tempfile info
  sprintf(infile, "frame%03d.tmp", frame);
  if (!tmp.open(infile, O_RDONLY)) {
    error(F("Could not open NeoPixel tempfile for input"));
  }
  if (!tmp.contiguousRange(&firstBlock, &lastBlock)) {
    error(F("NeoPixel tempfile is not contiguous"));
  }

  nBlocks = tmp.fileSize() / 512;
  tmp.close(); // File handle is no longer accessed, just block reads
  sd.card()->readBlock(firstBlock, sdBuf);
}
// PLAYBACK LOOP -------------------------------------------------------------------------------
void loop() {
  buffering();
  tm1637.clearDisplay();
  tm1637.displayByte(0, _F);
  tm1637.displayByte(1, _r);
  tm1637.displayInt(frame);

  while (digitalRead(TRIGGER)) {      // ждём нажатия кнопки !!!
    encoderTick();
    if (turn_flag) {
      if (SWstate) { // если НЕ НАЖАТА кнопка энкодера
        norm_counter = constrain(norm_counter, 0, nFrames - 1);
        frame = norm_counter;
        tm1637.clearDisplay();
        tm1637.displayByte(0, _F);
        tm1637.displayByte(1, _r);
        tm1637.displayInt(frame);
        buffering();
      } else {       // если НАЖАТА кнопка энкодера
        hold_counter = constrain(hold_counter, 0, 95);
        anim_speed = hold_counter;
        tm1637.clearDisplay();
        tm1637.displayByte(0, _S);
        tm1637.displayByte(1, _P);
        tm1637.displayInt(anim_speed);
      }
      turn_flag = false;
    }
  }
  tm1637.displayByte(_P, _r, _o, _c);

  uint32_t linesPerSec = map(anim_speed, 0, 95, 10, maxLPS);  // настройка скорости
  EEPROM.updateByte(3, anim_speed);      // записываем новую скорость в память
  OCR1A = (F_CPU / 64) / linesPerSec;          // Timer1 interval

  // ------------ бесконечный цикл
  for (;;) {                                   // бесконечный цикл
    while (!(TIFR1 & _BV(TOV1)));              // Wait for Timer1 overflow
    TIFR1 |= _BV(TOV1);                        // Clear overflow bit

    show();                                    // отобразить линию
    if (stopFlag) break;                       // выходим из цикла, если stopFlag

    if (++block >= nBlocks) {                  // показали последний блок
      if (digitalRead(TRIGGER) == HIGH) {      // если отпустили кнопку
        memset(sdBuf, 0, N_LEDS * 3);          // прекращаем показывать картинку
        stopFlag = true;                       // флаг на выключение
        continue;
      }                                        // если кнопка всё ещё удерживается
      block = 0;                               // начать сначала!
    }
    sd.card()->readBlock(block + firstBlock, sdBuf); // загрузить следующий блок с карты памяти
  }
  // ------------ бесконечный цикл
}

// BMP->NEOPIXEL FILE CONVERSION ---------------------------------------------

#define BMP_BLUE  0 // BMP and NeoPixels have R/G/B color
#define BMP_GREEN 1 // components in different orders.
#define BMP_RED   2 // (BMP = BGR, Neo = GRB)
#define NEO_GREEN 0
#define NEO_RED   1
#define NEO_BLUE  2

boolean bmpProcess(
  char    *inName,
  char    *outName,
  uint8_t *brightness) {

  SdFile    inFile,              // Windows BMP file for input
            outFile;             // NeoPixel temp file for output
  boolean   ok        = false,   // 'true' on valid BMP & output file
            flip      = FLIP_H;  // 'true' if image stored top-to-bottom
  int       bmpWidth,            // BMP width in pixels
            bmpHeight,           // BMP height in pixels
            bmpStartCol,         // First BMP column to process (crop/center)
            columns,             // Number of columns to process (crop/center)
            row,                 // Current image row (Y)
            column;              // and column (X)
  uint8_t  *ditherRow,           // 16-element dither array for current row
           pixel[3],            // For reordering color data, BGR to GRB
           b = 0,               // 1 + *brightness
           d,                   // Dither value for row/column
           color,               // Color component index (R/G/B)
           raw,                 // 'Raw' R/G/B color value
           corr,                // Gamma-corrected R/G/B
           *ledPtr,              // Pointer into sdBuf (output)
           *ledStartPtr;         // First LED column to process (crop/center)
  uint16_t  b16;                 // 16-bit dup of b
  uint32_t  bmpImageoffset,      // Start of image data in BMP file
            lineMax   = 0L,      // Cumulative brightness of brightest line
            rowSize,             // BMP row size (bytes) w/32-bit alignment
            sum,                 // Sum of pixels in row
            startTime = millis();

  if (brightness)           b = 1 + *brightness; // Wraps around, fun with maths
  else if (NULL == outName) return false; // MUST pass brightness for power est.

  if (DEBUG) Serial.print(F("Reading file '"));
  if (DEBUG) Serial.print(inName);
  if (DEBUG) Serial.print(F("'..."));
  if (!inFile.open(inName, O_RDONLY)) {
    tm1637.displayByte(_F, _r, _E, _r);
    if (DEBUG) Serial.println(F("error"));
    return false;
  }

  if (inFile.read(sdBuf, 34)             &&   // Load header
      (*(uint16_t *)&sdBuf[ 0] == 0x4D42) &&    // BMP signature
      (*(uint16_t *)&sdBuf[26] == 1)      &&    // Planes: must be 1
      (*(uint16_t *)&sdBuf[28] == 24)     &&    // Bits per pixel: must be 24
      (*(uint32_t *)&sdBuf[30] == 0)) {         // Compression: must be 0 (none)
    // Supported BMP format -- proceed!
    bmpImageoffset = *(uint32_t *)&sdBuf[10]; // Start of image data
    bmpWidth       = *(uint32_t *)&sdBuf[18]; // Image dimensions
    bmpHeight      = *(uint32_t *)&sdBuf[22];
    // That's some nonportable, endian-dependent code right there.

    if (DEBUG) Serial.print(bmpWidth);
    Serial.write('x');
    if (DEBUG) Serial.print(bmpHeight);
    if (DEBUG) Serial.println(F(" pixels"));

    if (outName) { // Doing conversion?  Need outFile.
      // Delete existing outFile file (if any)
      (void)sd.remove(outName);
      if (DEBUG) Serial.print(F("Creating contiguous file..."));
      // NeoPixel working file is always 512 bytes (one SD block) per row
      if (outFile.createContiguous(sd.vwd(), outName, 512L * bmpHeight)) {
        uint32_t lastBlock;
        outFile.contiguousRange(&firstBlock, &lastBlock);
        outFile.close();
        nBlocks = bmpHeight; // See note in setup() re: block calcs
        ok      = true;      // outFile is good; proceed
        if (DEBUG) Serial.println(F("OK"));
      } else {
        if (DEBUG) Serial.println(F("error"));
        tm1637.displayByte(_F, _r, _E, _r);
      }
    } else ok = true; // outFile not needed; proceed

    if (ok) { // Valid BMP and contig file (if needed) are ready
      if (DEBUG) Serial.print(F("Processing..."));

      rowSize = ((bmpWidth * 3) + 3) & ~3; // 32-bit line boundary
      b16     = (int)b;

      if (bmpHeight < 0) {      // If bmpHeight is negative,bmpHeight
        bmpHeight = -bmpHeight; // image is in top-down order.
        flip      = true;       // Rare, but happens.
      }

      if (bmpWidth >= N_LEDS) { // BMP matches LED bar width, or crop image
        bmpStartCol = (bmpWidth - N_LEDS) / 2;
        ledStartPtr = sdBuf;
        columns     = N_LEDS;
      } else {                 // Center narrow image within LED bar
        bmpStartCol = 0;
        ledStartPtr = &sdBuf[((N_LEDS - bmpWidth) / 2) * 3];
        columns     = bmpWidth;
        memset(sdBuf, 0, N_LEDS * 3); // Clear left/right pixels
      }

      for (row = 0; row < bmpHeight; row++) { // For each row in image...
        Serial.write('.');
        inFile.seekSet(
          bmpImageoffset + (bmpStartCol * 3) + (rowSize * (flip ?
                                                (bmpHeight - 1 - row) : // Image is stored top-to-bottom
                                                row)));                 // Image stored bottom-to-top
        if (!inFile.read(ledStartPtr, columns * 3)) { // Load row
          if (DEBUG) Serial.println(F("Read error"));
          tm1637.displayByte(_F, _r, _E, _r);
        }

        sum       = 0L;
        ditherRow = (uint8_t *)&dither[row & 0x0F]; // Dither values for row
        ledPtr    = ledStartPtr;
        for (column = 0; column < columns; column++) { // For each column...
          if (b) { // Scale brightness, reorder R/G/B
            pixel[NEO_BLUE]  = (ledPtr[BMP_BLUE]  * b16) >> 8;
            pixel[NEO_GREEN] = (ledPtr[BMP_GREEN] * b16) >> 8;
            pixel[NEO_RED]   = (ledPtr[BMP_RED]   * b16) >> 8;
          } else { // Full brightness, reorder R/G/B
            pixel[NEO_BLUE]  = ledPtr[BMP_BLUE];
            pixel[NEO_GREEN] = ledPtr[BMP_GREEN];
            pixel[NEO_RED]   = ledPtr[BMP_RED];
          }

          d = pgm_read_byte(&ditherRow[column & 0x0F]); // Dither probability
          for (color = 0; color < 3; color++) {         // 3 color bytes...
            raw  = pixel[color];                        // 'Raw' G/R/B
            corr = pgm_read_byte(&gamma[raw]);          // Gamma-corrected
            if (pgm_read_byte(&bump[raw]) > d) corr++;  // Dither up?
            *ledPtr++ = corr;                           // Store back in sdBuf
            sum      += corr;                           // Total brightness
          } // Next color byte
        } // Next column

        if (outName) {
          if (!sd.card()->writeBlock(firstBlock + row, (uint8_t *)sdBuf)) {
            if (DEBUG) Serial.println(F("Write error"));
            tm1637.displayByte(_F, _r, _E, _r);
          }
        }
        if (sum > lineMax) lineMax = sum;

      } // Next row
      if (DEBUG) Serial.println(F("OK"));

      if (brightness) {
        lineMax = (lineMax * 20) / 255; // Est current @ ~20 mA/LED
        if (lineMax > CURRENT_MAX) {
          // Estimate suitable brightness based on CURRENT_MAX
          *brightness = (*brightness * (uint32_t)CURRENT_MAX) / lineMax;
        } // Else no recommended change
      }

      if (DEBUG) Serial.print(F("Processed in "));
      if (DEBUG) Serial.print(millis() - startTime);
      if (DEBUG) Serial.println(F(" ms"));

    } // end 'ok' check
  } else { // end BMP header check
    if (DEBUG) Serial.println(F("BMP format not recognized."));
    tm1637.displayByte(_F, _r, _E, _r);
  }

  inFile.close();
  return ok; // 'false' on various file open/create errors
}

// MISC UTILITY FUNCTIONS ----------------------------------------------------

// Estimate maximum block-read time for card (microseconds)
static uint32_t benchmark(uint32_t block, uint32_t n) {
  uint32_t t, maxTime = 0L;

  do {
    t = micros();
    sd.card()->readBlock(block++, sdBuf);
    if ((t = (micros() - t)) > maxTime) maxTime = t;
  } while (--n);

  return maxTime;
}

// NEOPIXEL FUNCTIONS --------------------------------------------------------

static void show(void) {
  volatile uint16_t
  i   = N_LEDS * 3; // Loop counter
  volatile uint8_t
  *ptr = sdBuf,      // Pointer to next byte
   b   = *ptr++,     // Current byte value
   hi,               // PORT w/output bit set high
   lo,               // PORT w/output bit set low
   next,
   bit = 8;

  noInterrupts();
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;

  asm volatile(
    "head20_%=:"                "\n\t"
    "st   %a[port],  %[hi]"    "\n\t"
    "sbrc %[byte],  7"         "\n\t"
    "mov  %[next], %[hi]"     "\n\t"
    "dec  %[bit]"              "\n\t"
    "st   %a[port],  %[next]"  "\n\t"
    "mov  %[next] ,  %[lo]"    "\n\t"
    "breq nextbyte20_%="       "\n\t"
    "rol  %[byte]"             "\n\t"
    "rjmp .+0"                 "\n\t"
    "nop"                      "\n\t"
    "st   %a[port],  %[lo]"    "\n\t"
    "nop"                      "\n\t"
    "rjmp .+0"                 "\n\t"
    "rjmp head20_%="           "\n\t"
    "nextbyte20_%=:"            "\n\t"
    "ldi  %[bit]  ,  8"        "\n\t"
    "ld   %[byte] ,  %a[ptr]+" "\n\t"
    "st   %a[port], %[lo]"     "\n\t"
    "nop"                      "\n\t"
    "sbiw %[count], 1"         "\n\t"
    "brne head20_%="          "\n"
    : [port]  "+e" (port),
    [byte]  "+r" (b),
    [bit]   "+r" (bit),
    [next]  "+r" (next),
    [count] "+w" (i)
    : [ptr]    "e" (ptr),
    [hi]     "r" (hi),
    [lo]     "r" (lo));

  interrupts();
  // There's no explicit 50 uS delay here as with most NeoPixel code;
  // SD card block read provides ample time for latch!
}
