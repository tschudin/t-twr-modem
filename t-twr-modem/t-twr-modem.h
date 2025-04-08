// t-twr-modem.h

// 1000bps modem (4-FSK/500 Baud) with LDPC FEC, 120 Bytes packets
// for the Lilygo T-TWR v2.1 walkie talkie ESP32-S3 device

// 2025 (c) christian.tschudin@unibas.ch, HB9HUH/K6CFT

#include <Arduino.h>
#include <Wire.h>
#include <driver/sdm.h>

#include <LilyGo_TWR.h>
#include <LittleFS.h>
#include <TinyGPS++.h>          //https://github.com/mikalhart/TinyGPSPlus
#include <AceButton.h>
#include <U8g2lib.h>

#include "util/adc.h"
#include "util/fht.h"
#include "util/crc12_b91.h"

#include "filters.h"

#define ARRAY_LEN(ptr) (sizeof(ptr)/sizeof(*ptr))

void send_message(char *cmd);
void report(char *buf, bool quote=false);
void report_hex(uint8_t *data, int cnt);
char* gps_to_tim();
char* gps_to_loc();
void set_status_msg(char *msg);

/*

FS=6400 --> bandwidth is 3200Hz

a) FSK4:

    N=32 samples per symbol -> 1+16 freq bins, each 200 Hz wide
    we bundle three frequency bins for one symbol:

        '0' '1' '2' '3'          symbols
         |   |   |   |
    D01 234 567 89a bcd ef       frequency bins


b) FSK2

    8 samples per symbol -> 1+4 freq bins, each 800 Hz wide

       '0' '1'                   symbols
        |   |
    D 0 1 2 3                    frequency bins


--- to be explored and optimized:

c) FSK8:

    N=64 samples per symbol -> 1+32 frequency bins, each 100Hz wide

           _0 _1 _2 _3 _4 _5 _6 _7        symbols
            |  |  |  |  |  |  |  |
    xxxxxxx000111222333444555666777xx     frequency bins

d) "skewed bands" (0 has less freq width, N-1 is broadest batch)

 */

// ---------------------------------------------------------------------------
// adjustable configuations:

// select the modulation
#define FSK4   // or define FSK8, FSK2

// good HAM radio practice: declare on the airwaves who you are
#define CALLSIGN      "K6CFT"
#define TEST_MSG      (CALLSIGN " / HB9HUH - 70cm/432MHz TWR-T experiment")

// hardwired snd/rcv frequency in the 70cm HAM band, IARU-R1 Switzerland
// note: the PMR frequencies are not available to HAMs in the US

// #define PMR_FREQ      (432975000)  // UHF amateur band 430-440MHz, "new APRS"
// #define PMR_FREQ      (432950000)  // squatted by REGA in Basel
// #define PMR_FREQ      (432587500)  // squatted by REGA in Ticino
// #define PMR_FREQ      (432575000)  // squatted by REGA in Basel
#define PMR_FREQ      (432562500) // seems to be a quite band

#define LDPC_PARITY_LENGTH     (256)   // use 256, 512 or 1024 redundancy bits
                                       // (this selects the LDPC code)

// ---------------------------------------------------------------------------
// settings that can be toggled via the serial command line interface:

int debug_output = 0; // 0 = off, ... 4 also shows intermediate bit values
int data_dump = 0;    // 0 = off, 1 = on
int clock_dump = 0;   // 0 = off, 1 = on
int waterfall = 0;    // 0 = off, 1 = fast/density, 2 = slow/detailed

// ---------------------------------------------------------------------------
// low-level implementation-specific definitions:

#include "lib/codec2-ldpc/codec2-ldpc.c"

#define MESSAGE_BYTE_LENGTH    (120)
#define MESSAGE_BIT_LENGTH     (8*MESSAGE_BYTE_LENGTH)
#define CRC_BIT_LENGTH         (12)
#define PADDING_LENGTH         ((BITS_PER_SYM - ((MESSAGE_BIT_LENGTH + CRC_BIT_LENGTH + LDPC_PARITY_LENGTH)%BITS_PER_SYM)) % BITS_PER_SYM)

#define PAYLOAD_BIT_LEN        (MESSAGE_BIT_LENGTH + CRC_BIT_LENGTH + PADDING_LENGTH)
#define PACKET_BIT_LEN         (PAYLOAD_BIT_LEN + LDPC_PARITY_LENGTH)
#define PACKET_SYMBOL_LEN      (PACKET_BIT_LEN / BITS_PER_SYM)

#define CSM 0x034776C7272895B0 // CCSDS Codeblock Sync Marker, 64 bits
#define SYNC_MARK_LEN (8*sizeof(CSM))
#define SYNC_THRHOLD  (48)     // 48 from 64: up to 25% bit errors ok for CSM

#define SIGMADELTAMOD  // this enables true sine waves instead of rectangles

#if defined(FSK8) // doesn;t work right now due to CSM not being divisible by 3
  #define FS            (6400)
  #define DATA_BAUD     (200)
  #define BITS_PER_SYM  (3)
  #define SAMPLE_WIN    (64)  // number of audio samples to analyze (32x100Hz)
  #define F_0           (850) // F_BIN_0 * 100 + 0.5 * F_DELTA
  #define F_DELTA       (300) // F_BIN_CNT * 100
  #define F_BIN_0       7
  #define F_BIN_CNT     3
  #define FREQ_BOOST    1.01 // 1.035
  #define PILOT_NDX     2
  #define WTRFALL_HDR   "#          01234567  [          0   1   2   3   4   5   6   7   ]"
#endif

#if defined(FSK4)
  #define FS            (6400)
  #define DATA_BAUD     (500)
  #define BITS_PER_SYM  (2)
  #define SAMPLE_WIN    (32)  // number of audio samples to analyze (16x200Hz)
  #define F_0           (900) // F_BIN_0 * 200 + 0.5 * F_DELTA
  #define F_DELTA       (600) // F_BIN_CNT * 200
  #define F_BIN_0       3
  #define F_BIN_CNT     3
  #define FREQ_BOOST    1.0
  #define PILOT_NDX     1
  #define WTRFALL_HDR   "#                  0123      [ 0   1   2   3 ]"
#endif

#if defined(FSK2)
  #define FS            (6400)
  #define DATA_BAUD     (600)
  #define BITS_PER_SYM  (1)
  #define SAMPLE_WIN    (16)   // number of audio samples to analyze (8x400Hz)
  #define F_0           (1400)
  #define F_DELTA       (1400)
  #define F_BIN_0       3
  #define F_BIN_CNT     3
  #define FREQ_BOOST    1.0
  #define PILOT_NDX     (1)
  #define WTRFALL_HDR   "#       01  [ 0  1 ]"
#endif

// ---------------------------------------------------------------------------

#define ECHOREQUEST "cmd=ECHO.REQUEST"
#define ECHOREPLY   "cmd=ECHO.REPLY"

#define MyFS LittleFS
#define LOG_FILE_NAME "/log.txt"
File the_log;

sdm_channel_handle_t sdm_chan = NULL;          // sigmadeltamod output
adc_continuous_handle_t the_adc_handle = NULL; // ADC input

static int16_t  rcv_audio[4*6400];  // ring buffer of filtered audio
static uint32_t rcv_audio_start;    // first valid entry (=oldest one)
static uint32_t rcv_audio_end;      // last sample we have analyzed, so far

static uint8_t  rcv_symbols[PACKET_SYMBOL_LEN + 2 * 8 * sizeof(CSM)/BITS_PER_SYM];
static uint8_t  rcv_confid[sizeof(rcv_symbols)];
static uint32_t rcv_sym_start;      // first valid entry (=oldest one)

float sym_clock;                    // symbol-level synchronization

enum {
  MODEM_NOSYNC,  // searching for CSM
  MODEM_SYNCED,  // inside codeblock, collecting symbols
  MODEM_XMIT
};
static uint8_t modem_state = MODEM_NOSYNC;

// ---------------------------------------------------------------------------
// hardware definitions

#define GPIO_OF_RADIO_AUDIO 1
#define GPIO_OF_RADIO_SQL   2

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void u8g2_print(int x, int y, char *cp, bool refresh=true)
{
    u8g2.drawStr(x, y, cp);
    if (refresh)
      u8g2.sendBuffer();
}

void u8g2_clearLine(int y, bool refresh=true)
{
  u8g2.setDrawColor(0);
  u8g2.drawBox(0,y-10,128,12);
  u8g2.setDrawColor(1);
  if (refresh)
    u8g2.sendBuffer();
}

// ---

using namespace ace_button;
AceButton buttons[3];

const uint8_t buttonPins [] = {
    ENCODER_OK_PIN,
    BUTTON_PTT_PIN,
    BUTTON_DOWN_PIN
};

enum Button {
    ShortPress,
    LongPress,
    Unknown
};

void handleEvent(AceButton *button, uint8_t eventType, uint8_t buttonState)
{
    uint8_t id = button->getId();

    // Serial.printf("# button id=%d event=%d state=%d\r\n",
    //               id, eventType, buttonState);

    switch (id) {
    case 0: // rotary center button
            break;
    case 1: // PTT button
        if (eventType == AceButton::kEventPressed)
            send_message(ECHOREQUEST);
        break;
    case 2: // BOOT button
        break;
    default:
        break;
    }
}

// ---

TinyGPSPlus gps;

// ---------------------------------------------------------------------------

void setup()
{
    bool rlst = false;

    Serial.begin(115200);
    // Serial.begin(460800);

    Serial.setTimeout(5000);
    Serial.printf("\r\n\r\n# T-TWR %dbps Walkie Talkie Modem (%d-FSK, %d Baud)\r\n",
                  BITS_PER_SYM * DATA_BAUD, (1<<BITS_PER_SYM), DATA_BAUD);

    twr.begin();
    if (twr.getVersion() == TWRClass::TWR_REV2V1) {
        Serial.println("# detected TWR Rev2.1");
        radio.setPins(SA868_PTT_PIN, SA868_PD_PIN);
        rlst = radio.begin(RadioSerial, twr.getBandDefinition());
    } else {
        Serial.println("# detected TWR Rev2.0");
        radio.setPins(SA868_PTT_PIN, SA868_PD_PIN, SA868_RF_PIN);
        // must pick band by hand:
        //  rlst = radio.begin(RadioSerial, SA8X8_UHF);
        rlst = radio.begin(RadioSerial, SA8X8_VHF);
    }

    if (!rlst) {
        while (1) {
            Serial.println("#   SA868 is not online !");
            delay(1000);
        }
    }
    twr.routingMicrophoneChannel(TWRClass::TWR_MIC_TO_ESP);
    twr.enablePowerOff(true);
    uint8_t addr = twr.getOLEDAddress();
    while (addr == 0xFF) {
        Serial.println("#   No OLED?");
        delay(1000);
    }
    u8g2.setI2CAddress(addr << 1);
    u8g2.begin();
    u8g2.setFontMode(0);

    char s[20];
    sprintf(s, "%dbps modem", BITS_PER_SYM * DATA_BAUD);
    u8g2.setFont(u8g2_font_helvR10_tf); //u8g2_font_cu12_hr);
    u8g2_print(0, 11, s);

    u8g2.setFont(u8g2_font_helvR08_tf); //u8g2_font_cu12_hr);
    int m = PMR_FREQ/1000000, k = (PMR_FREQ - m*1000000) / 1000;
    sprintf(s, "%03d.%03d MHz", m, k);
    u8g2_print(0, 64, s);
    int w = u8g2.getStrWidth(CALLSIGN);
    u8g2_print(128-w, 64, CALLSIGN);

    radio.setRxFreq(PMR_FREQ);
    radio.setTxFreq(PMR_FREQ);
    radio.setRxCXCSS(0);
    radio.setTxCXCSS(0);
    // radio.lowPower();
    radio.highPower();
    // radio.enableLowPass(true);
    // radio.enableHighPass(true);
    // radio.enableEmpHassis(true); // we implement our own freq bosting

    // twr.routingSpeakerChannel(TWRClass::TWR_ESP_TO_SPK);
    twr.routingSpeakerChannel(TWRClass::TWR_RADIO_TO_SPK);
    radio.setVolume(0);
    radio.setSquelchLevel(1);

    twr.enableGPS();

    // Initialize button
    for (uint8_t i = 0; i < ARRAY_LEN(buttonPins); i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
        buttons[i].init(buttonPins[i], HIGH, i);
    }
    ButtonConfig *buttonConfig = ButtonConfig::getSystemButtonConfig();
    buttonConfig->setEventHandler(handleEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);
    
    if (!MyFS.begin(false)) {
        Serial.println("#  littleFS.begin() failed, reformatting");
        MyFS.format();
        MyFS.begin(false);
        Serial.println("  done formatting");
    }
    the_log = MyFS.open(LOG_FILE_NAME, FILE_APPEND);

    ldpc_setup();

    the_adc_handle = continuous_adc_init(FS, GPIO_OF_RADIO_AUDIO,
                                         GPIO_OF_RADIO_SQL);
    adc_continuous_start(the_adc_handle);

    modem_state = MODEM_NOSYNC;

    Serial.println("# setup done");
    // Serial.printf("# log level=%d, data dump=%d, clock_dump=%d\r\n",
    //               debug_output, data_dump, clock_dump);
    Serial.println();
}

// ---------------------------------------------------------------------------
// visual output routines

void hexdump(uint8_t *buf, int cnt)
{
      Serial.printf("# data:");
      for (int i = 0; i < cnt; i++) {
        if (i != 0 && (i%16) == 0 )
          Serial.printf("\r\n#      ");
        Serial.printf(" %02x", buf[i]);
      }
      Serial.printf(" (%d bytes)\r\n", cnt);
}

void show_waterfall(float power, int pos, float *p, int p_len,
                    uint8_t *conf, float *sym_clock)
{
    Serial.printf("t=%04x ", rcv_audio_end);

    Serial.printf("e=%-5d ", int(power));

    for (int i = 0; i < pos; i++)
        Serial.printf(".");
    Serial.printf("@");
    for (int i = 0; i < (1<<BITS_PER_SYM) - 1 - pos; i++)
        Serial.printf(".");
    Serial.printf("  ");

    // find per-band max for scaling energy levels to 0..9
    float ma = p[1];
    for (int i = 2; i < p_len; i++)
        if (p[i] > ma)
            ma = p[i];

    for (int i = 0; i < p_len; i++) {
    #if defined(FSK2)
        if (i % F_BIN_CNT == 2)
    #endif
    #if defined(FSK8) || defined(FSK4b) || defined(FSK2)
        if (i % F_BIN_CNT == 1)
    #endif
    #if defined(FSK4) || defined(FSK4_8000) || defined(FSK4c) || defined(FSK4sk)
        if (i % F_BIN_CNT == 0)
    #endif          
            Serial.printf(" ");

        // scale and map to [0..9]
        int v = int(9 * p[i] / ma);
        Serial.printf(v > 9 ? "x" : "%d", v);
    }

    Serial.printf(" ");
    if (conf)
        Serial.printf(" c=%-3d", *conf);
    if (sym_clock)
        Serial.printf(" clk=%-6.3g", *sym_clock);
    Serial.println();
}

// ---------------------------------------------------------------------------
// modulation

// CCSDS 2023 randomizer (x^17 + x^14 + 1)
static uint32_t lfsr_state;
void randomizer_reset() { lfsr_state = 0x18e38; }
uint8_t randomizer_next_bit()
{
    uint8_t out = lfsr_state & 0x01;
    lfsr_state = (lfsr_state | ( (out ^ (lfsr_state>>14)) << 17 )) >> 1;
    lfsr_state &= 0x1ffff;
    return out;
}

#if BITS_PER_SYM == 3
int int2gray(int n) // 0 <= n < 8
{
    static const char t[] = {0, 4, 5, 7, 6, 2, 3, 1};
    return t[n & 0x07];
}

int gray2int(int n)
{
    static const char t[] = {0, 7, 5, 6, 1, 2, 4, 3};
    return t[n & 0x07];
}
#endif

#if BITS_PER_SYM == 2
int int2gray(int n) // 0 <= n < 4
{
    static const char t[] = {0, 2, 3, 1};
    return t[n & 0x03];
}

int gray2int(int n)
{
  static const char t[] = {0, 3, 1, 2};
    return t[n & 0x03];
}
#endif

#if BITS_PER_SYM == 1
int int2gray(int n) { return n & 0x01; }
int gray2int(int n) { return n & 0x01; }
#endif


void emitOneSymbol(int ndx, int baud, uint64_t *end_time)
{
    uint32_t fr = (ndx == -1) ? 0 : F_0 + ndx * F_DELTA;

#if defined(FSK8)
    // const float ampl[] = {0.3, 0.3, 0.4, 0.5, 0.7, 0.9, 1.2, 1.4};
    // const float ampl[] = {0.7, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9, 1.1};
    static const float ampl[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.1, 1.2, 1.2};
#endif
#if defined(FSK4)
    static const float ampl[] = {0.3, 0.3, 0.7, 1.4}; // sender-side emphasis
    // const float ampl[] = {0.2, 0.2, 0.5, 1.4}; // sender-side emphasis
#endif
#if defined(FSK2)
    static const float ampl[] = {0.7, 1.2};
#endif
    float a = (ndx == -1) ? 1.0 : ampl[ndx];
    
    *end_time += 1000000 / baud; // potential accumulation of rounding error

#if defined (SIGMADELTAMOD)
    // Sigma Delta Modulation using 20Mhz oversampling for our 3khz signal
    static float phase = 0; // keep track of phase, continue there for next sym
    uint32_t t = 40000. * phase / fr;
    uint64_t end_of_sample = esp_timer_get_time();
    if (fr == 0)
        sdm_channel_set_pulse_density(sdm_chan, 0); // silence
    while (esp_timer_get_time() < *end_time) {
        if (fr && end_of_sample < esp_timer_get_time()) {
            phase = (fr * t++) / 40000.;
            phase -= truncf(phase);
            // FIXME: density is signed 8bit, we could lower the 40kHz to
            // a frequency where the amplitude differs between steps
            sdm_channel_set_pulse_density(sdm_chan,
                                          a * 70. * cos(2. * PI * phase));
            // +-70. is empiric: 95 is too loud, <50 too weak
            end_of_sample += 25;  // every 25usec (40khz)
        }
    }
#else  // use the LED pulse output (rectangle waves)
    static int old_freq = 0;
    if (old_freq != fr)
        ledcWriteTone(ESP2SA868_MIC, fr);
    old_freq = fr;

    int64_t now;
    while (1) {
      now = esp_timer_get_time();
      if (now >= t0_usec) break;
      // else idle loop
    }
#endif
}

void modem_emit_bits(uint8_t *ibits, int cnt) // this is the modulator
{
    if (cnt <= 0) return;

    if (BITS_PER_SYM > 1 && (cnt % BITS_PER_SYM) != 0) {
      Serial.printf("# ** number of bits not multiple of %d\r\n", BITS_PER_SYM);
      return;
    }
    uint8_t *syms = (uint8_t*) alloca(cnt/BITS_PER_SYM);
    int symcnt = 0;

    if (debug_output > 3) {
        Serial.printf("# outgoing %d+%d+%d post-rnd pre-gray bits (incl CSM):\r\n#  ",
                      SYNC_MARK_LEN, PACKET_BIT_LEN, SYNC_MARK_LEN);
        for (int i = 0; i < cnt;) {
            if (i != 0 && (i % 48) == 0)
                Serial.printf("\r\n#  ");
            if (i == 8*sizeof(CSM) ||
                i == (cnt - 8*sizeof(CSM)))
                Serial.printf("*");
            else
                Serial.printf(" ");
            for (int j = 0; j < BITS_PER_SYM; j++)
                Serial.printf("%d", ibits[i++]);
        }
        Serial.println();
    }

    // map to symbols, includes Gray encoding
#if BITS_PER_SYM != 1
    for (int i = 0; i < cnt;) {
        uint32_t a = 0;
        for (int j = 0; j < BITS_PER_SYM; j++)
            a = (a << 1) | ibits[i++];
        syms[symcnt++] = int2gray(a);
    }
#else
    for (int i = 0; i < cnt;)
        syms[symcnt++] = ibits[i++];
#endif
    if (debug_output > 0) {
        Serial.printf("# outgoing %d symbols (incl CSM):\r\n",
                      (2*SYNC_MARK_LEN+PACKET_BIT_LEN)/BITS_PER_SYM);
        int csm_len = 8*sizeof(CSM)/BITS_PER_SYM;
        Serial.printf("# /");
        for (int i = 0; i < csm_len; i++) {
            if (i != 0 && (i % 32) == 0)
                Serial.printf("\r\n# /");
            Serial.printf(" %d", syms[i]);
        }
        Serial.printf("\r\n# |");
        for (int i = csm_len; i < symcnt-csm_len; i++) {
            if (i != csm_len && ((i-csm_len) % 32) == 0)
                Serial.printf("\r\n# |");
            Serial.printf(" %d", syms[i]);
        }
        Serial.printf("\r\n# \\");
        for (int i = symcnt - csm_len; i < symcnt; i++) {
            if (i != symcnt - csm_len && ((i-symcnt+csm_len) % 32) == 0)
                Serial.printf("\r\n# \\");
            Serial.printf(" %d", syms[i]);
        }
        Serial.println();
    }

#if defined(SIGMADELTAMOD)
    sdm_config_t config = {
        .gpio_num = ESP2SA868_MIC,
        .clk_src = SDM_CLK_SRC_DEFAULT,
        .sample_rate_hz = 20*1000000,
    };
    sdm_new_channel(&config, &sdm_chan);
    sdm_channel_enable(sdm_chan);
#else
    if (!ledcAttach(ESP2SA868_MIC, 10000, 8)) {
        Serial.printf("# ledcAttach(%d) failed\r\n", ESP2SA868_MIC);
        return;
    }
#endif

    // here comes the packet train
    uint64_t start, time_ref;
    start = time_ref = esp_timer_get_time();

    // bring remote (as well as local?) AGC into a steady state,
    // use this 250msec warmup time to train the symbol clock
    while (esp_timer_get_time() - start < 250000) {
        emitOneSymbol(esp_random() % (1<<BITS_PER_SYM), DATA_BAUD, &time_ref);
        // emitOneSymbol(-1, DATA_BAUD, &time_ref);
    }
    // emitOneSymbol(esp_random() % ((1<<BITS_PER_SYM)-1), DATA_BAUD, &time_ref);

    // send the data
    for (int i = 0; i < symcnt; i++)
        emitOneSymbol(syms[i], DATA_BAUD, &time_ref);

    // we could start more packets here..

    emitOneSymbol(PILOT_NDX, DATA_BAUD, &time_ref); // one dummy symbol
    emitOneSymbol(-1, DATA_BAUD, &time_ref); // two times silence
    emitOneSymbol(-1, DATA_BAUD, &time_ref);

#if !defined(SIGMADELTAMOD)
    ledcDetach(ESP2SA868_MIC); // end of rectangle signal
#else
    sdm_channel_disable(sdm_chan);
    sdm_del_channel(sdm_chan);
    sdm_chan = NULL;
#endif

    Serial.println();
}

// ---------------------------------------------------------------------------
// demodulation

// audio band-pass filter, built from a LPF and HPF
Filter audio_hpf(  200, 1.0/FS, IIR::ORDER::OD4, IIR::TYPE::HIGHPASS);
Filter audio_lpf( 3000, 1.0/FS, IIR::ORDER::OD4, IIR::TYPE::LOWPASS);
Filter power_lpf(25, 1.0/FS, IIR::ORDER::OD4, IIR::TYPE::LOWPASS);

void radio_receive()
{
    uint8_t frame[ADC_FRAME_LEN];
    uint32_t sample_cnt = 0;
    int rc = adc_continuous_read(the_adc_handle,
                                 frame, sizeof(frame),
                                 &sample_cnt, 0);
    if (rc != ESP_OK) {
        if (rc == ESP_ERR_INVALID_STATE)
            Serial.println("#---- adc_read(): lost some samples");
        return;
    }

    for (int i; i < sample_cnt; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&frame[i];
        uint32_t chan_num = p->type2.channel;
        if (chan_num != 0) // invalid data (channel >10: invalid)
            continue;
        int16_t a = audio_hpf.filterIn(audio_lpf.filterIn(p->type2.data));
        rcv_audio[rcv_audio_start++] = a;
        rcv_audio_start %= ARRAY_LEN(rcv_audio);
        if (data_dump) {
            Serial.printf(",");
            Serial.printf(rcv_audio_start % 24 == 0 ? "\r\n" : " ");
            Serial.printf("%d", a);
        }
    }
}

float sym_clock_filter(float c)
{
    // https://user.eng.umd.edu/~tretter/commlab/c6713slides/FSKSlides.pdf
    static const double r = 0.9995; // 0.998
    static double v[3]; // ndx0 is nT-2T, ndx1 is nT-T, ndx2 is nT
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (1-r)*c + 2*r*cos(2*PI*DATA_BAUD/FS)*v[1] - r*r*v[0];
    return v[2] - v[0];
}

void modem_decode(uint16_t ptr, uint16_t cnt)
{
    u8g2_clearLine(25);
    u8g2_print(0, 25, "   decoding ...");
    if (debug_output > 0) {
        Serial.printf("# incoming %d codeblock symbols (no CSM):\r\n# |", cnt);
        for (int i = 0; i < cnt; i++) {
            if (i % 32 == 0 && i != 0)
                Serial.printf("\r\n# |");
            Serial.printf(" %d", rcv_symbols[(ptr+i) % sizeof(rcv_symbols)]);
        }
        Serial.println();
    }

    if (debug_output > 1) {
        Serial.printf("# confidence:\r\n#  ");
        float c_sum = 0.;
        for (int i = 0; i < cnt; i++) {
            float c = 10 * rcv_confid[(ptr+i) % sizeof(rcv_symbols)] / 128; // map 0..127 to 0..9
            c_sum += c;
            if (i != 0 && i % 32 == 0)
                Serial.printf("\r\n#  ");
            Serial.printf(" %d", (int)c);
        }
        Serial.printf("\r\n# avg conf (in 0..9): %g\r\n", c_sum/cnt);
    }

    if (debug_output > 3) {
        Serial.printf("# incoming %d post-ungray pre-unrnd bits (no CSM):\r\n#  ",
                      cnt * BITS_PER_SYM);
        for (int i = 0; i < cnt; i++) {
            uint16_t ndx = (ptr+i) % sizeof(rcv_symbols);
            uint8_t val = gray2int(rcv_symbols[ndx]);
            // if (i % BITS_PER_SYM == 0)
            if (i != 0 && i % (48/BITS_PER_SYM) == 0)
                Serial.printf("\r\n#  ");
            Serial.printf(" ");
            for (int j = 0; j < BITS_PER_SYM; j++) {
                uint8_t b = 0x01 & (val >> (BITS_PER_SYM-1-j));
                Serial.printf("%d", b);
            }
        }
        Serial.println();
    }

    if (debug_output > 2) {
        randomizer_reset();
        Serial.printf("# incoming %d post-unrnd bits (no CSM):\r\n#  ",
                      cnt * BITS_PER_SYM);
        for (int i = 0; i < cnt; i++) {
            uint16_t ndx = (ptr+i) % sizeof(rcv_symbols);
            uint8_t val = gray2int(rcv_symbols[ndx]);
            // if (i % BITS_PER_SYM == 0)
            if (i != 0 && (i % (48/BITS_PER_SYM)) == 0)
                Serial.printf("\r\n#  ");
            Serial.printf(" ");
            for (int j = 0; j < BITS_PER_SYM; j++) {
                uint8_t b = 0x01 & (val >> (BITS_PER_SYM-1-j));
                b ^= randomizer_next_bit();
                Serial.printf("%d", b);
            }
        }
        Serial.println();
    }

    randomizer_reset();
    static float uncorrected_llr[LDPC_WORD_LENGTH];
    int bit_cnt = 0;
    for (int i = 0; i < cnt; i++) {
        if ((bit_cnt + BITS_PER_SYM) >= ARRAY_LEN(uncorrected_llr))
            break;
        uint16_t ndx = (ptr+i) % sizeof(rcv_symbols);
        uint8_t val = gray2int(rcv_symbols[ndx]);
        float c = rcv_confid[ndx]/127.;
        for (int j = 0; j < BITS_PER_SYM; j++) {
            uint8_t b = 0x01 & (val >> (BITS_PER_SYM-1-j));
            b ^= randomizer_next_bit();
            uncorrected_llr[bit_cnt++] = b ? -c : c;  // soft decision value
        }
    }
    sd_to_llr(uncorrected_llr, uncorrected_llr, bit_cnt); // inplace
    for (int i = 0; i < PADDING_LENGTH; i++)
      uncorrected_llr[MESSAGE_BIT_LENGTH + CRC_BIT_LENGTH + i] = -10;

    int parChkCnt;
    static uint8_t corrected[LDPC_WORD_LENGTH];
    uint64_t start = esp_timer_get_time();
    int iter = ldpc_decode_1024(corrected, uncorrected_llr,
                                MESSAGE_BIT_LENGTH + CRC_BIT_LENGTH,
                                &parChkCnt);
    if (debug_output > 1) {
        Serial.printf("# LDPC decoding: %d iterations, parityCheckCount=%d\r\n",
                    iter, parChkCnt);
        Serial.printf("#   %ld usec per iteration\r\n",
                      (esp_timer_get_time() - start)/iter);
    }
    char buf[100];
    sprintf(buf, "LDPC iterations=%d, parChkCnt=%d", iter, parChkCnt);

    uint8_t msg[MESSAGE_BYTE_LENGTH];
    for (int i = 0; i < MESSAGE_BIT_LENGTH; i += 8) { // without CRC
        uint8_t a = 0;
        for (int j = 0; j < 8; j++)
            a = (a << 1) | corrected[i+j]; // (corrected[i + j]<0 ? 1 : 0);
        msg[i/8] = a;
    }
    uint16_t crc = 0;
    for (int i = 0; i < CRC_BIT_LENGTH; i++)
      crc = (crc << 1) | corrected[MESSAGE_BIT_LENGTH + i]; //  < 0 ? 1 : 0);
    hexdump(msg, MESSAGE_BYTE_LENGTH);
    report_hex(msg, MESSAGE_BYTE_LENGTH);
    uint8_t ok = 0;
    if (crc == crc12_b91(msg, MESSAGE_BYTE_LENGTH)) {
        set_status_msg("OK");
        Serial.println("#       crc ok");
        strcpy(buf+strlen(buf), ", crc=ok");
        ok = 1;
    } else {
        set_status_msg("??");
        Serial.println("#       crc mismatch");
        strcpy(buf+strlen(buf), ", crc=fail");
        char *cp = (char*)msg;
        while (*cp && (cp - (char*)msg) < 120) {
            if (!isprint(*cp))
                *cp = '?';
            cp++;
        }
    }
    report(buf);

    if (strlen((char*)msg) < 120) {
        Serial.printf("# msg: '%s'\r\n\r\n", (char*)msg);
        report((char*)msg, true);
        if (ok && strstr((char*)msg, ECHOREQUEST))
            send_message(ECHOREPLY);
    } else
        Serial.println();
}

void modem_process_symbol(int sym, uint16_t confid, float power)
{
    static float old_sym_clock;    // for detecting zero-crossing
    static int next_sample_ticks;  // after-next sampling time
    static int ticks_until_sample; // counted from last zero-crossing
    static uint16_t sym_cnt;

    // Serial.printf("%g,\r\n", sym_clock);
    if (old_sym_clock > 0 && sym_clock < 0) // zero-crossing
        next_sample_ticks = 7 * FS / DATA_BAUD / 4; // 11+16;
    else
        next_sample_ticks--;
    old_sym_clock = sym_clock;

    if (ticks_until_sample-- > 0)
        return;
    ticks_until_sample = next_sample_ticks;

    // if (power > 100) Serial.printf("%d,\r\n", sym);
    rcv_symbols[rcv_sym_start] = sym;
    rcv_confid[rcv_sym_start] = confid;
    rcv_sym_start = (rcv_sym_start + 1) % sizeof(rcv_symbols);
    sym_cnt++;

    // check for CSM: count matching bits
    int match = 0;
    uint16_t sptr = rcv_sym_start + sizeof(rcv_symbols)
                                  - SYNC_MARK_LEN/BITS_PER_SYM;
    sptr %= sizeof(rcv_symbols);
    uint64_t v = CSM;
    for (int i = 0; i < SYNC_MARK_LEN/BITS_PER_SYM; i++) {
        uint8_t s = gray2int(rcv_symbols[sptr]);
        for (int j = BITS_PER_SYM-1; j >= 0; j--) {
            if (((s >> j) & 0x01) == (v & 0x01))
                match++;
            v >>= 1;
        }
        sptr = (sptr + 1) % sizeof(rcv_symbols);
    }

    static uint16_t no_signal;
    if (match >= SYNC_THRHOLD) {
        char buf[100];
        if (modem_state == MODEM_NOSYNC) {
            u8g2_print(0, 25, "   receiving ...");
            sprintf(buf, "CSM %d - codeblock starts ..", match);
            report(buf);
            Serial.printf("# %s\r\n", buf);
            modem_state = MODEM_SYNCED;
            sym_cnt = 0;
        } else if (modem_state == MODEM_SYNCED) {
            sym_cnt -= 32;
            sprintf(buf, "CSM %d - codeblock ended, frames %d symbols",
                    match, sym_cnt);
            report(buf);
            Serial.printf("# %s\r\n", buf);
            sptr = rcv_sym_start + sizeof(rcv_symbols) - sym_cnt - 32;
            sptr %= sizeof(rcv_symbols);
            modem_decode(sptr, sym_cnt);
            modem_state = MODEM_NOSYNC;
            u8g2_clearLine(25);
            memset(rcv_symbols, 0, sizeof(rcv_symbols));
        }
    }
    if (power > 100) {
        no_signal = 0;
    } else if (modem_state == MODEM_SYNCED && ++no_signal > 15) {
        // too many missing symbols
        char *s = "CSM missing: codeblock aborted";
        report(s);
        Serial.printf("# %s\r\n", s);
        modem_state = MODEM_NOSYNC;
        u8g2_clearLine(25);
        memset(rcv_symbols, 0, sizeof(rcv_symbols));
    }
}

void modem_demodulate()
{
    static uint64_t next_waterfall;

    for (int h = 0; h < 10; h++) { // do 10 FHTs in a row, at most

        uint32_t sp = rcv_audio_end + 1; // sample pointer
        sp %= ARRAY_LEN(rcv_audio);
        if (sp == rcv_audio_start) // no new data yet
            return;
        rcv_audio_end = sp;

        // copy audio samples for doing the Hartley transform
        int16_t fht_buf[SAMPLE_WIN];
        sp += ARRAY_LEN(rcv_audio) - SAMPLE_WIN;
        sp %= ARRAY_LEN(rcv_audio); // sample pointer
        for (int j = 0; j < SAMPLE_WIN; j++) {
            fht_buf[j] = rcv_audio[sp];
            sp = (sp + 1) % ARRAY_LEN(rcv_audio);
        }

        // do the Hartley transform
        float *psp = (float*) alloca((SAMPLE_WIN/2+1)*sizeof(float));
        float power;
        fht(fht_buf, SAMPLE_WIN, false, FREQ_BOOST, psp, &power);
        power = power_lpf.filterIn((int16_t)power);

        // batch energy levels into bins
        float tones[1 << BITS_PER_SYM];
        for (int i = 0; i < (1 << BITS_PER_SYM); i++) {
            tones[i] = 0;
            for (int j = 0; j < F_BIN_CNT; j++)
                tones[i] += psp[F_BIN_0 + F_BIN_CNT * i + j];
        }

        // pace symbol clock: measure tone diversity (max at symbol transition)
        float d = 0;
        for (int i = 0; i < (1 << BITS_PER_SYM); i++)
            for (int j = i+1; j < (1 << BITS_PER_SYM); j++)
                d += fabs(tones[i] - tones[j]);
        sym_clock = sym_clock_filter(d);
        if (clock_dump)
            Serial.printf("%g,\r\n", sym_clock);

        // find winner tone (symbol)
        int sym = -1;
        float m = -1;
        for (int i = 0; i < (1 << BITS_PER_SYM); i++) // pick winner tone
            if (tones[i] > m)
                m = tones[i], sym = i;

        uint8_t confid;
        { // compare energy of winner symbol to avg
            float avg = 0.001;
            for (int i = F_BIN_0; i < SAMPLE_WIN/2+1; i++) // skip bin 0
                avg += psp[i];
            avg /= SAMPLE_WIN/2 + 1 - 1 - F_BIN_0;
            float c = tones[sym]/F_BIN_CNT / avg;
            // below 1 is very bad, 1.5 is barely ok, 3 or above is very good
            c = c >= 1 ? 2*(c-1) : 0.0;
            if (c > 4) c = 4;
            confid = 127 * c / 4; // 0=no confidence, 127=high confidence
        }

        if (waterfall == 1 && millis() > next_waterfall) {
            show_waterfall(power,
                           sym, psp, SAMPLE_WIN/2+1, &confid, &sym_clock);
            next_waterfall = millis() + 300;
        }
        if (waterfall == 2 && millis() > next_waterfall) {
            static const char *g = " .:-=+*#%@"; // ASCIIART grey levels
            float ma = 120; // sqrt(700);
            Serial.printf("# %04x [", rcv_audio_end);
            // map power level to 0..9
            for (int i = 0; i < SAMPLE_WIN/2+1; i++) {
                int v = int(9.9 * psp[i] / ma);
                if (v < 0) v = 0;
                if (v > 9) v = 9;
                Serial.printf("%c", g[v]);
            }
            Serial.printf("] p=%d\r\n", (int)power);
            next_waterfall = millis() + 40; // less than 25 lines per second
        }

        modem_process_symbol(sym, confid, power);
    }
}

// ---------------------------------------------------------------------------

void modem_start_transmit()
{
    modem_state = MODEM_XMIT;

    u8g2_clearLine(25);
    u8g2_print(0, 25, "   transmitting ...");

    radio.transmit();
    delay(200); // wait 200msec sec before blasting off
}

void modem_end_transmit()
{
    radio.receive();
    u8g2_clearLine(25);

    modem_state = MODEM_NOSYNC;
    memset(rcv_audio, 0, sizeof(rcv_audio));
    memset(rcv_symbols, 0, sizeof(rcv_symbols));
}

void send_message(char *cmd)
{
    report("sending the following msg ...");
    // fields are: CSM - MSG - CRC - PADDING - PARITYBITS - CSM

    uint8_t msg[MESSAGE_BYTE_LENGTH];
    strcpy((char*) msg, TEST_MSG);
    char *cp = gps_to_tim();
    if (*cp) {
        strcpy((char*) msg + strlen((char*) msg), " tim=");
        strcpy((char*) msg + strlen((char*) msg), cp);
    }
    cp = gps_to_loc();
    if (*cp) {
        strcpy((char*) msg + strlen((char*) msg), " loc=");
        strcpy((char*) msg + strlen((char*) msg), cp);
    }
    if (cmd) {
        strcpy((char*) msg + strlen((char*) msg), " ");
        strcpy((char*) msg + strlen((char*) msg), cmd);
    }
    for (int i = strlen((char*) msg) + 1; i < sizeof(msg); i++)
        msg[i] = esp_random();
    report((char*)msg, true);
    hexdump(msg, sizeof(msg));
    Serial.printf("# msg: '%s'\r\n", (char*)msg);

    // bits for the message
    uint8_t bits[SYNC_MARK_LEN + PACKET_BIT_LEN + SYNC_MARK_LEN];
    for (int i = 0; i < sizeof(msg); i++)
        for (int j = 0; j < 8; j++)
            bits[SYNC_MARK_LEN + 8*i + j] = 0x1 & (msg[i] >> (7-j));
    uint16_t crc = crc12_b91(msg, sizeof(msg));
    for (int i = 0; i < CRC_BIT_LENGTH; i++)
        bits[SYNC_MARK_LEN + MESSAGE_BIT_LENGTH + i] = 0x01 & (crc >> (CRC_BIT_LENGTH-1 - i));
    for (int i = 0; i < PADDING_LENGTH; i++)
        bits[SYNC_MARK_LEN + MESSAGE_BIT_LENGTH + CRC_BIT_LENGTH + i] = 1;

    // partity check bits
    ldpc_encode_1024(bits + SYNC_MARK_LEN + PAYLOAD_BIT_LEN,
                     bits + SYNC_MARK_LEN, PAYLOAD_BIT_LEN);

    if (debug_output > 2) {
        Serial.printf("# outgoing %d pre-rnd bits (no CSM):\r\n#  ", PACKET_BIT_LEN);
        for (int i = 0; i < PACKET_BIT_LEN;) {
            if (i != 0 && (i % 48) == 0)
                Serial.printf("\r\n#  ");
            Serial.printf(" ");
            for (int j = 0; j < BITS_PER_SYM; j++)
                Serial.printf("%d", bits[SYNC_MARK_LEN + i++]);
        }
        Serial.println();
    }

    // randomization of code word
    randomizer_reset();
    for (int i = 0; i < PACKET_BIT_LEN; i++)
        bits[SYNC_MARK_LEN + i] ^= randomizer_next_bit();

    // pre- and append Codeblock Sync Marker (these are not randomized)
    uint64_t v = CSM;
    for (int i = 0; i < SYNC_MARK_LEN; i++) {
        bits[i] = bits[sizeof(bits) - SYNC_MARK_LEN + i] = v & 0x01;
        v >>= 1;
    }

    modem_start_transmit();
    modem_emit_bits(bits, sizeof(bits));
    modem_end_transmit();

    return;
}

// ---------------------------------------------------------------------------

void report_hdr()
{
    char ts[50];

    ts[0] = '\0';
    if (gps.date.isValid()) {
        int y = gps.date.year(), m = gps.date.month(), d = gps.date.day();
        sprintf(ts, "%04d-%02d-%02d ", y, m, d);
    }
    if (gps.time.isValid()) {
        int h = gps.time.hour(), m = gps.time.minute(), s= gps.time.second();
        sprintf(ts + strlen(ts), "%02d:%02d:%02dU: ", h, m, s);
    }
    if (*ts == '\0')
        sprintf(ts, "%d: ", millis());

    the_log.write((uint8_t*)ts, strlen(ts));
}

void report(char *buf, bool quote)
{
    report_hdr();

    if (quote)
        the_log.write((uint8_t*)"'", 1);
    the_log.write((uint8_t*)buf, strlen(buf));
    if (quote)
        the_log.write((uint8_t*)"'", 1);
    the_log.write((uint8_t*)"\r\n", 2);
}

void report_hex(uint8_t *data, int cnt)
{
    report_hdr();

    for (int i = 0; i < cnt; i++) {
        if ((i % 16) == 0)
            the_log.write((uint8_t*)"\r\n ", 3);
        char buf[4];
        sprintf(buf, " %02x", data[i]);
        the_log.write((uint8_t*)buf, 3);
    }
    the_log.write((uint8_t*)"\r\n", 2);
}

// ---------------------------------------------------------------------------

char* gps_to_tim() // fill static buffer with current date and time
{
    static char tim[30];

    tim[0] = '\0';

    if (gps.date.isValid()) {
      int y = gps.date.year(), m = gps.date.month(), d = gps.date.day();
      sprintf(tim, "%04d-%02d-%02d", y, m, d);
    }

    if (gps.time.isValid()) {
      int h = gps.time.hour(), m = gps.time.minute(), s= gps.time.second();
      if (tim[0] != '\0')
          strcpy(tim + strlen(tim), "_");
      sprintf(tim + strlen(tim), "%02d:%02d:%02dU", h, m, s);
    }

    return tim;
}

char* gps_to_loc() // fill static buffer with PlusCode of current location
{
    static char loc[20];

    loc[0] = '\0';

    if (gps.location.isValid() && gps.location.age() < 5000) {
        double lat = gps.location.lat(), lng = gps.location.lng();
        // https://groups.google.com/g/open-location-code/c/Ws9Bydnkii8
        int a = (lat + 90) * 1e6, b = (lng + 180) * 1e6;
        const char alpha[] = "23456789CFGHJMPQRVWX";
        for (int i = 0; i < 10; i++) {
            loc[i < 2 ? 10 - i : 9 - i] = alpha[b / 125 % 20];
            int d = b;
            b = a;
            a = d / 20;
        }
        loc[8] = '+';
    }

    return loc;
}

void gps_display() // draws date/time/location on screen
{
    static uint64_t next_update;
    uint64_t now = millis();
    if (now < next_update)
        return;
    next_update = now + 1000; // update display at most once a second

    if (gps.date.isUpdated()) {
      char dat[40];
      int y = gps.date.year(), m = gps.date.month(), d = gps.date.day();
      sprintf(dat, "%04d-%02d-%02d", y, m, d);
      u8g2_clearLine(40, false);
      u8g2_print(0, 40, dat);
    } else if (gps.date.age() > 5000)
      u8g2_clearLine(40);

    if (gps.time.isUpdated()) {
      char tim[40], loc[40];
      int h = gps.time.hour(), m = gps.time.minute(), s= gps.time.second();
      sprintf(tim, "%02d:%02d:%02dU", h, m, s);

      if (gps.location.isValid() && gps.location.age() < 5000) {
        double lat = gps.location.lat(), lng = gps.location.lng();
        // https://groups.google.com/g/open-location-code/c/Ws9Bydnkii8
        int a = (lat + 90) * 1e6, b = (lng + 180) * 1e6;
        const char alpha[] = "23456789CFGHJMPQRVWX";
        for (int i = 0; i < 10; i++) {
          loc[i < 2 ? 10 - i : 9 - i] = alpha[b / 125 % 20];
          int d = b;
          b = a;
          a = d / 20;
        }
        loc[8] = '+';
      } else
        loc[0] = '\0';

      u8g2_clearLine(52, false);
      int w = u8g2.getStrWidth(loc);
      u8g2_print(128-w, 52, loc, false);
      u8g2_print(0, 52, tim);
    } else if (gps.time.age() > 5000)
      u8g2_clearLine(52);
}

// ---------------------------------------------------------------------------

uint64_t end_of_status;
void set_status_msg(char *msg)
{
    static uint16_t status_width;
    bool refresh = 0;

    if (status_width != 0) {
        u8g2.setDrawColor(0);
        u8g2.drawBox(128 - status_width, 0, 128, 12);
        u8g2.setDrawColor(1);
        status_width = 0;
        refresh = true;
    }
    if (msg) {
        end_of_status = millis() + 2000; // timeout when status must be removed
        u8g2.setFont(u8g2_font_helvR08_tf);
        status_width = u8g2.getStrWidth(msg);
        u8g2_print(128 - status_width, 11, msg, false);
        refresh = true;
    }
    if (refresh)
        u8g2.sendBuffer();
}

void command_line(void)
{
    if (!Serial.available())
        return;
    char cmd = Serial.read();

    if (cmd == 'x') {
        Serial.println("\r\n ... rebooting ...\r\n");
        esp_restart();
    }
 
    if (cmd == 'c') {
        clock_dump = 1 - clock_dump;
        Serial.printf("\r\n# (symbol) clock dump now %d\r\n", data_dump);
        return;
    }

    if (cmd == 'd') {
        data_dump = 1 - data_dump;
        Serial.printf("\r\n# data dump now %d\r\n", data_dump);
        return;
    }

    if (cmd == 'L') {
        debug_output = (debug_output + 1) % 5;
        Serial.printf("\r\n# log level now %d\r\n", debug_output);
        return;
    }
 
    if (cmd == 'r') { // show log report
        Serial.println("# --- showing log content:");
        the_log.close();
        the_log = MyFS.open(LOG_FILE_NAME, FILE_READ);
        while(the_log.available()){
            Serial.write(the_log.read());
        }        
        the_log.close();
        the_log = MyFS.open(LOG_FILE_NAME, FILE_APPEND);
        Serial.println("# --- done showing log content");
        return;
    }

    if (cmd == 'R') { // reset log report
        the_log.close();
        MyFS.remove(LOG_FILE_NAME);
        the_log = MyFS.open(LOG_FILE_NAME, FILE_APPEND);
        Serial.println("# --- report file erased, starts new");
        return;
    }

    if (cmd == 's') { // send echo request message
        send_message(ECHOREQUEST);
        return;
    }

    if (cmd == 'w') {
        waterfall = (waterfall + 1) % 3;
        Serial.printf("\r\n# waterfall mode %d\r\n", waterfall);
        return;
    }

    Serial.println("\r\n# unknown command. Choose from");
    Serial.println("#   c  toggle dump of symbol clock value");
    Serial.println("#   d  toggle dump of sampled data");
    Serial.println("#   L  cycle through log levels");
    Serial.println("#   r  show report (log file)");
    Serial.println("#   R  reset report (log file)");
    Serial.println("#   s  send echo request msg (3-5sec)");
    Serial.println("#   w  cycle through waterfall modes");
    Serial.println("#   x  reboot");
}

// ---------------------------------------------------------------------------

void loop()
{
    command_line();

    if (end_of_status && millis() > end_of_status) {
        set_status_msg(NULL);
        end_of_status = 0;
    }

    radio_receive(); // serve the ADC
    if (modem_state != MODEM_XMIT)
        modem_demodulate();

    int cnt = 10; // limit nr of characters to digest, else we miss audio
    while (GPSSerial.available() and cnt-- > 0)
        gps.encode(GPSSerial.read());
    gps_display();

    for (int i = 0; i < ARRAY_LEN(buttonPins); i++)
        buttons[i].check();
}

// eof
