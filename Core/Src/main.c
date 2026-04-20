/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : 3-Phase Power Monitor & Oscilloscope Snapshot System
  * @details        : This firmware captures 200ms of 3-phase ADC data via
  * TIM2-triggered DMA and displays snapshots on a TFT screen
  * every 3 seconds with dynamic time-period labeling.
  *
  * @note           HARDWARE INPUT REQUIREMENTS:
  * Currently, this system utilizes the internal DACs to simulate
  * 3-phase AC voltages. When transitioning to measure real physical
  * AC grid voltages, the external hardware interface MUST provide
  * signal conditioning prior to reaching the ADC pins:
  * * 1. Attenuation: The AC peak-to-peak voltage must be stepped down
  *  to safely fit within the MCU's
  * 0V to 3.3V analog window.
  * 2. DC Biasing: The STM32 ADC cannot measure negative voltages.
  * Because real AC has negative half-cycles, external circuitry
  *  must shift the AC waveform up so its zero-crossing sits perfectly
  * at +1.65V DC.
  *
  * This firmware mathematically expects the 0V AC crossing to
  * evaluate to exactly 2048 on the 12-bit ADC (defined by DC_BIAS).
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */







/* ============================================================================
 * SYSTEM TIMING & FAULT PARAMETERS
 * ============================================================================ */
// Hardware fault simulation pulse width in milliseconds.
#define FAULT_INJECTION_TIME 120

/* AC SIGNAL CALIBRATION:
 * The 12-bit ADC reads 0 to 4095. A true 0V AC crossing sits exactly at the midpoint.
 */
#define DC_BIAS             2048.0f

/* THRESHOLD MULTIPLIERS:
 * Used to calculate the RMS limits dynamically.
 * - Sag (Dip): Standard 10% grid voltage drop threshold.
 * - Swell (Spike): Standard 10% grid voltage overage threshold.
 * - Hysteresis: 2% buffer prevents the system state from rapidly toggling (chattering)
 * when hovering right on the recovery boundary.
 */
#define SAG_RATIO           0.90f
#define SWELL_RATIO         1.10f
#define HYST_RATIO          0.02f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================================
 * ACQUISITION ENGINE (ADC/DMA SCALING)
 * ============================================================================ */
// Hardware sampling rate: 30kHz per channel.
#define ACQ_FREQ            30000
// Total memory depth per channel: (30,000 Hz * 0.2s) = 200ms window.
//had to reduce it to 100 ms for 6 channells acqusiition

#define SAMPLES_PER_CH      3000

/* REAL-TIME ANALYSIS WINDOWS:
 * We evaluate the power quality in discrete 20ms blocks (1 full 50Hz AC cycle).
 * At 30kHz, 20ms requires exactly 600 samples per channel.
 */
#define SAMPLES_20MS_TOTAL 1200 // Interleaved ADC1 offset (PhA + PhB = 600 * 2).
#define SAMPLES_20MS_CHAN  600  // 600 samples = exactly 20ms at 30kHz.

// Mathematical constant: The true physical time (in microseconds) between each ADC sample.
/*1,000,000 is the number of microseconds in one second.30,000 is your sampling frequency.*/
#define REAL_SAMPLE_US (1000000.0f / ACQ_FREQ)

/* ============================================================================
 * TFT DISPLAY & RENDERING CONSTANTS
 * ============================================================================ */
// Width of the TFT oscilloscope grid in physical pixels.
#define SAMPLE_COUNT        260
// Maximum possible timeframe the screen can map horizontally.
#define DISPLAY_TIME_MS     100
// Time per pixel horizontally (~769us).
#define SAMPLE_INTERVAL_US  ((DISPLAY_TIME_MS * 1000) / SAMPLE_COUNT)

// Base decimation ratio: How many raw ADC samples represent a single pixel at maximum zoom.
#define DECIMATION          (SAMPLES_PER_CH/SAMPLE_COUNT)

// Resolution of the generated reference DAC 3-phase Look-Up Table.
#define SINE_STEPS          400

/* VERTICAL GRAPHICS SCALING:
 * Maps the internal electrical values to physical screen coordinates.
 */
#define GRAPH_WIDTH         260
// The physical Y-pixel acting as the 0V reference line.
#define GRAPH_Y_CENTER      90
// Visual Ratio: Fits 1.65V peak perfectly into an 80px half-height.
#define PIXELS_PER_VOLT     48.48f
// Hardware Ratio: 3.3V reference / 4095 12-bit ADC steps.
#define VOLTS_PER_STEP      0.000805f

/* ============================================================================
 * EVENT LOGGING & CAPTURE MARKERS
 * ============================================================================ */
#define ANOMALY_THRESHOLD    4090   // Absolute upper ceiling (detects hardware clipping).
#define ANOMALY_LOW          10     // Absolute lower floor.
#define PRETRIGGER_SAMPLES   1500   // Retained history before fault (approx 5 cycles at 50Hz).
#define POST_SAMPLES         1500   // Delay to capture post-fault behavior before locking snapshot.
#define ANOMALY_MARKER_X     130    // The physical X-pixel where the fault trigger is centered.
#define COLOR_ANOMALY        0xFFE0 // RGB565 Yellow indicator for UI fault mapping.

// System memory allocation limit for stored distinct event logs.
#define ANOMALY_LOG_SIZE     5


#define NUM_VIEWS           5      // V_all, I_all, VA+IA, VB+IB, VC+IC

// Buffer length math
#define V_BUF_LEN           (SAMPLES_PER_CH * 5 * 2) // 30,000 index depth
#define V_C_BUF_LEN         (SAMPLES_PER_CH * 2)     // 6,000 index depth
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

// ========================================================
// 🛠️ USER CONFIGURATION DASHBOARD (Change these safely!) 🛠️
// ========================================================

/* 1. HORIZONTAL TIME SCALING (Time Zoom)
 * Target Format: Evaluated in terms of standard 50Hz AC cycles (20ms each).
 * Math: display_cycles (4.0) * 20ms = 80ms total viewable window on the TFT.
 * Adjusting this variable directly alters the dynamic decimation ratio during rendering.
 */
float display_cycles = 5.0f; // Change to 5.0f for more time, 2.0f for zoomed in
//at 2.0f the recover snapshot breaks since
//2 cycles are checked after recovery and then a snapshot is taken


// 2. VOLTAGE ZOOM: Higher number = taller waves (Default: 55.21)

/* * DERIVATION OF VOLTAGE SCALE (Calibration):
 * 1. Hardware: 3.3V / 4095 steps = 0.000805 Volts per ADC count.
 * 2. Signal: 1800 counts peak = (1800 * 0.000805) = 1.449V peak.
 * 3. Screen: Grid half-height is 80 pixels.
 * 4. Goal: Make 1.449V fill exactly 80 pixels.
 * 5. Math: 80 / 1.449 = 55.21
 */
float voltage_scale = 40.00f; // Try 80.0f to zoom in, 20.0f to zoom out

// 3. PHASE MUTES: Bit-flags to bypass the rendering loop for specific channels (1=Draw, 0=Hide).
uint8_t show_phase_A = 1;
uint8_t show_phase_B = 0;
uint8_t show_phase_C = 0;

// 4.  CURRENT MUTES (Amps) -
uint8_t show_curr_A = 1;  // Set to 0 if not wired
uint8_t show_curr_B = 1;
uint8_t show_curr_C = 1;

// ========================================================

// The resulting physical timeframe represented by the current display_cycles selection.
// Default to 200ms (showing 100ms before and 100ms after the trigger)
float ui_time_window_ms = 200.0f;


// ============================================================================
// ⚡ POWER MONITORING THRESHOLDS
// ============================================================================

/* CONTINUOUS ANALYSIS POINTER:
 * Tracks the index position inside the circular DMA buffer. We evaluate the power
 * quality every time the DMA advances by exactly 'SAMPLES_20MS_TOTAL'.
 */
uint32_t last_check_idx = 0;
// Expected ideal raw ADC peak amplitude.
float target_amplitude = 1800.0f;

/* DYNAMIC RMS EVALUATION LIMITS:
 * Pre-calculated at startup to avoid expensive floating-point math inside the active loop.
 * The system evaluates Mean Square (MS) directly against the squared versions of these thresholds.
 */
float v_nom_rms;        // Ideal grid RMS calculated from target_amplitude / sqrt(2).
float v_sag_thresh;     // Lower bound before fault declared.
float v_swell_thresh;   // Upper bound before fault declared.
float v_recov_thresh;   // Required healthy voltage to exit fault state.

// Debounce counter for Grid Recovery.
uint8_t recovery_count = 0;
// Grid must remain healthy for "RECOVERY_CONFIRMATION" contiguous 20ms blocks to officially "recover".
#define RECOVERY_CONFIRMATION  2

/* --- DIAGNOSTIC VARIABLES delete later --- */
volatile char trigger_chan;      // Stores the event identifier: 'F' (Fault) or 'R' (Recovery).
volatile uint16_t trigger_val;   // Stores the raw ADC value that caused the breach.


/* Timing & Control */
// System tick marker for the automated 3-second live refresh cycle.
uint32_t lastDisplayRefresh = 0;
// Temporary string buffer for formatting textual UI data before printing to TFT.
char msg[50];

// ============================================================================
// 🗄️ MEMORY BUFFERS & LOG ARCHITECTURE
// ============================================================================

/* Hardware DAC Buffers (3-Phase Reference Generation)
 * Look-Up Tables fed directly to the DACs via hardware DMA.
 */
uint16_t dac_sine_A[SINE_STEPS];
uint16_t dac_sine_B[SINE_STEPS];
uint16_t dac_sine_C[SINE_STEPS];

/* DMA Circular Buffers (Always filling in background via TIM2)
 * ADC1 Size: 6000 samples * 2 channels (PhA + PhB) * 2 (Double Buffering) = 24,000 index depth.
 * ADC2 Size: 6000 samples * 1 channel (PhC) * 2 (Double Buffering) = 12,000 index depth.
 */
// ADC1 holds 5 channels (Va, Vb, Ia, Ib, Ic)
uint16_t adc1_buf[V_BUF_LEN];
// ADC2 holds 1 channel (Vc)
uint16_t adc2_buf[V_C_BUF_LEN];

/* Control Flags */
// Synchronization flags mapping DMA half-complete/full-complete interrupts to the display renderer.
volatile uint8_t snapshot_ready = 0;
volatile uint32_t buffer_offset = 0;


/* Snapshot Buffers (Frozen "Photo" for display)
 * Down-sampled data formatted strictly for 260px rendering width.
 */
uint16_t snapA[SAMPLE_COUNT];
uint16_t snapB[SAMPLE_COUNT];
uint16_t snapC[SAMPLE_COUNT];

/* Measurement Variables (Calculated in background) */
//float vInstA;

// Analytical variables tracking the mathematical period (ms) of the captured waves.
float pA, pB, pC;



// ============================================================================
// 📸 CAPTURE STATE MACHINE (The "Camera")
// ============================================================================
// The Camera only cares about taking pictures now, not power levels.
typedef enum {
    CAMERA_IDLE,     // Waiting for a change in grid health.
    CAMERA_ARMED,    // Change detected, waiting for post-trigger data to fill.
    CAMERA_CAPTURED  // Data ready to be saved to persistent log.
} CameraState;

volatile CameraState camera_state = CAMERA_IDLE;

/* GRID HEALTH BITMASK:
 * Tracks the fault status of individual phases. 0 = All phases healthy.
 * Bit 0: Phase A | Bit 1: Phase B | Bit 2: Phase C
 */
uint8_t grid_state = 0;

// Independent recovery counters for each phase (Scalable!)
uint8_t recov_count[3] = {0, 0, 0};


// Hardware tracking markers utilized during the precise moment a fault is tripped.
volatile uint32_t post_samples_collected = 0;
//volatile uint8_t  trigger_half = 0;
//volatile uint16_t trigger_sample_idx = 0;
volatile uint32_t trigger_tick = 0; // The exact system HAL_Tick() the fault occurred.


//volatile uint16_t post_samples_collected = 0;
volatile uint8_t trigger_half = 0; // Tracks which DMA half saw the anomaly
volatile uint8_t anomaly_detected = 0; // Global indicator flag.

/* Dedicated Anomaly Snapshots */
uint16_t anomaly_snapA[SAMPLE_COUNT];
uint16_t anomaly_snapB[SAMPLE_COUNT];
uint16_t anomaly_snapC[SAMPLE_COUNT];

// The exact sample (0-5999) within the half that triggered the mathematical fault.
volatile uint16_t trigger_sample_idx = 0;

// The Ring Buffer Log: Three-dimensional array storing fully captured anomaly events.
uint16_t anomaly_logA[ANOMALY_LOG_SIZE][SAMPLE_COUNT];
uint16_t anomaly_logB[ANOMALY_LOG_SIZE][SAMPLE_COUNT];
uint16_t anomaly_logC[ANOMALY_LOG_SIZE][SAMPLE_COUNT];


// Metadata array linking the timestamps to their specific log array.
uint32_t anomaly_timestamps[ANOMALY_LOG_SIZE];

char anomaly_types[ANOMALY_LOG_SIZE]; // <--- ADD THIS NEW LINE

uint8_t log_write_idx = 0;  // Where the next anomaly will be saved (0-4).
uint8_t log_count = 0;      // How many anomalies are currently stored (max 5).
int8_t  view_idx = -1;      // Display Pointer: -1 means Live Mode, 0-4 means viewing a specific log.


// Defines the 16-bit RGB565 colors used to draw each phase line.
uint16_t traceColorA = 0x07E0; // Green
uint16_t traceColorB = 0xF800; // Red
uint16_t traceColorC = 0x001F; // Blue

uint32_t btn_press_time = 0;
uint8_t  last_btn_state = GPIO_PIN_RESET;

// Thresholds for the fast MS math
float lowSq;
float highSq;
float recovSq;


/* Font Table
 * 5x8 bitmapped character set for independent TFT rendering.
 */
const uint8_t font5x8[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
    {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x00,0x7F,0x41,0x41},
    {0x02,0x04,0x08,0x10,0x20}, {0x41,0x41,0x7F,0x00,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x08,0x14,0x54,0x54,0x3C}, {0x7F,0x08,0x04,0x04,0x78},
    {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x00,0x7F,0x10,0x28,0x44}, {0x00,0x41,0x7F,0x40,0x00},
    {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20}, {0x04,0x3F,0x44,0x40,0x20},
    {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44},
    {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00}, {0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00}, {0x08,0x08,0x2A,0x1C,0x08}
};


//_________channel Ia,Ib,Ic acquisition______

/* Current Buffer (IA, IB, IC) */
// Layout: [IA, IB, IC, IA, IB, IC...]
//uint16_t adc3_buf[SAMPLES_PER_CH * 3 * 2];

/* Snapshots for the screen */
uint16_t snapIA[SAMPLE_COUNT], snapIB[SAMPLE_COUNT], snapIC[SAMPLE_COUNT];


#define NUM_CHANNELS_ADC3   3      // IA, IB, IC

typedef enum {
    VIEW_V_ALL,     // Va, Vb, Vc
    VIEW_I_ALL,     // Ia, Ib, Ic
    VIEW_PHASE_A,   // Va + Ia
    VIEW_PHASE_B,   // Vb + Ib
    VIEW_PHASE_C    // Vc + Ic
} DisplayView;

DisplayView current_view = VIEW_V_ALL;


// Current scaling and colors
float current_scale = 40.0f;
uint16_t traceColorIA = 0xFE19; // pink
uint16_t traceColorIB = 0x07FF; // Cyan
uint16_t traceColorIC = 0xF81F; // Magenta


/* Expand Anomaly Logs to store Current data too */
uint16_t anomaly_logIA[ANOMALY_LOG_SIZE][SAMPLE_COUNT];
uint16_t anomaly_logIB[ANOMALY_LOG_SIZE][SAMPLE_COUNT];
uint16_t anomaly_logIC[ANOMALY_LOG_SIZE][SAMPLE_COUNT];

// Colors
uint16_t colorCurrent = 0xFFE0; // Yellow for Amps in V+I view

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TFT_Init(void);
void TFT_FillScreen(uint16_t color);
void TFT_Print(uint16_t x, uint16_t y, char* str, uint16_t color);
void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void TFT_DrawFilledRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void TFT_ConnectDots(uint16_t x, int old_y, int new_y, uint16_t color);
void Draw_Oscilloscope_Grid(uint32_t totalTime);
void Generate_3Phase_LUT(void);
float Get_Period_MS(uint16_t* snap, uint32_t decimation) ;
void TFT_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void Set_Signal_Failure(uint8_t active);
float Calculate_Buffer_MS(uint16_t* buffer, uint32_t start_idx, uint32_t length, uint8_t stride, uint8_t offset, uint32_t total_buf_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ============================================================================
 *  main FUNCTIONS
 * ============================================================================ */

void Process_Grid_State(void)
{
    // --- SECTION 1: THE FAST PEEK (RMS Bitmask Evaluation) ---
    uint32_t V_LEN  = V_BUF_LEN;     // Uses dynamic 30,000 define
    uint32_t VC_LEN = V_C_BUF_LEN;   // Uses dynamic 6,000 define

    // Identify current DMA position
    uint32_t current_dma_pos = V_LEN - __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
    current_dma_pos = current_dma_pos - (current_dma_pos % 5); // Force alignment to Va

    // CALCULATE BACKLOG: How many samples are waiting?
    uint32_t samples_waiting = (current_dma_pos >= last_check_idx) ?
                                (current_dma_pos - last_check_idx) :
                                (V_LEN - last_check_idx + current_dma_pos);

    // Evaluate in blocks of 20ms (600 samples * 5 channels = 3000 elements)
    while (samples_waiting >= (SAMPLES_20MS_CHAN * 5))
    {
        float ms[3];
        // STRIDE is now 5! Offset 0 = Va, Offset 1 = Vb.
        ms[0] = Calculate_Buffer_MS(adc1_buf, last_check_idx, SAMPLES_20MS_CHAN, 5, 0, V_LEN);
        ms[1] = Calculate_Buffer_MS(adc1_buf, last_check_idx, SAMPLES_20MS_CHAN, 5, 1, V_LEN);
        ms[2] = Calculate_Buffer_MS(adc2_buf, last_check_idx / 5, SAMPLES_20MS_CHAN, 1, 0, VC_LEN); // ADC2 remains unchanged

        uint8_t new_grid_state = grid_state;

        for (int i = 0; i < 3; i++) {
            uint8_t is_muted = 0;
            if (i == 0 && !show_phase_A) is_muted = 1;
            if (i == 1 && !show_phase_B) is_muted = 1;
            if (i == 2 && !show_phase_C) is_muted = 1;

            if (is_muted) {
                new_grid_state &= ~(1 << i);
                recov_count[i] = 0;
                continue;
            }

            if (ms[i] < lowSq || ms[i] > highSq) {
                new_grid_state |= (1 << i);
                recov_count[i] = 0;
            }
            else if (ms[i] > recovSq && ms[i] < highSq) {
                if (grid_state & (1 << i)) {
                    if (++recov_count[i] >= RECOVERY_CONFIRMATION) {
                        new_grid_state &= ~(1 << i);
                        recov_count[i] = 0;
                    }
                }
            }
            else {
                recov_count[i] = 0;
            }
        }

        uint8_t changed_bits = grid_state ^ new_grid_state;

        // Trigger log only if state actually changed AND camera is not busy
        if (changed_bits > 0 && camera_state == CAMERA_IDLE)
        {
            camera_state = CAMERA_ARMED;
            trigger_tick = HAL_GetTick();
            trigger_sample_idx = last_check_idx;

            if ((changed_bits & new_grid_state) > 0) trigger_chan = 'F';
            else trigger_chan = 'R';
        }

        // Advance the math pointer by exactly 1 block (3000 elements)
        grid_state = new_grid_state;
        last_check_idx = (last_check_idx + (SAMPLES_20MS_CHAN * 5)) % V_LEN;

        // Update remaining backlog count for the next 'while' iteration
        samples_waiting = (current_dma_pos >= last_check_idx) ?
                          (current_dma_pos - last_check_idx) :
                          (V_LEN - last_check_idx + current_dma_pos);
    }
}

void Validate_Post_Capture(void)
{
    // --- SECTION 2: POST-TRIGGER CAPTURE VALIDATION ---

    // Once armed, the system waits passively until the background DMA collects
    // enough "future" data (POST_SAMPLES) so the event is centered visibly on the TFT.
    if (camera_state == CAMERA_ARMED)
    {
        uint32_t V_LEN = V_BUF_LEN; // Use the dynamic define, not hardcoded 12000!

        // Get current DMA position from ADC1 (The "Master" timing reference)
        uint32_t current_dma_pos = V_LEN - __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
        current_dma_pos = current_dma_pos - (current_dma_pos % 5); // Keep 5-channel alignment

        // Calculate how many samples the DMA has moved since the trigger moment
        uint32_t moved;
        if (current_dma_pos >= trigger_sample_idx) {
            moved = current_dma_pos - trigger_sample_idx;
        } else {
            // Handle the circular wrap-around
            moved = V_LEN - trigger_sample_idx + current_dma_pos;
        }

        // 'moved' counts array elements. ADC1 has 5 channels, so 1 time-sample = 5 elements.
        // Multiply POST_SAMPLES by 5 to wait exactly 50ms!
        if (moved >= (POST_SAMPLES * 5)) {
            camera_state = CAMERA_CAPTURED;
        }
    }
}


void Assemble_Anomaly_Log(uint8_t *do_refresh)
{
    // --- SECTION 3: SILENT LOG ASSEMBLY (ADC1 5-CHANNEL + ADC2 1-CHANNEL) ---

    // Buffer is full. Lock the specific data sequence and copy it safely into static RAM.
    if (camera_state == CAMERA_CAPTURED)
    {
        // 1. Calculate the necessary data compression ratio based on user's time-zoom.
        uint32_t samples_to_save = (uint32_t)(display_cycles * 20.0f * 30.0f);
        uint32_t log_decimation = samples_to_save / SAMPLE_COUNT;
        if (log_decimation < 1) log_decimation = 1;

        // 2. Define our buffer lengths for wrapping math
        uint32_t V_LEN = V_BUF_LEN;   // ADC1: Va, Vb, Ia, Ib, Ic
        uint32_t VC_LEN = V_C_BUF_LEN; // ADC2: Vc

        // 3. Backtrack from the trigger point to find the "Start" point for drawing.
        int samples_to_go_back = 130 * log_decimation;

        // --- CALCULATE START INDEX (ADC1: 5 Channels) ---
        // Multiply samples_to_go_back by 5 because there are 5 elements per time-sample.
        int start_idx_v = (trigger_sample_idx + V_LEN - (samples_to_go_back * 5)) % V_LEN;

        // Force alignment to the start of a 5-channel block (Phase A Voltage)
        start_idx_v = start_idx_v - (start_idx_v % 5);

        // 4. Downsample and copy all 6 channels from the DMA ring buffers.
        for (int j = 0; j < SAMPLE_COUNT; j++) {
            int offset_steps = j * log_decimation;

            // --- EXTRACT FROM ADC1 (Stride 5) ---
            int curV = (start_idx_v + (offset_steps * 5)) % V_LEN;
            anomaly_logA[log_write_idx][j]  = adc1_buf[curV + 0]; // Va
            anomaly_logB[log_write_idx][j]  = adc1_buf[curV + 1]; // Vb
            anomaly_logIA[log_write_idx][j] = adc1_buf[curV + 2]; // Ia
            anomaly_logIB[log_write_idx][j] = adc1_buf[curV + 3]; // Ib
            anomaly_logIC[log_write_idx][j] = adc1_buf[curV + 4]; // Ic

            // --- EXTRACT FROM ADC2 (Stride 1) ---
            // Divide the ADC1 index by 5 to find the equivalent time-moment in ADC2
            int curVC = ((start_idx_v / 5) + offset_steps) % VC_LEN;
            anomaly_logC[log_write_idx][j] = adc2_buf[curVC];     // Vc
        }

        // 5. Save metadata tags to correlate UI identifiers with this array slot.
        anomaly_timestamps[log_write_idx] = trigger_tick;
        anomaly_types[log_write_idx] = trigger_chan;

        // === THE CRITICAL HISTORY FLUSH ===
        // Jump the math pointer to the CURRENT DMA position.
        // This stops the system from re-detecting the same fault "tail".
        last_check_idx = (V_LEN - __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle));
        last_check_idx = last_check_idx - (last_check_idx % 5); // Ensure alignment

        // 6. Reset camera state machine to permit future fault detection.
        camera_state = CAMERA_IDLE;

        // We DO NOT refresh the screen when a fault starts.
        if (trigger_chan == 'F') {
            printf("LOG: Fault Captured (Silently)\n");
            // *do_refresh = 0; (Leave it 0)
        } else {
            printf("LOG: Recovery Captured (Updating UI)\n");
            *do_refresh = 1; // Now that the event is over, draw the screen!
        }

        // 7. Advance the circular RAM storage index to prevent memory overflows.
        log_write_idx = (log_write_idx + 1) % ANOMALY_LOG_SIZE;
        if (log_count < ANOMALY_LOG_SIZE) log_count++;
    }
}


void Handle_Button_Logic(uint8_t *do_refresh)
{
    // --- SECTION 4: TRANSIENT INJECTOR & VIEW TOGGLE LOGIC ---

    // Read physical hardware pin state.
    uint8_t current_btn = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    // PERSISTENCE VARIABLES
    static uint32_t transient_end_tick = 0;
    static uint8_t  is_transient_active = 0;
    static uint8_t  long_press_handled = 0;   // Used for the 1s Fault Injection
    static uint8_t  view_cycle_handled = 0;   // Used for the 2s View Change

    // 1. SELF-HEALING LOGIC (Keep exactly as you had it)
    // Overrides the manual fault and restores DAC output when timer expires.
    if (is_transient_active && (HAL_GetTick() >= transient_end_tick)) {
        Set_Signal_Failure(0);
        is_transient_active = 0;
        printf("SIM: 30ms Transient RESTORED\n");
    }

    // 2. PRESS EDGE DETECTION
    if (current_btn == GPIO_PIN_SET && last_btn_state == GPIO_PIN_RESET) {
        btn_press_time = HAL_GetTick();
        long_press_handled = 0;
        view_cycle_handled = 0; // Reset both gatekeepers
    }

    // 3. CONTINUOUS HOLD DETECTION (The "Multi-Zone" Logic)
    if (current_btn == GPIO_PIN_SET && last_btn_state == GPIO_PIN_SET) {
        uint32_t hold_duration = HAL_GetTick() - btn_press_time;

        // ZONE A: 1.0 Second - Inject Fault
        if (hold_duration > 1000 && !long_press_handled) {
            Set_Signal_Failure(1);
            transient_end_tick = HAL_GetTick() + FAULT_INJECTION_TIME;
            is_transient_active = 1;
            long_press_handled = 1; // Mark that we've triggered the fault
            printf("SIM: Transient INJECTED\n");
        }

        // ZONE B: 2.0 Seconds - Cycle Display View (New Feature!)
        if (hold_duration > 2000 && !view_cycle_handled) {
            current_view = (current_view + 1) % NUM_VIEWS;
            *do_refresh = 1;        // Force screen to update
            view_cycle_handled = 1; // Mark that we've switched the view
            printf("UI: View Changed to Mode %d\n", current_view);
        }
    }

    // 4. RELEASE EDGE DETECTION (Navigation)
    if (current_btn == GPIO_PIN_RESET && last_btn_state == GPIO_PIN_SET) {
        uint32_t duration = HAL_GetTick() - btn_press_time;

        // Only process a "Short Press" if we didn't trigger a fault or a view change
        if (!long_press_handled && !view_cycle_handled && duration > 50) {
            if (log_count > 0) {
                view_idx++;
                if (view_idx >= (int8_t)log_count) view_idx = -1;
            } else {
                view_idx = -1;
            }
            *do_refresh = 1;
        }
    }

    last_btn_state = current_btn;
}

void Render_Display_Engine(uint8_t do_refresh)
{
    // --- SECTION 5: DYNAMIC REFRESH & RENDERING ENGINE ---

    // Gatekeeper: Draw if 3s timer expires (Live) OR if a manual event/toggle occurs.
    if ((HAL_GetTick() - lastDisplayRefresh >= 3000 && view_idx == -1 && snapshot_ready) || do_refresh)
    {
        lastDisplayRefresh = HAL_GetTick();
        snapshot_ready = 0;

        // Calculate Dynamic Decimation
        uint32_t samples_to_show = (uint32_t)(display_cycles * 20.0f * 30.0f);
        uint32_t dynamic_decimation = samples_to_show / SAMPLE_COUNT;
        if (dynamic_decimation < 1) dynamic_decimation = 1;

        // --- STEP 1: DATA EXTRACTION (ADC1 = 5 Channels, ADC2 = 1 Channel) ---
        if (view_idx == -1) {
            // LIVE MODE extraction
            for (int i = 0; i < SAMPLE_COUNT; i++) {
                int r = i * dynamic_decimation;
                if (r >= SAMPLES_PER_CH) r = SAMPLES_PER_CH - 1;

                // Live extraction from the 5-channel buffer (ADC1)
                snapA[i]  = adc1_buf[buffer_offset + (r * 5) + 0]; // Va
                snapB[i]  = adc1_buf[buffer_offset + (r * 5) + 1]; // Vb
                snapIA[i] = adc1_buf[buffer_offset + (r * 5) + 2]; // Ia
                snapIB[i] = adc1_buf[buffer_offset + (r * 5) + 3]; // Ib
                snapIC[i] = adc1_buf[buffer_offset + (r * 5) + 4]; // Ic

                // Phase C Voltage from ADC2 (Divide ADC1's buffer_offset by 5)
                snapC[i]  = adc2_buf[(buffer_offset / 5) + r];     // Vc
            }
        } else {
            // LOG MODE extraction
            for (int i = 0; i < SAMPLE_COUNT; i++) {
                snapA[i] = anomaly_logA[view_idx][i];
                snapB[i] = anomaly_logB[view_idx][i];
                snapC[i] = anomaly_logC[view_idx][i];
                snapIA[i] = anomaly_logIA[view_idx][i];
                snapIB[i] = anomaly_logIB[view_idx][i];
                snapIC[i] = anomaly_logIC[view_idx][i];
            }
        }

        // --- STEP 2: UI PREPARATION ---
        ui_time_window_ms = display_cycles * 20.0f;
        Draw_Oscilloscope_Grid((uint32_t)ui_time_window_ms);

        if (view_idx == -1) {
            // --- LIVE HEADER ---
            if (current_view == VIEW_V_ALL)        TFT_Print(10, 225, "VIEW: VOLTAGES (ABC)", 0xFFFF);
            else if (current_view == VIEW_I_ALL)   TFT_Print(10, 225, "VIEW: CURRENTS (ABC)", 0xFFFF);
            else if (current_view == VIEW_PHASE_A) TFT_Print(10, 225, "VIEW: PHASE A (V+I)", 0xFFFF);
            else if (current_view == VIEW_PHASE_B) TFT_Print(10, 225, "VIEW: PHASE B (V+I)", 0xFFFF);
            else if (current_view == VIEW_PHASE_C) TFT_Print(10, 225, "VIEW: PHASE C (V+I)", 0xFFFF);

            sprintf(msg, "NOW: T+%.1fs", (float)HAL_GetTick() / 1000.0f);
            TFT_Print(210, 10, msg, 0xFFFF);
        } else {
            // --- LOG HEADER (The Info You Needed) ---
            float t_trig = (float)anomaly_timestamps[view_idx] / 1000.0f;
            float t_start = t_trig - (ui_time_window_ms / 2000.0f);
            float t_end   = t_trig + (ui_time_window_ms / 2000.0f);

            if (anomaly_types[view_idx] == 'F') {
                sprintf(msg, "STATUS: FAULT %d/%d", view_idx + 1, log_count);
                TFT_Print(10, 225, msg, 0xF800);
                TFT_Print(120, 10, "*EVENT ~ +-20MS PRIOR/AFTER", 0xFFFF);
            } else {
                sprintf(msg, "STATUS: RECOVER %d/%d", view_idx + 1, log_count);
                TFT_Print(10, 225, msg, 0x07E0);
                TFT_Print(120, 10, "*EVENT ~ +-40MS PRIOR/AFTER", 0xFFFF);
            }

            // Print Start, End, and Trigger times
            sprintf(msg, "START:%.2fs", t_start);  TFT_Print(40, 188, msg, 0x7BEF);
            sprintf(msg, "END:%.2fs", t_end);      TFT_Print(230, 188, msg, 0x7BEF);
            sprintf(msg, "TRIG: T+%.2fs", t_trig); TFT_Print(165, 225, msg, 0xFFFF);

            TFT_DrawFastVLine(ANOMALY_MARKER_X + 40, 10, 160, 0xFFFF);
        }

        // --- STEP 3: ANALYTICS ---
        // Calculate periods for both Voltages and Currents
        pA = Get_Period_MS(snapA, dynamic_decimation);
        pB = Get_Period_MS(snapB, dynamic_decimation);
        pC = Get_Period_MS(snapC, dynamic_decimation);
        float pIA = Get_Period_MS(snapIA, dynamic_decimation);
        float pIB = Get_Period_MS(snapIB, dynamic_decimation);
        float pIC = Get_Period_MS(snapIC, dynamic_decimation);

        // Print Voltage Analytics (Top Row: Y=195)
        sprintf(msg, "PHA|%.1fms", pA); TFT_Print(10, 195, msg, traceColorA);
        sprintf(msg, "PHB|%.1fms", pB); TFT_Print(110, 195, msg, traceColorB);
        sprintf(msg, "PHC|%.1fms", pC); TFT_Print(210, 195, msg, traceColorC);

        // Print Current Analytics (Bottom Row: Y=210)
        sprintf(msg, "I-A|%.1fms", pIA); TFT_Print(10, 210, msg, traceColorIA);
        sprintf(msg, "I-B|%.1fms", pIB); TFT_Print(110, 210, msg, traceColorIB);
        sprintf(msg, "I-C|%.1fms", pIC); TFT_Print(210, 210, msg, traceColorIC);

        // --- STEP 4: WAVEFORM RENDERING (WITH MUTES) ---
        for (int x = 1; x < SAMPLE_COUNT; x++)
        {
            switch(current_view)
            {
                case VIEW_V_ALL:
                    if (show_phase_A) TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapA[x-1]-2048)*VOLTS_PER_STEP*voltage_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapA[x]-2048)*VOLTS_PER_STEP*voltage_scale), traceColorA);
                    if (show_phase_B) TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapB[x-1]-2048)*VOLTS_PER_STEP*voltage_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapB[x]-2048)*VOLTS_PER_STEP*voltage_scale), traceColorB);
                    if (show_phase_C) TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapC[x-1]-2048)*VOLTS_PER_STEP*voltage_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapC[x]-2048)*VOLTS_PER_STEP*voltage_scale), traceColorC);
                    break;

                case VIEW_I_ALL:
                    if (show_curr_A)  TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapIA[x-1]-2048)*VOLTS_PER_STEP*current_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapIA[x]-2048)*VOLTS_PER_STEP*current_scale), traceColorIA);
                    if (show_curr_B)  TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapIB[x-1]-2048)*VOLTS_PER_STEP*current_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapIB[x]-2048)*VOLTS_PER_STEP*current_scale), traceColorIB);
                    if (show_curr_C)  TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapIC[x-1]-2048)*VOLTS_PER_STEP*current_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapIC[x]-2048)*VOLTS_PER_STEP*current_scale), traceColorIC);
                    break;

                case VIEW_PHASE_A:
                    if (show_phase_A) TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapA[x-1]-2048)*VOLTS_PER_STEP*voltage_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapA[x]-2048)*VOLTS_PER_STEP*voltage_scale), traceColorA);
                    if (show_curr_A)  TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapIA[x-1]-2048)*VOLTS_PER_STEP*current_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapIA[x]-2048)*VOLTS_PER_STEP*current_scale), traceColorIA);
                    break;

                case VIEW_PHASE_B:
                    if (show_phase_B) TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapB[x-1]-2048)*VOLTS_PER_STEP*voltage_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapB[x]-2048)*VOLTS_PER_STEP*voltage_scale), traceColorB);
                    if (show_curr_B)  TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapIB[x-1]-2048)*VOLTS_PER_STEP*current_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapIB[x]-2048)*VOLTS_PER_STEP*current_scale), traceColorIB);
                    break;

                case VIEW_PHASE_C:
                    if (show_phase_C) TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapC[x-1]-2048)*VOLTS_PER_STEP*voltage_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapC[x]-2048)*VOLTS_PER_STEP*voltage_scale), traceColorC);
                    if (show_curr_C)  TFT_ConnectDots(x, GRAPH_Y_CENTER - (int)((float)(snapIC[x-1]-2048)*VOLTS_PER_STEP*current_scale),
                                                        GRAPH_Y_CENTER - (int)((float)(snapIC[x]-2048)*VOLTS_PER_STEP*current_scale), traceColorIC);
                    break;
            }
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
//  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */

  /* ============================================================================
   * SYSTEM INITIALIZATION & HARDWARE STARTUP
   * ============================================================================ */

  /* DYNAMIC THRESHOLD CALCULATION:
   * Translates the nominal peak target into RMS values using V_rms = V_peak / sqrt(2).
   * Generates strict upper (swell) and lower (sag) limits, plus a recovery buffer.
   */
  v_nom_rms      = target_amplitude / sqrtf(2.0f);
  v_sag_thresh   = v_nom_rms * SAG_RATIO;
  v_swell_thresh = v_nom_rms * SWELL_RATIO;
  v_recov_thresh = v_sag_thresh + (v_nom_rms * HYST_RATIO);

  /* LCD INITIALIZATION:
   * Allow power to stabilize, execute hardware reset sequence, and clear screen.
   */
  HAL_Delay(500);
  TFT_Init();
  TFT_FillScreen(0x0000);

  // Renders the static graphing axes, grids, and labels based on the selected timeframe.
  Draw_Oscilloscope_Grid(DISPLAY_TIME_MS);

  // Print static Phase Identifiers in the footer for the status UI.
  TFT_Print(10, 195, "PH-A", 0x07E0); // Green
  TFT_Print(10, 210, "PH-B", 0xF800); // Red
  TFT_Print(10, 225, "PH-C", 0x001F); // Blue

  /* SIGNAL GENERATION STARTUP:
   * Pre-compute the 3-Phase Sine Look-Up Tables, then instruct the DACs to begin
   * pulling that data from memory automatically via DMA, paced by TIM6.
   */
  Generate_3Phase_LUT();
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_sine_A, SINE_STEPS, DAC_ALIGN_12B_R);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)dac_sine_B, SINE_STEPS, DAC_ALIGN_12B_R);
  HAL_DAC_Start_DMA(&hdac2, DAC_CHANNEL_1, (uint32_t*)dac_sine_C, SINE_STEPS, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim6);

  /* DATA ACQUISITION STARTUP:
   * Instruct ADCs to begin pushing 30kHz sample data continuously into circular buffers
   * in the background, paced by TIM2 hardware metronome.
   */


  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, SAMPLES_PER_CH * 5*2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, SAMPLES_PER_CH *2);

  /* START THE (IA,IB,IC) CURRENT ADC3 */
//    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, SAMPLES_PER_CH * 3 * 2);
//    HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc3_buf, SAMPLES_PER_CH * 3 * 2);

    HAL_TIM_Base_Start(&htim2);

  /* DEBUG COMMUNICATION INITIALIZATION:
   * Configures high-speed serial port for diagnostic printing.
   */
  BspCOMInit.BaudRate   = 921600;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;

  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
      Error_Handler(); // Stop execution if UART hardware fails to initialize.
  }
  /* USER CODE END 2 */

  /* Initialize led */
//  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 921600;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* ============================================================================
   * MAIN APPLICATION LOOP
   * ============================================================================ */


  /* PRE-CALCULATED SQUARED THRESHOLDS:
   * We square the static limits here to compare apples-to-apples.
   */
   lowSq   = v_sag_thresh * v_sag_thresh;
  highSq  = v_swell_thresh * v_swell_thresh;
  recovSq = v_recov_thresh * v_recov_thresh;

  while (1)
  {
	  // Local flag used to force the TFT to bypass the 3-second timer and redraw immediately.
	        uint8_t do_refresh = 0;

	        Process_Grid_State();
	        Validate_Post_Capture();
	        Assemble_Anomaly_Log(&do_refresh);
	        Handle_Button_Logic(&do_refresh);
	        Render_Display_Engine(do_refresh);

//	        definitions in  "user code 0"
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

extern SPI_HandleTypeDef hspi1;

/* ============================================================================
 * 🖥️ TFT DISPLAY HARDWARE DRIVERS (SPI LEVEL)
 * ============================================================================ */

/**
  * @brief  Transmits a hardware configuration command to the TFT controller.
  * @note   The DC (Data/Command) pin is pulled LOW to indicate a command byte.
  * @param  cmd: The 8-bit hex command register address (e.g., 0x2A for column address).
  * @retval None
  */
void TFT_WriteCmd(uint8_t cmd) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // DC Low (Command Mode)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low (Select Display)
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);                // Transmit 1 byte via SPI
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High (Deselect Display)
}

/**
  * @brief  Transmits parameter data or pixel colors to the TFT controller.
  * @note   The DC (Data/Command) pin is pulled HIGH to indicate data bytes.
  * @param  data: The 8-bit hex parameter or partial color payload.
  * @retval None
  */
void TFT_WriteData(uint8_t data) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // DC High (Data Mode)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
}

/**
  * @brief  Executes the boot sequence for the ILI9341/ST7789 TFT LCD Controller.
  * @details
  * 1. Hardware Reset: Toggles the physical reset pin.
  * 2. Software Reset: Issues command 0x01.
  * 3. Sleep Out: Issues command 0x11 to wake the controller.
  * 4. Memory Access Control (MADCTL): Sets orientation to Landscape (0xE8).
  * 5. Color Format: Configures 16-bit RGB565 format (0x55).
  * 6. Display ON: Activates the panel output (0x29).
  * @retval None
  */
void TFT_Init(void) {
    /* Hardware Reset Pulse */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(100);

    TFT_WriteCmd(0x01); HAL_Delay(100); // Software Reset
    TFT_WriteCmd(0x11); HAL_Delay(100); // Exit Sleep Mode

    TFT_WriteCmd(0x36); // MADCTL: Orientation
    TFT_WriteData(0xE8); // Landscape mode

    TFT_WriteCmd(0x3A); // Interface Pixel Format
    TFT_WriteData(0x55); // 16-bit color (RGB565)

    TFT_WriteCmd(0x20); // Display Inversion Off
    TFT_WriteCmd(0x29); // Display Main ON
}

/**
  * @brief  Defines a specific rectangular area in the LCD's GRAM to write pixels into.
  * @note   Limits where subsequent `TFT_WriteData` calls will physically plot on screen.
  * @param  x: Starting X coordinate (Column).
  * @param  y: Starting Y coordinate (Row).
  * @param  w: Width of the window in pixels.
  * @param  h: Height of the window in pixels.
  * @retval None
  */
void TFT_SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint16_t x_end = x + w - 1;
    uint16_t y_end = y + h - 1;

    TFT_WriteCmd(0x2A); // Column Address Set
    TFT_WriteData(x >> 8); TFT_WriteData(x & 0xFF);
    TFT_WriteData(x_end >> 8); TFT_WriteData(x_end & 0xFF);

    TFT_WriteCmd(0x2B); // Row Address Set
    TFT_WriteData(y >> 8); TFT_WriteData(y & 0xFF);
    TFT_WriteData(y_end >> 8); TFT_WriteData(y_end & 0xFF);

    TFT_WriteCmd(0x2C); // Memory Write (Prepares LCD to receive pixel data)
}

/* ============================================================================
 * 🎨 TFT GRAPHICS & RENDERING PRIMITIVES
 * ============================================================================ */

/**
  * @brief  Renders a single ASCII character onto the display using the internal 5x8 font table.
  * @details Opens a 5x8 window in GRAM. Iterates through the font array; if a bit is 1,
  * it sends the user-defined color. If 0, it sends black (transparent background).
  * @param  x: Top-left X coordinate for the character.
  * @param  y: Top-left Y coordinate for the character.
  * @param  c: The specific ASCII character to draw.
  * @param  color: 16-bit RGB565 color for the text.
  * @retval None
  */
void TFT_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color) {
    TFT_SetWindow(x, y, 5, 8);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // Data Mode
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low

    for(int row = 0; row < 8; row++) {
        for(int col = 0; col < 5; col++) {
            uint8_t line = font5x8[c - 32][col]; // Font lookup (ASCII offset 32)
            // If bit is set, use text color. If not, use black (background).
            uint16_t p_color = (line & (1 << row)) ? color : 0x0000;

            uint8_t data[2] = {p_color >> 8, p_color & 0xFF}; // Split 16-bit color to 8-bit bytes
            HAL_SPI_Transmit(&hspi1, data, 2, 10);
        }
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
  * @brief  Speed-optimized vertical line drawing routine.
  * @details Rather than setting the window 1x1 for every pixel, this sets a 1xH window
  * and burst-transmits the entire line from an intermediate buffer.
  * This is critical for rendering high-amplitude waveforms without lag.
  * @param  x: Column position of the line.
  * @param  y: Starting row position.
  * @param  h: Height of the line in pixels.
  * @param  color: 16-bit RGB565 color.
  * @retval None
  */
void TFT_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    if (x >= 320 || h == 0 || h > 240) return; // Screen bounds safety check

    // 1. Tell the screen where we are drawing
    TFT_SetWindow(x, y, 1, h);

    // 2. Use a static buffer to avoid stack overflow (240 pixels max * 2 bytes)
    static uint8_t line_buffer[480];
    uint8_t high = color >> 8;
    uint8_t low = color & 0xFF;

    // 3. Fill the buffer once
    for (uint16_t i = 0; i < h; i++) {
        line_buffer[i * 2]     = high;
        line_buffer[i * 2 + 1] = low;
    }

    // 4. Send the entire line in ONE hardware call
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // Data Mode
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS Low

    // This is now 10x-50x faster than the loop version
    HAL_SPI_Transmit(&hspi1, line_buffer, h * 2, 10);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // CS High
}

/**
  * @brief  Connects two consecutive discrete waveform samples to create a continuous line.
  * @details Calculates the vertical delta between the previous sample and the current sample,
  * then delegates to the fast vertical line drawer to fill the gap.
  * @param  x: The current X-axis pixel column.
  * @param  old_y: The Y-pixel location of the previous sample.
  * @param  new_y: The Y-pixel location of the current sample.
  * @param  color: 16-bit RGB565 color.
  * @retval None
  */
void TFT_ConnectDots(uint16_t x, int old_y, int new_y, uint16_t color) {
    if (x == 0) return;

    int y_start = (old_y < new_y) ? old_y : new_y; // Find highest starting point (lowest integer)
    int h = abs(new_y - old_y) + 1;                // Calculate vertical distance

    // Safety: Restrict drawing strictly to the graphing grid boundaries (Y: 10 to 170)
    if (y_start < 10) y_start = 10;
    if (y_start + h > 170) h = 170 - y_start;

    TFT_DrawFastVLine(x + 40, y_start, h, color); // +40 offsets for the Y-axis label space
}

/**
  * @brief  Floods the entire 320x240 display with a single solid color.
  * @param  color: 16-bit RGB565 color.
  * @retval None
  */
void TFT_FillScreen(uint16_t color) {
    TFT_SetWindow(0, 0, 320, 240);
    uint8_t data[2] = {color >> 8, color & 0xFF};

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    for(uint32_t i = 0; i < (320 * 240); i++) {
        HAL_SPI_Transmit(&hspi1, data, 2, 10);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
  * @brief  Iterates through a character array to print a complete string.
  * @param  x: Starting X coordinate.
  * @param  y: Starting Y coordinate.
  * @param  str: Pointer to the null-terminated string.
  * @param  color: 16-bit RGB565 color.
  * @retval None
  */
void TFT_Print(uint16_t x, uint16_t y, char* str, uint16_t color) {
    while(*str) {
        TFT_DrawChar(x, y, *str++, color);
        x += 6; // Space character by 5px font width + 1px structural kerning
    }
}

/**
  * @brief  Draws a single pixel. Extremely slow; intended only for static UI detail work.
  * @param  x, y: Screen coordinates.
  * @param  color: 16-bit RGB565 color.
  * @retval None
  */
void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if(x >= 320 || y >= 240) return;
    TFT_SetWindow(x, y, 1, 1);
    uint8_t data[2] = {color >> 8, color & 0xFF};
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
  * @brief  Fills a user-defined rectangular bounding box with a single color.
  * @param  x, y: Top-left origin coordinate.
  * @param  w, h: Width and Height in pixels.
  * @param  color: 16-bit RGB565 color.
  * @retval None
  */
void TFT_DrawFilledRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    TFT_SetWindow(x, y, w, h);
    uint8_t data[2] = {color >> 8, color & 0xFF};
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    for(uint32_t i = 0; i < (w * h); i++) {
        HAL_SPI_Transmit(&hspi1, data, 2, 10);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}



/* ============================================================================
 * 🧮 SIGNAL MATH & GENERATION
 * ============================================================================ */

/**
  * @brief  Pre-computes three perfect sine wave periods separated by 120 degrees.
  * @details
  * - Target: 12-bit hardware DAC.
  * - Bias: 2048 (centers the wave in the 0-4095 voltage space).
  * - Amplitude: 1800 (keeps the peak below the 4095 ceiling to prevent op-amp hardware clipping).
  * - Shift: 120 degrees mathematically represented as (2 * PI / 3) radians.
  * @retval None
  */
void Generate_3Phase_LUT(void) {
    const float phase_shift = 2.0f * M_PI / 3.0f;
    // Amplitude limited to 1800 to prevent output clipping after op-amp stage gain of 1.14x
    const int amplitude = 1800; // Reduced from absolute max of 2047
    const int offset = 2048;

    for (int i = 0; i < SINE_STEPS; i++) {
        float angle = (i * 2.0f * M_PI) / SINE_STEPS;
        dac_sine_A[i] = offset + (int)(amplitude * sinf(angle));
        dac_sine_B[i] = offset + (int)(amplitude * sinf(angle - phase_shift));
        dac_sine_C[i] = offset + (int)(amplitude * sinf(angle - (2.0f * phase_shift)));
    }
}


/* ============================================================================
 * 🔄 HARDWARE INTERRUPTS & CALLBACKS
 * ============================================================================ */

/**
  * @brief  DMA Interrupt: Triggered automatically when ADC1 fills exactly half its buffer.
  * @note   Alerts the main rendering engine that new data is safe to evaluate/draw.
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        buffer_offset = 0;  // Read from the first half of the array while DMA fills the second.
        snapshot_ready = 1; // Assert flag for 3-second live TFT refresh.
    }
}

/**
  * @brief  DMA Interrupt: Triggered automatically when ADC1 completely fills its buffer.
  * @note   Alerts the main rendering engine that new data is safe to evaluate/draw.
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        buffer_offset = SAMPLES_PER_CH * 5; // Read from the second half of the array while DMA loops to the first.
        snapshot_ready = 1;
    }
}

/**
  * @brief  Simulates a catastrophic hardware failure on the grid (or recovers it).
  * @details Alters the source data of the Phase A DAC Look-Up Table on the fly.
  * @param  active:
  * - 1: Overwrites Phase A LUT entirely with DC_BIAS (2048), flattening the wave to 0V AC.
  * - 0: Triggers `Generate_3Phase_LUT()` to repopulate Phase A with healthy mathematics.
  * @retval None
  */
void Set_Signal_Failure(uint8_t active) {
    if (active) {
        // Only bring Phase A to center bias (0V AC), simulating a single-phase line break.
        for (int i = 0; i < SINE_STEPS; i++) {
            dac_sine_A[i] = DC_BIAS;
        }
        // Phase B and C remain healthy sines
    } else {
        // Restore all phases to full health parameters
        Generate_3Phase_LUT();
    }
}

/**
  * @brief  Hardware EXTI (External Interrupt) Button Callback.
  * @note   This function is intentionally kept empty. The button's sophisticated short-press
  * (log cycling) and long-press (fault injection) duration timing is explicitly handled
  * by a state machine embedded inside the `while(1)` loop. Putting logic here would
  * conflict with the polling routine.
  * @param  GPIO_Pin: The specific hardware pin triggering the interrupt (e.g., GPIO_PIN_13).
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // implement  intterupts for future migrate from polling
}



/**
  * @brief  Clears the screen and builds the static UI components (Axis, Grid, Labels).
  * @param  totalTime: Dynamic time window (ms) to calculate specific grid markers.
  * @retval None
  */
void Draw_Oscilloscope_Grid(uint32_t totalTime) {
    uint16_t gridColor = 0x7BEF;
    uint16_t axisColor = 0xFFFF;

    TFT_DrawFilledRect(0, 0, 320, 240, 0x0000);

    /* Y-Axis Labels */
    TFT_Print(2, 10, "+1.6", 0xFFFF);
    TFT_Print(2, 165, "-1.6", 0xFFFF);
    TFT_Print(10, 86, "0.0", axisColor);

    /* --- DYNAMIC X-AXIS LABELS --- */
    for (int i = 0; i <= 5; i++) {
        // Calculate the physical MS value for this specific UI tick mark based on zoom depth
        int ms_label = (totalTime * i) / 5;
        // Map to exact pixel coordinate on the grid
        int x_pos = 40 + (i * 260 / 5);

        sprintf(msg, "%d", ms_label);
        TFT_Print(x_pos - 5, 175, msg, gridColor);
    }
    TFT_Print(130, 188, "TIME (MS)", axisColor);

    /* Grid Lines Generation */
    for(int y = 10; y <= 170; y += 40) TFT_DrawFilledRect(40, y, 260, 1, gridColor); // Horizontals
    for(int x = 40; x <= 300; x += 52) TFT_DrawFilledRect(x, 10, 1, 160, gridColor); // Verticals

    /* Primary Axis Line Overlays */
    TFT_DrawFilledRect(38, 10, 2, 162, axisColor); // Y-Axis thick line
    TFT_DrawFilledRect(40, 90, 260, 2, axisColor); // X-Axis (0V) thick line
}


/**
  * @brief  Calculates the Mean Square (MS) of a specified block within a circular DMA buffer.
  * @details
  * - Normalizes the raw data against the 0V reference (DC_BIAS = 2048).
  * - Squares the values to evaluate absolute AC magnitude.
  * - Utilizes stride and offset parameters to accurately extract data from interleaved buffers
  * (e.g., Phase A is at index 0, 2, 4 while Phase B is at 1, 3, 5).
  * @param  buffer: Pointer to the raw DMA buffer.
  * @param  start_idx: The oldest point in time to begin evaluation.
  * @param  length: Total samples to evaluate (e.g., 600 samples = 20ms block).
  * @param  stride: Memory gap between valid data points (1 for dedicated buffer, 2 for interleaved).
  * @param  offset: Starting positional shift (0 for Phase A, 1 for Phase B).
  * @param  total_buf_size: The absolute memory size (used for safe modulo wrapping).
  * @retval Mean Square float value evaluating grid health.
  */
float Calculate_Buffer_MS(uint16_t* buffer, uint32_t start_idx, uint32_t length, uint8_t stride, uint8_t offset, uint32_t total_buf_size)
{
    double sum_sq = 0;
    for (uint32_t i = 0; i < length; i++) {
        // Correctly wrap the evaluation pointer around the circular memory architecture using modulo
        uint32_t idx = (start_idx + (i * stride) + offset) % total_buf_size;

        float val = (float)buffer[idx] - DC_BIAS; // Normalize to 0V AC
        sum_sq += (double)(val * val);            // Sum the squares
    }
    return (float)(sum_sq / length);              // Return the Mean Square
}










/**
  * @brief  Calculates the physical time (frequency period) of a captured waveform.
  * @details Finds two consecutive positive-going Zero Crossings (value transitions > 2048).
  * It determines the distance in the snapshot array, un-decimates it back to
  * raw hardware samples, and multiplies by the hardware acquisition time.
  * @param  snap: Pointer to the 260-element downsampled waveform array.
  * @param  decimation: The ratio applied to compress the raw buffer into the snapshot.
  * @retval Computed period of the wave in milliseconds.
  */
float Get_Period_MS(uint16_t* snap, uint32_t decimation) {
    int first = -1, second = -1;
    for (int i = 1; i < SAMPLE_COUNT; i++) {
        if (snap[i] > 2048 && snap[i-1] <= 2048) { // Positive Zero Crossing detected
            if (first == -1) {
                first = i;
                i += 5; // Jump forward safely to prevent detecting noise around the same crossing
            }
            else {
                second = i;
                break; // Found one full cycle
            }
        }
    }
    if (first != -1 && second != -1) {
        // Un-decimate array distance to find raw samples between crossings
        uint32_t delta_samples = (second - first) * decimation;
        // Multiply by hardware sample time (~33us) to get real-world period
        return (float)delta_samples * REAL_SAMPLE_US / 1000.0f;
    }
    return 0.0f; // Failure return if a full cycle is not visible in the current time-zoom
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
