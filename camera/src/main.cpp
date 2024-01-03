#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include <SPI.h>
#include <SD.h>
#include "FS.h"

#include <U8x8lib.h>

#include "reader.h"
#include "settings.h"

#include <mixed-mobilenet_inferencing.h>
// #include "grayscale_camera.h"

#include <esp_camera.h>

#include "pins.h"

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 96
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 96
#define EI_CAMERA_FRAME_BYTE_SIZE 1

bool ei_grayscale_init(void);
void ei_grayscale_deinit(void);
bool ei_grayscale_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_grayscale_get_data(size_t offset, size_t length, float *out_ptr);
static int ei_depth_get_data(size_t offset, size_t length, float *out_ptr);
void run_depth_ei(void);
void run_grayscale_ei(void);

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; // points to the output of the capture

const int buttonPin = D1;
int buttonState = 0;

uint16_t imageCount = 0;
char filename[32];
char logString[100];
bool isPressed = false;

// static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE); // OLEDs without Reset of the Display


char *grayscale_label;
char *depth_label;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_96X96,       // QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, // 0-63 lower number means higher quality
    .fb_count = 1,      // if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

void writeFile(fs::FS &fs, const char *path, uint8_t *data, size_t len)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.write(data, len) == len)
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();

  imageCount++;
  sprintf(filename, "/IMG_%04d.bin", imageCount);
}

void grayscale_setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // comment out the below line to start inference immediately after upload
  while (!Serial)
    ;
  Serial.println("Edge Impulse Inferencing Demo");
  if (ei_grayscale_init() == false)
  {
    ei_printf("Failed to initialize Camera!\r\n");
  }
  else
  {
    ei_printf("Camera initialized\r\n");
  }

  ei_printf("\nStarting continious inference in 2 seconds...\n");
  ei_sleep(2000);
}

bool ei_grayscale_init(void)
{

  if (is_initialised)
    return true;
  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);      // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, 0); // lower the saturation
  }

  is_initialised = true;
  return true;
}

void ei_grayscale_deinit(void)
{

  // deinitialize the camera
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK)
  {
    ei_printf("Camera deinit failed\n");
    return;
  }

  is_initialised = false;
  return;
}

bool ei_grayscale_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
  bool do_resize = false;

  if (!is_initialised)
  {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb)
  {
    ei_printf("Camera capture failed\n");
    return false;
  }

  for (int i = 0; i < fb->len; i++)
  {
    snapshot_buf[i] = fb->buf[i];
  }

  esp_camera_fb_return(fb);

  return true;
}

static int ei_grayscale_get_data(size_t offset, size_t length, float *out_ptr)
{
  size_t pixel_ix = offset;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0)
  {
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix] << 8) + snapshot_buf[pixel_ix];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 1;
    pixels_left--;
  }
  // and done!
  return 0;
}


void refreshIndex(void)
{
  while (true)
  {
    sprintf(filename, "/IMG_%04d.bin", imageCount);
    if (SD.exists(filename))
    {
      imageCount++;
    }
    else
    {
      break;
    }
  }
}

void freshPrint(const char *s)
{
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.println(s);
}

void log(char *string)
{
  Serial.println(string);
  freshPrint(string);
}

void setup()
{

  // grayscale_setup();

  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  Wire.begin();

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);

  // Begin serial with A010's default baudrate.
  CameraSerial.begin(115200, SERIAL_8N1, RX, TX);
  while (!CameraSerial)
  {
    delay(100);
  }
  delay(2000);

  // Then set to a higher baudrate. Otherwise the A010 module will became sluggish.
  CameraSerial.print(ATC_BAUDRATE);
  delay(1000);
  CameraSerial.updateBaudRate(230400);
  delay(1000);

  log("Setting up \n camera...");

  CameraSerial.print(ATC_UNIT);
  delay(1000);
  CameraSerial.print(ATC_FPS);
  delay(1000);
  CameraSerial.print(ATC_BINNMODE);
  delay(1000);
  CameraSerial.print(ATC_DISP);

  log("Initializing \n SD card...");
  pinMode(D2, OUTPUT);
  if (!SD.begin(D2))
  {
    log("initialization \n failed!");
    return;
  }
  log("initialization \n done.");
  refreshIndex();
}

void loop()
{
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW)
  {
    if (!isPressed && isNewFrameReady)
    {

      isPressed = true;
      writeFile(SD, filename, dataBuffer, fileSize);

      u8x8.clear();
      u8x8.printf("%s \nwritten.\n", filename);
      u8x8.printf("File size:\n %6d\n", fileSize);

      Serial.printf("File written. \n File size: \n %8d\n", fileSize);
      isNewFrameReady = false;
      delay(200);
    }
  }
  else
  {
    isPressed = false;
  }

  while (CameraSerial.available())
  {
    rawData.push_back((byte)CameraSerial.read());
  }

  parseFrame();
#ifdef ENABLE_INFERENCING
  run_depth_ei();
  run_grayscale_ei();
#endif
}


void run_grayscale_ei()
{

  // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
  if (ei_sleep(5) != EI_IMPULSE_OK)
  {
    return;
  }

  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

  // check if allocation was successful
  if (snapshot_buf == nullptr)
  {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_grayscale_get_data;

  if (ei_grayscale_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false)
  {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = {0};

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK)
  {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // print the predictions
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

  int _max = 0;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    if (result.classification[ix].value > _max)
    {
      _max = result.classification[ix].value;
      grayscale_label = (char *)result.classification[ix].label;
    }
    ei_printf("    %s: %.5f\n", result.classification[ix].label,
              result.classification[ix].value);
  }
  free(snapshot_buf);
}

void run_depth_ei()
{

  if (isNewFrameReady)
  {
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_depth_get_data;

    // Run the classifier
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK)
    {
      ei_printf("ERR: Failed to run classifier (%d)\n", err);
      return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

    float highest = 0;
    const char *highestLabel;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
      ei_printf("    %s: %.5f\n", result.classification[ix].label,
                result.classification[ix].value);
      if (result.classification[ix].value > highest)
      {
        highest = result.classification[ix].value;
        highestLabel = result.classification[ix].label;
      }
    }

    u8x8.clear();
    u8x8.println(highestLabel);
    u8x8.printf("\n%3.6f\n", highest);
    isNewFrameReady = false;
  }
}

static int ei_depth_get_data(size_t offset, size_t length, float *out_ptr)
{
  size_t counter = 0;

  size_t pixel_ix = offset + 2;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0)
  {
    uint8_t pixel = dataBuffer[pixel_ix];
    out_ptr[out_ptr_ix] = (pixel << 16) + (pixel << 8) + pixel;
    // go to the next pixel
    out_ptr_ix++;
    pixel_ix++;
    pixels_left--;
    counter++;

    if (counter >= 96)
    {
      pixel_ix += 4;
      counter = 0;
    }
  }
  // and done!
  return 0;
}


