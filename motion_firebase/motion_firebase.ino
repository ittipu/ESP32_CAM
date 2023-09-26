#include "WiFi.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <SPIFFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include <addons/TokenHelper.h>
#include "time.h"


//Replace with your network credentials
//const char* ssid = "iot_hack_back";
//const char* password = "!01h@(k_b@(k_007007@br@(n3t";
const char* ssid = "IoT_5g";
const char* password = "tipu1234";
const char* ntpServer = "pool.ntp.org";

// Insert Firebase project API Key
#define API_KEY "AIzaSyDYgLJOOJF6SNdtc4ItcHBS9Zftlyd6w3Y"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "testuser@email.com"
#define USER_PASSWORD "testuser"


// Insert Firebase storage bucket ID e.g bucket-name.appspot.com
#define STORAGE_BUCKET_ID "bracnet-demo.appspot.com"

#define DATABASE_URL "https://bracnet-demo-default-rtdb.asia-southeast1.firebasedatabase.app/"
String parentPath = "motion_face_db/";

// Photo File Name to save in SPIFFS
String FILE_PHOTO = "";



RTC_DATA_ATTR int bootCount = 0;


// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


const int PIN_TO_SENSOR = 14;   // the pin that OUTPUT pin of sensor is connected
int pinStateCurrent   = LOW; // current state of pin
int pinStatePrevious  = LOW; // previous state of pin
boolean takeNewPhoto = true;
unsigned long epochTime = 0;

//Define Firebase Data objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig configF;

bool taskCompleted = false;

// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSaveSpiffs( void ) {
  bool formatted = SPIFFS.format();
  if (formatted) {
    Serial.println("\n\nSuccess formatting");
  } else {
    Serial.println("\n\nError formatting");
  }
  FILE_PHOTO = "/motionFace/";
  camera_fb_t * fb = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly
  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }

    epochTime = getTime();
    Serial.print("Epoch Time: ");
    Serial.println(epochTime);
    String filename = String(epochTime) + ".jpg";
    Serial.print("File Name: ");
    FILE_PHOTO = FILE_PHOTO + filename;
    Serial.println(String(FILE_PHOTO));
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    // Insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    // Close the file
    file.close();
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);

  } while ( !ok );
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(300);
  }
  Serial.println(WiFi.localIP());
}

void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
}

void initCamera() {
  // OV2640 camera module
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);            // initialize serial
  pinMode(PIN_TO_SENSOR, INPUT); // set arduino pin to input mode to read value from OUTPUT pin of sensor
  initWiFi();
  initSPIFFS();
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  initCamera();
  configTime(0, 0, ntpServer);
  Serial.print("Current time: ");
  Serial.println(getTime());

  //Firebase
  // Assign the api key
  configF.api_key = API_KEY;
  //Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  configF.database_url = DATABASE_URL;
  //Assign the callback function for the long running token generation task
  configF.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&configF, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  pinStatePrevious = pinStateCurrent; // store old state
  pinStateCurrent = digitalRead(PIN_TO_SENSOR);   // read new state

  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {   // pin state change: LOW -> HIGH
    Serial.println("Motion detected!");
    takePic();
    FILE_PHOTO = "";

    // TODO: turn on alarm, light or activate a device ... here
  }
  else if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {  // pin state change: HIGH -> LOW
    Serial.println("Motion stopped!");
    // TODO: turn off alarm, light or deactivate a device ... here
  }
}

void takePic() {
  if (takeNewPhoto) {
    capturePhotoSaveSpiffs();
    delay(2000);
    if (Firebase.ready() && !taskCompleted) {
      taskCompleted = true;
      Serial.print("Uploading picture... ");

      //MIME type should be valid to avoid the download problem.
      //The file systems for flash and SD/SDMMC can be changed in FirebaseFS.h.
      if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, FILE_PHOTO /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, FILE_PHOTO /* path of remote file stored in the bucket */, "image/jpeg" /* mime type */)) {
        Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
        FirebaseJson json;
        String path = parentPath + String(epochTime) + "/";
        json.set("image_url", fbdo.downloadURL().c_str());
        json.set("time", String(epochTime));
        json.set("visibility", "true");
        Serial.printf("Set json...%s\n\n", Firebase.RTDB.setJSON(&fbdo, path, &json) ? "ok" : fbdo.errorReason().c_str());
      }
      else {
        Serial.println(fbdo.errorReason());
      }
    }
    taskCompleted = false;
  }
  return;
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}
