#include "BluetoothSerial.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

BluetoothSerial SerialBT;

// WiFi 設定
const char* ssid = "Gptt_center_1F";
const char* password = "gptt1234567";

// 網頁伺服器
WebServer server(80);

// 靜態 IP 設定
IPAddress local_IP(10, 0, 0, 80);
IPAddress gateway(10, 0, 0, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(168, 95, 192, 1);
IPAddress secondaryDNS(168, 95, 1, 1);

// 原始系統腳位定義
const int HX710B_dout = 19;
const int HX710B_sck = 18;
const int Reloading = 25;
const int SHOOT_OUTPUT = 22;
const int MAGAZINE_OUTPUT = 22;  // 射擊跟彈匣共用燈號
const int HALL_LED_PIN = 21;     // 霍爾感測器燈號控制
const int MAGNET_PIN = 26;       // 藍牙連接馬達

// 霍爾感測器腳位定義
const int HALL_LEFT_PIN = 36;    
const int HALL_RIGHT_PIN = 32;   
const int OUTPUT_PIN = 33;       // 藍牙連線指示燈腳位

// 藍牙連線指示燈參數設定
const int LEDC_CHANNEL = 0;      // PWM通道
const int LEDC_FREQ = 5000;      // PWM頻率
const int LEDC_RESOLUTION = 8;   // PWM解析度（0-255）

// 呼吸燈參數設定
const int BREATHE_PERIOD = 1200;     // 總呼吸週期（1.2秒）
const int BRIGHT_TIME = 450;         // 亮起時間（450ms）
const int BRIGHT_HOLD_TIME = 100;    // 亮起持續時間（100ms）
const int DIM_TIME = 450;            // 暗淡時間（450ms）
const int DIM_HOLD_TIME = 200;       // 暗淡持續時間（200ms）

// 優化：HX710B 讀取間隔從 1ms 改為 10ms
const long interval = 10; // 優化：從 1ms 改為 10ms
int RISE_THRESHOLD = 1200;  // 上升門檻（可遠端修改）
const unsigned long SHOOT_INTERVAL = 50; // 最短射擊間隔(ms)

// 霍爾感測器參數設定
const int SAMPLE_COUNT = 5;      // 取樣次數從3提高到5，增強穩定性
const unsigned long READ_INTERVAL = 10; // 檢測間隔從15ms改為10ms，提高響應速度
const int GRIP_THRESHOLD = 150;   // 霍爾感測器觸發閾值從150降到120，提高靈敏度
const unsigned long DEBOUNCE_TIME = 30; // 防抖動時間從50ms減少到30ms，加快反應

// 調試輸出控制參數
const bool ENABLE_HALL_DEBUG = false;    // 設為false可關閉霍爾感測器調試輸出
const unsigned long DEBUG_PRINT_INTERVAL = 500; // 調試輸出間隔（500ms）

// 射擊燈號參數設定
const unsigned long SHOOT_LED_DURATION = 350; // 射擊燈號持續時間（350ms）

// 心跳包參數設定
const unsigned long HEARTBEAT_INTERVAL = 2000; // 每2秒發送一次心跳包（軟體timeout為10秒，提供充足容錯率）
unsigned long lastHeartbeatTime = 0;           // 上次心跳包發送時間

// 藍牙連接狀態監控
bool bluetoothWasConnected = false;             // 上一次的藍牙連接狀態
const unsigned long STATUS_SEND_DELAY = 200;   // 連接後延遲發送狀態的時間(ms) - 從500ms改為200ms加快響應
unsigned long connectionEstablishedTime = 0;   // 連接建立時間
bool initialStatusSent = false;                // 是否已發送初始狀態

// 電磁鐵防燒保護參數設定
const unsigned long MAGNET_PROTECTION_TIME = 60000; // 電磁鐵防燒保護時間（60秒）
unsigned long magnetOnTime = 0;                     // 電磁鐵開啟時間記錄

// 彈匣狀態變化防誤觸參數設定
const unsigned long MAGAZINE_CHANGE_IGNORE_TIME = 200; // 彈匣狀態變化後忽略射擊判定的時間(ms)
unsigned long lastMagazineChangeTime = 0;              // 上次彈匣狀態變化時間

// 原始系統變數
unsigned long lastShootTime = 0;
unsigned long previousMillis = 0;
unsigned long shootPulseStartTime = 0;
bool shootPulseActive = false;
bool magnetActive = false;

// 藍牙連線指示燈變數
unsigned long breatheStartTime = 0;
bool bluetoothConnected = false;
bool lastBluetoothState = false;

// 射擊燈號變數
unsigned long shootLedStartTime = 0;
bool shootLedActive = false;

// 霍爾感測器變數
int leftBaselineValue = 1150;
int rightBaselineValue = 1150;
unsigned long lastReadTime = 0;
unsigned long lastLeftStateChangeTime = 0;
unsigned long lastRightStateChangeTime = 0;
bool currentLeftGripState = false;
bool currentRightGripState = false;
bool pendingLeftGripState = false;
bool pendingRightGripState = false;

// 基準值修改模式相關變數
bool calibrationMode = false;          // 是否處於基準值修改模式
int calibrationStep = 0;               // 修改步驟：0=未開始，1=等待左邊基準值，2=等待右邊基準值，3=等待上升門檻值
String inputBuffer = "";               // 藍牙輸入緩衝區

// 調試輸出控制變數
unsigned long lastDebugPrintTime = 0;

// WiFi 連線狀態變數
bool wifiEnabled = false;

//OTA 介面
const char* updatePage =
R"rawliteral(
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BT_GLOCK45韌體更新中心</title>
    <link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css" rel="stylesheet">
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a1a 0%, #0f0f0f 50%, #1a1a1a 100%);
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        
        .container {
            max-width: 500px;
            width: 100%;
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(15px);
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 40px rgba(220, 38, 38, 0.3);
            border: 2px solid rgba(220, 38, 38, 0.5);
            animation: slideUp 0.8s ease-out;
        }
        
        @keyframes slideUp {
            from {
                opacity: 0;
                transform: translateY(30px);
            }
            to {
                opacity: 1;
                transform: translateY(0);
            }
        }
        
        .header {
            text-align: center;
            margin-bottom: 40px;
        }
        
        .header i {
            font-size: 3.5rem;
            color: #dc2626;
            margin-bottom: 15px;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.05); }
        }
        
        .header h1 {
            color: #1a1a1a;
            font-size: 2.2rem;
            font-weight: 600;
            margin-bottom: 10px;
        }
        
        .header p {
            color: #4a5568;
            font-size: 1.1rem;
            font-weight: 500;
        }
        
        .upload-section {
            background: #f8f9fa;
            border-radius: 15px;
            padding: 40px;
            margin-bottom: 30px;
            border: 2px dashed #dc2626;
            transition: all 0.3s ease;
            text-align: center;
        }
        
        .upload-section:hover {
            border-color: #991b1b;
            background: #f1f5f9;
            transform: translateY(-2px);
        }
        
        .upload-icon {
            font-size: 4rem;
            color: #dc2626;
            margin-bottom: 25px;
        }
        
        .file-input-wrapper {
            position: relative;
            display: inline-block;
            margin-bottom: 25px;
        }
        
        .file-input {
            position: absolute;
            opacity: 0;
            width: 100%;
            height: 100%;
            cursor: pointer;
        }
        
        .file-input-btn {
            display: inline-block;
            padding: 15px 35px;
            background: linear-gradient(135deg, #dc2626 0%, #991b1b 100%);
            color: white;
            border-radius: 30px;
            cursor: pointer;
            transition: all 0.3s ease;
            font-weight: 600;
            font-size: 1.1rem;
            position: relative;
            overflow: hidden;
        }
        
        .file-input-btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 10px 25px rgba(220, 38, 38, 0.4);
            background: linear-gradient(135deg, #b91c1c 0%, #7f1d1d 100%);
        }
        
        .file-input-btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.2), transparent);
            transition: left 0.5s;
        }
        
        .file-input-btn:hover::before {
            left: 100%;
        }
        
        .file-name {
            margin-top: 20px;
            padding: 15px;
            background: rgba(220, 38, 38, 0.1);
            border-radius: 10px;
            color: #1a1a1a;
            font-weight: 500;
            font-size: 1rem;
            display: none;
            border: 1px solid rgba(220, 38, 38, 0.2);
        }
        
        .upload-btn {
            width: 100%;
            padding: 18px;
            background: linear-gradient(135deg, #dc2626 0%, #991b1b 100%);
            color: white;
            border: none;
            border-radius: 15px;
            font-size: 1.1rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            margin-top: 25px;
            position: relative;
            overflow: hidden;
        }
        
        .upload-btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 30px rgba(220, 38, 38, 0.4);
            background: linear-gradient(135deg, #b91c1c 0%, #7f1d1d 100%);
        }
        
        .upload-btn:disabled {
            background: #6b7280;
            cursor: not-allowed;
            transform: none;
            box-shadow: none;
        }
        
        .upload-btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.2), transparent);
            transition: left 0.5s;
        }
        
        .upload-btn:hover::before {
            left: 100%;
        }
        
        .progress-section {
            margin-top: 30px;
            display: none;
        }
        
        .progress-bar-container {
            background: #e5e7eb;
            border-radius: 25px;
            height: 12px;
            overflow: hidden;
            margin-bottom: 20px;
            position: relative;
        }
        
        .progress-bar {
            height: 100%;
            background: linear-gradient(135deg, #dc2626 0%, #991b1b 100%);
            border-radius: 25px;
            width: 0%;
            transition: width 0.3s ease;
            position: relative;
            overflow: hidden;
        }
        
        .progress-bar::after {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            bottom: 0;
            right: 0;
            background: linear-gradient(45deg, 
                rgba(255,255,255,0.2) 25%, 
                transparent 25%, 
                transparent 50%, 
                rgba(255,255,255,0.2) 50%, 
                rgba(255,255,255,0.2) 75%, 
                transparent 75%);
            background-size: 20px 20px;
            animation: move 1s linear infinite;
        }
        
        @keyframes move {
            0% { background-position: 0 0; }
            100% { background-position: 20px 20px; }
        }
        
        .progress-text {
            text-align: center;
            font-size: 1.3rem;
            font-weight: 600;
            color: #1a1a1a;
        }
        
        .status-message {
            padding: 18px;
            border-radius: 12px;
            margin-top: 25px;
            text-align: center;
            font-weight: 600;
            font-size: 1.1rem;
            display: none;
        }
        
        .status-success {
            background: #d1fae5;
            color: #065f46;
            border: 2px solid #6ee7b7;
        }
        
        .status-error {
            background: #fee2e2;
            color: #991b1b;
            border: 2px solid #fca5a5;
        }
        
        .warning-text {
            background: rgba(255, 193, 7, 0.1);
            color: #b45309;
            padding: 15px;
            border-radius: 10px;
            margin-top: 25px;
            text-align: center;
            font-weight: 500;
            border: 1px solid rgba(255, 193, 7, 0.3);
        }
        
        @media (max-width: 480px) {
            .container {
                padding: 25px;
                margin: 10px;
            }
            
            .header h1 {
                font-size: 1.8rem;
            }
            
            .upload-section {
                padding: 25px;
            }
            
            .upload-icon {
                font-size: 3rem;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <i class="fas fa-microchip"></i>
            <h1>BT_GLOCK45韌體更新中心</h1>
            <p>選擇韌體檔案並開始更新</p>
        </div>
        
        <div class="upload-section">
            <div class="upload-icon">
                <i class="fas fa-cloud-upload-alt"></i>
            </div>
            
            <form method="POST" action="#" enctype="multipart/form-data" id="upload_form">
                <div class="file-input-wrapper">
                    <input type="file" name="update" class="file-input" accept=".bin" id="fileInput">
                    <label for="fileInput" class="file-input-btn">
                        <i class="fas fa-folder-open"></i> 選擇韌體檔案
                    </label>
                </div>
                <div class="file-name" id="fileName"></div>
                
                <button type="submit" class="upload-btn" id="uploadBtn" disabled>
                    <i class="fas fa-upload"></i> 開始更新韌體
                </button>
            </form>
        </div>
        
        <div class="progress-section" id="progressSection">
            <div class="progress-bar-container">
                <div class="progress-bar" id="progressBar"></div>
            </div>
            <div class="progress-text" id="progressText">準備中...</div>
        </div>
        
        <div class="status-message" id="statusMessage"></div>
        
        <div class="warning-text">
            <i class="fas fa-exclamation-triangle"></i>
            <strong>注意：</strong>更新期間請勿關閉電源或按任何按鈕！
        </div>
    </div>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/jquery/3.6.0/jquery.min.js"></script>
    <script>
        $(document).ready(function() {
            $('#fileInput').change(function() {
                const fileName = $(this)[0].files[0]?.name;
                if (fileName) {
                    $('#fileName').text('已選擇: ' + fileName).show();
                    $('#uploadBtn').prop('disabled', false);
                } else {
                    $('#fileName').hide();
                    $('#uploadBtn').prop('disabled', true);
                }
            });
            
            $('#upload_form').submit(function(e) {
                e.preventDefault();
                
                const formData = new FormData(this);
                const progressSection = $('#progressSection');
                const progressBar = $('#progressBar');
                const progressText = $('#progressText');
                const statusMessage = $('#statusMessage');
                const uploadBtn = $('#uploadBtn');
                
                progressSection.show();
                uploadBtn.prop('disabled', true).html('<i class="fas fa-spinner fa-spin"></i> 更新中...');
                statusMessage.hide();
                
                $.ajax({
                    url: '/update',
                    type: 'POST',
                    data: formData,
                    contentType: false,
                    processData: false,
                    xhr: function() {
                        const xhr = new window.XMLHttpRequest();
                        xhr.upload.addEventListener('progress', function(evt) {
                            if (evt.lengthComputable) {
                                const percentComplete = Math.round((evt.loaded / evt.total) * 100);
                                progressBar.css('width', percentComplete + '%');
                                progressText.text('上傳進度: ' + percentComplete + '%');
                                
                                if (percentComplete === 100) {
                                    progressText.text('處理中，請稍候...');
                                }
                            }
                        }, false);
                        return xhr;
                    },
                    success: function(response) {
                        progressBar.css('width', '100%');
                        progressText.text('更新完成！');
                        statusMessage.removeClass('status-error').addClass('status-success')
                            .html('<i class="fas fa-check-circle"></i> 韌體更新成功！設備將自動重啟。').show();
                        
                        setTimeout(() => {
                            window.location.reload();
                        }, 3000);
                    },
                    error: function(xhr, status, error) {
                        statusMessage.removeClass('status-success').addClass('status-error')
                            .html('<i class="fas fa-exclamation-triangle"></i> 更新失敗，請檢查檔案格式並重試。').show();
                        
                        uploadBtn.prop('disabled', false).html('<i class="fas fa-upload"></i> 重新嘗試');
                        progressSection.hide();
                    }
                });
            });
        });
    </script>
</body>
</html>
)rawliteral";

// 檢查藍牙連線狀態
bool checkBluetoothConnection() {
    return SerialBT.hasClient();
}

// 檢查電磁鐵防燒保護並執行保護動作
void checkMagnetProtection() {
    unsigned long currentTime = millis();
    
    // 如果電磁鐵處於開啟狀態，檢查是否需要防燒保護
    if (magnetActive) {
        // 檢查是否超過防燒保護時間（60秒）
        if (currentTime - magnetOnTime >= MAGNET_PROTECTION_TIME) {
            // 強制關閉電磁鐵
            magnetActive = false;
            digitalWrite(MAGNET_PIN, LOW);
            
            // 發送防燒保護啟動通知
            if (bluetoothConnected) {
                SerialBT.println("MAGNET_PROTECTION_ACTIVATED");
            }
            
            // 注意：不設定保護標誌，允許立即重新開啟
        }
    }
}

// 安全開啟電磁鐵（包含防燒保護機制）
void safeTurnOnMagnet() {
    if (bluetoothConnected) {
        magnetActive = true;
        magnetOnTime = millis(); // 記錄開啟時間
        digitalWrite(MAGNET_PIN, HIGH);
        SerialBT.println("MAGNET_ON_OK");
    } else {
        SerialBT.println("MAGNET_ERROR_NO_CONNECTION");
    }
}

// 安全關閉電磁鐵
void safeTurnOffMagnet() {
    magnetActive = false;
    digitalWrite(MAGNET_PIN, LOW);
    SerialBT.println("MAGNET_OFF_OK");
}

// 獲取設備狀態編碼字串
String getDeviceStatusCode() {
    String statusCode = "s";
    
    // 第1位：彈匣狀態 (0=未插入, 1=已插入)
    int magazineState = digitalRead(Reloading);
    statusCode += (magazineState == LOW) ? "1" : "0";
    
    // 第2位：電磁鐵狀態 (0=關閉, 1=開啟)
    statusCode += magnetActive ? "1" : "0";
    
    // 第3位：左邊霍爾感測器狀態 (0=未握持, 1=握持)
    statusCode += currentLeftGripState ? "1" : "0";
    
    // 第4位：右邊霍爾感測器狀態 (0=未握持, 1=握持)
    statusCode += currentRightGripState ? "1" : "0";
    
    return statusCode;
}

// 發送設備狀態
void sendDeviceStatus() {
    if (bluetoothConnected) {
        String statusCode = getDeviceStatusCode();
        SerialBT.println(statusCode);
    }
}

// 計算呼吸燈亮度值
int calculateBreatheBrightness(unsigned long currentTime) {
    unsigned long elapsedTime = currentTime - breatheStartTime;
    unsigned long cycleTime = elapsedTime % BREATHE_PERIOD;
    
    if (cycleTime < BRIGHT_TIME) {
        return map(cycleTime, 0, BRIGHT_TIME, 0, 255);
    } else if (cycleTime < BRIGHT_TIME + BRIGHT_HOLD_TIME) {
        return 255;
    } else if (cycleTime < BRIGHT_TIME + BRIGHT_HOLD_TIME + DIM_TIME) {
        unsigned long dimElapsed = cycleTime - BRIGHT_TIME - BRIGHT_HOLD_TIME;
        return map(dimElapsed, 0, DIM_TIME, 255, 0);
    } else {
        return 0;
    }
}

// 更新藍牙連線指示燈
void updateBluetoothLED() {
    unsigned long currentTime = millis();
    
    if (bluetoothConnected) {
        ledcWrite(LEDC_CHANNEL, 255);
    } else {
        int brightness = calculateBreatheBrightness(currentTime);
        ledcWrite(LEDC_CHANNEL, brightness);
    }
}

// 優化：提升霍爾感測器讀取穩定性，減少tracker晃動影響
int readFastHallSensor(int pin) {
    long sum = 0;
    int validReadings = 0;
    
    // 進行多次讀取並過濾異常值
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        int reading = analogRead(pin);
        
        // 基本範圍檢查，過濾明顯異常的讀值
        if (reading >= 0 && reading <= 4095) {
            sum += reading;
            validReadings++;
        }
        delayMicroseconds(100); // 極短延遲穩定讀取
    }
    
    // 如果有效讀取數不足，返回上次的基準值
    if (validReadings < (SAMPLE_COUNT / 2)) {
        return (pin == HALL_LEFT_PIN) ? leftBaselineValue : rightBaselineValue;
    }
    
    return sum / validReadings;
}

// 校準兩個霍爾感測器的基準值
void calibrateBaselines() {
    // 直接使用變數中的基準值，不重新賦值
}

// 手動發送霍爾感測器原始數據到Unity
void sendCurrentHallData() {
    if (bluetoothConnected) {
        int currentLeftRaw = readFastHallSensor(HALL_LEFT_PIN);
        int currentRightRaw = readFastHallSensor(HALL_RIGHT_PIN);
        
        // 優化：使用 printf 格式取代字串串接
        SerialBT.printf("HALL_DATA:LEFT=%d,RIGHT=%d\n", currentLeftRaw, currentRightRaw);
    }
}

// 進入基準值修改模式
void enterCalibrationMode() {
    calibrationMode = true;
    calibrationStep = 1;
    inputBuffer = "";
    
    SerialBT.println("CALIB_MODE_START");
    SerialBT.println("Current Settings:");
    // 優化：使用 printf 格式
    SerialBT.printf("  Left Hall Baseline: %d\n", leftBaselineValue);
    SerialBT.printf("  Right Hall Baseline: %d\n", rightBaselineValue);
    SerialBT.printf("  Rise Threshold: %d\n", RISE_THRESHOLD);
    SerialBT.println("Starting calibration...");
    SerialBT.println("Enter new left hall sensor baseline value:");
}

// 處理基準值修改輸入
void processCalibrationInput(String input) {
    if (!calibrationMode) return;
    
    int value = input.toInt();
    
    if (calibrationStep == 1) {
        if (value > 0 && value <= 4095) {
            leftBaselineValue = value;
            calibrationStep = 2;
            
            // 優化：使用 printf 格式
            SerialBT.printf("Left baseline set to: %d\n", leftBaselineValue);
            SerialBT.println("Enter new right hall sensor baseline value:");
        } else {
            SerialBT.println("Error: Baseline must be 1-4095, re-enter left baseline:");
        }
    } else if (calibrationStep == 2) {
        if (value > 0 && value <= 4095) {
            rightBaselineValue = value;
            calibrationStep = 3;
            
            // 優化：使用 printf 格式
            SerialBT.printf("Right baseline set to: %d\n", rightBaselineValue);
            SerialBT.println("Enter HX710B rise threshold (recommended 100-5000):");
        } else {
            SerialBT.println("Error: Baseline must be 1-4095, re-enter right baseline:");
        }
    } else if (calibrationStep == 3) {
        if (value > 0 && value <= 10000) {
            RISE_THRESHOLD = value;
            
            // 優化：使用 printf 格式
            SerialBT.printf("Rise threshold set to: %d\n", RISE_THRESHOLD);
            SerialBT.println("CALIB_MODE_COMPLETE");
            SerialBT.println("All parameters updated successfully!");
            SerialBT.println("New settings:");
            SerialBT.printf("  Left Hall Baseline: %d\n", leftBaselineValue);
            SerialBT.printf("  Right Hall Baseline: %d\n", rightBaselineValue);
            SerialBT.printf("  Rise Threshold: %d\n", RISE_THRESHOLD);
            
            calibrationMode = false;
            calibrationStep = 0;
            inputBuffer = "";
        } else {
            SerialBT.println("Error: Rise threshold must be 1-10000, re-enter:");
        }
    }
}

// OTA 模式啟動函數
void startOTAMode() {
    if (!wifiEnabled) {
        SerialBT.println("Starting WiFi for OTA update...");
        
        // 設定靜態 IP
        if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
            SerialBT.println("WiFi config failed");
        }
        
        WiFi.begin(ssid, password);
        
        // 等待連線，最多 10 秒
        int timeout = 0;
        while (WiFi.status() != WL_CONNECTED && timeout < 20) {
            delay(500);
            timeout++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            wifiEnabled = true;
            
            // 設定 mDNS
            if (!MDNS.begin("esp32")) {
                SerialBT.println("mDNS setup failed");
            }
            
            // 設定 ArduinoOTA
            ArduinoOTA.begin();
            
            // 設定網頁伺服器（使用您原本的介面）
            server.on("/", HTTP_GET, []() {
                server.sendHeader("Connection", "close");
                server.send(200, "text/html", updatePage);
            });
            
            server.on("/update", HTTP_POST, []() {
                server.sendHeader("Connection", "close");
                server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
                delay(1000);
                ESP.restart();
            }, []() {
                HTTPUpload& upload = server.upload();
                if (upload.status == UPLOAD_FILE_START) {
                    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
                } else if (upload.status == UPLOAD_FILE_WRITE) {
                    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
                } else if (upload.status == UPLOAD_FILE_END) {
                    if (Update.end(true)) {
                        // 優化：移除 Serial.printf - 省電
                    } else {
                        Update.printError(Serial);
                    }
                }
            });
            
            server.begin();
            
            SerialBT.printf("OTA ready at: http://%s/\n", 
                           WiFi.localIP().toString().c_str());
        } else {
            SerialBT.println("WiFi connection failed, unable to start OTA");
        }
    } else {
        SerialBT.printf("OTA available at: http://%s/\n", 
                       WiFi.localIP().toString().c_str());
    }
}

void setup() {
    // 立即設定關鍵腳位為輸出並設為安全狀態
    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW);
    pinMode(SHOOT_OUTPUT, OUTPUT);
    digitalWrite(SHOOT_OUTPUT, LOW);
    pinMode(MAGAZINE_OUTPUT, OUTPUT);
    digitalWrite(MAGAZINE_OUTPUT, HIGH);
    
    // 優化：移除 Serial.begin(115200) - 省電
    
    // 原始系統腳位初始化
    pinMode(HX710B_dout, INPUT);
    pinMode(Reloading, INPUT_PULLUP);
    pinMode(HX710B_sck, OUTPUT);
    pinMode(HALL_LED_PIN, OUTPUT);
    
    // 霍爾感測器腳位初始化
    pinMode(HALL_LEFT_PIN, INPUT);
    pinMode(HALL_RIGHT_PIN, INPUT);
    
    // 設定PWM通道用於藍牙連線指示燈
    ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(OUTPUT_PIN, LEDC_CHANNEL);
    
    // 初始化燈號狀態
    digitalWrite(HALL_LED_PIN, LOW);
    ledcWrite(LEDC_CHANNEL, 0);
    
    // 初始化呼吸燈計時器
    breatheStartTime = millis();
    
    // 校準霍爾感測器基準值
    calibrateBaselines();
    
    // 藍牙初始化 - 優先啟動以確保快速開機
    SerialBT.begin("BT_GLOCK22_80");
    
    // 初始設定為可發現模式（允許配對）
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    
    // 優化：WiFi 不再自動連線，改為指令控制
    // 這樣可以實現秒開機，藍牙立即可用
    wifiEnabled = false;
}

void loop() {
    static int shootCount = 0;
    static int lastMagazineState = HIGH;
    
    // 只在 WiFi 啟用時才處理 OTA
    if (wifiEnabled) {
        ArduinoOTA.handle();
        server.handleClient();
    }
    
    // 每次迴圈都檢查電磁鐵防燒保護
    checkMagnetProtection();
    
    // 藍牙指令處理
    if (SerialBT.available()) {
        String receivedData = "";
        
        while (SerialBT.available()) {
            char c = SerialBT.read();
            if (c == '\n' || c == '\r') {
                break;
            }
            receivedData += c;
            delay(1);
        }
        
        while(SerialBT.available()) SerialBT.read();
        
        if (receivedData.length() > 0) {
            if (calibrationMode) {
                processCalibrationInput(receivedData);
            } else {
                if (receivedData == "1") {
                    // 使用安全開啟電磁鐵函數
                    safeTurnOnMagnet();
                }
                else if (receivedData == "0") {
                    // 使用安全關閉電磁鐵函數
                    safeTurnOffMagnet();
                }
                else if (receivedData == "2") {
                    // OTA 模式啟動
                    startOTAMode();
                }
                else if (receivedData == "S") {
                    // 使用新的編碼格式發送狀態
                    sendDeviceStatus();
                }
                else if (receivedData == "3") {
                    sendCurrentHallData();
                }
                else if (receivedData == "4") {
                    if (bluetoothConnected) {
                        enterCalibrationMode();
                    } else {
                        SerialBT.println("Error: Must be connected via Bluetooth to modify settings");
                    }
                }
            }
        }
    }
    
    unsigned long currentMillis = millis();
    
    // 簡化心跳包發送機制 - 每2秒發送一次字母 h
    if (bluetoothConnected && (currentMillis - lastHeartbeatTime >= HEARTBEAT_INTERVAL)) {
        lastHeartbeatTime = currentMillis;
        SerialBT.println("h");  // 只發送字母 h，傳送壓力極小
    }
    
    // 射擊脈衝處理（100ms脈衝）
    if (shootPulseActive && (currentMillis - shootPulseStartTime >= 100)) {
        digitalWrite(SHOOT_OUTPUT, LOW);
        shootPulseActive = false;
    }
    
    // 射擊燈號處理（350ms持續時間）
    if (shootLedActive && (currentMillis - shootLedStartTime >= SHOOT_LED_DURATION)) {
        int currentMagazineState = digitalRead(Reloading);
        if (currentMagazineState == LOW) {
            digitalWrite(MAGAZINE_OUTPUT, LOW);
        } else {
            digitalWrite(MAGAZINE_OUTPUT, HIGH);
        }
        shootLedActive = false;
    }
    
    // 檢查藍牙連線狀態
    bluetoothConnected = checkBluetoothConnection();
    
    // 藍牙連接狀態變化處理，包含自動發送狀態
    if (bluetoothConnected != bluetoothWasConnected) {
        bluetoothWasConnected = bluetoothConnected;
        
        if (bluetoothConnected) {
            // 藍牙剛連接時記錄時間並重置狀態標誌
            connectionEstablishedTime = currentMillis;
            initialStatusSent = false;
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        } else {
            // 藍牙斷線時的處理
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            
            // 藍牙斷線時使用安全關閉電磁鐵函數
            if (magnetActive) {
                safeTurnOffMagnet();
            }
            
            if (calibrationMode) {
                calibrationMode = false;
                calibrationStep = 0;
                inputBuffer = "";
            }
        }
    }
    
    // 藍牙連接後立即發送設備狀態，並增加重複發送確保接收
    if (bluetoothConnected && !initialStatusSent && connectionEstablishedTime > 0 && 
        (currentMillis - connectionEstablishedTime >= STATUS_SEND_DELAY)) {
        sendDeviceStatus();
        delay(50); // 短暫延遲
        sendDeviceStatus(); // 重複發送一次確保軟體接收到
        initialStatusSent = true;
        connectionEstablishedTime = 0; // 重置，避免重複發送
    }
    
    // 當藍牙連線狀態改變時重新開始呼吸燈循環
    if (bluetoothConnected != lastBluetoothState) {
        lastBluetoothState = bluetoothConnected;
        breatheStartTime = currentMillis;
    }
    
    // 更新藍牙連線指示燈
    updateBluetoothLED();
    
    // 霍爾感測器檢測
    if (currentMillis - lastReadTime >= READ_INTERVAL) {
        lastReadTime = currentMillis;
        
        int leftValue = readFastHallSensor(HALL_LEFT_PIN);
        int rightValue = readFastHallSensor(HALL_RIGHT_PIN);
        
        int leftDeviation = abs(leftValue - leftBaselineValue);
        int rightDeviation = abs(rightValue - rightBaselineValue);
        
        if (ENABLE_HALL_DEBUG && (currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL)) {
            lastDebugPrintTime = currentMillis;
        }
        
        // 霍爾感測器數值變化檢查，增加抗干擾能力
        bool isLeftGripping = (leftDeviation >= GRIP_THRESHOLD);
        bool isRightGripping = (rightDeviation >= GRIP_THRESHOLD);
        
        // 狀態變化連續性檢查，減少tracker晃動造成的誤觸發
        static int leftStableCount = 0;
        static int rightStableCount = 0;
        static bool lastLeftReading = false;
        static bool lastRightReading = false;
        
        // 左邊穩定性檢查
        if (isLeftGripping == lastLeftReading) {
            leftStableCount++;
        } else {
            leftStableCount = 0;
            lastLeftReading = isLeftGripping;
        }
        
        // 右邊穩定性檢查  
        if (isRightGripping == lastRightReading) {
            rightStableCount++;
        } else {
            rightStableCount = 0;
            lastRightReading = isRightGripping;
        }
        
        // 只有連續3次讀取都一致才認為是真正的狀態變化
        bool stableLeftGripping = (leftStableCount >= 3) ? isLeftGripping : pendingLeftGripState;
        bool stableRightGripping = (rightStableCount >= 3) ? isRightGripping : pendingRightGripState;
        
        // 左邊防抖動處理 - 使用穩定化的讀值
        if (stableLeftGripping != pendingLeftGripState) {
            pendingLeftGripState = stableLeftGripping;
            lastLeftStateChangeTime = currentMillis;
        } else if (currentMillis - lastLeftStateChangeTime >= DEBOUNCE_TIME) {
            if (pendingLeftGripState != currentLeftGripState) {
                currentLeftGripState = pendingLeftGripState;
                
                if (currentLeftGripState) {
                    SerialBT.println("666");
                } else {
                    SerialBT.println("777");
                }
            }
        }
        
        // 右邊防抖動處理 - 使用穩定化的讀值
        if (stableRightGripping != pendingRightGripState) {
            pendingRightGripState = stableRightGripping;
            lastRightStateChangeTime = currentMillis;
        } else if (currentMillis - lastRightStateChangeTime >= DEBOUNCE_TIME) {
            if (pendingRightGripState != currentRightGripState) {
                currentRightGripState = pendingRightGripState;
                
                if (currentRightGripState) {
                    SerialBT.println("444");
                } else {
                    SerialBT.println("555");
                }
            }
        }
        
        // 控制霍爾燈號
        if (currentLeftGripState || currentRightGripState) {
            digitalWrite(HALL_LED_PIN, HIGH);
        } else {
            digitalWrite(HALL_LED_PIN, LOW);
        }
    }
    
    // 優化：HX710B 讀取間隔從 1ms 改為 10ms
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        if (digitalRead(HX710B_dout) == LOW) {
            long result = 0;
            for (int i = 0; i < 24; i++) {
                digitalWrite(HX710B_sck, HIGH);
                digitalWrite(HX710B_sck, LOW);
                result = result << 1;
                if (digitalRead(HX710B_dout)) result++;
            }
            result = result ^ 0x800000;
            for (int i = 0; i < 3; i++) {
                digitalWrite(HX710B_sck, HIGH);
                digitalWrite(HX710B_sck, LOW);
            }
            
            int magazineState = digitalRead(Reloading);
            
            // 彈匣狀態變化檢測並記錄時間
            if (magazineState != lastMagazineState) {
                lastMagazineChangeTime = currentMillis; // 記錄彈匣狀態變化時間
                
                if (magazineState == LOW) {
                    if (!shootLedActive) {
                        digitalWrite(MAGAZINE_OUTPUT, LOW);
                    }
                    SerialBT.println("222");
                } else {
                    if (!shootLedActive) {
                        digitalWrite(MAGAZINE_OUTPUT, HIGH);
                    }
                    SerialBT.println("333");
                }
                lastMagazineState = magazineState;
            }
            
            static long lastResult = 0;
            if ((result - lastResult) > RISE_THRESHOLD) {
                unsigned long now = millis();
                
                // 檢查是否在彈匣狀態變化後的忽略時間內
                if (now - lastMagazineChangeTime > MAGAZINE_CHANGE_IGNORE_TIME) {
                    if (now - lastShootTime > SHOOT_INTERVAL && !shootPulseActive) {
                        shootCount++;
                        digitalWrite(SHOOT_OUTPUT, HIGH);
                        shootPulseStartTime = currentMillis;
                        shootPulseActive = true;
                        
                        digitalWrite(MAGAZINE_OUTPUT, HIGH);
                        shootLedStartTime = currentMillis;
                        shootLedActive = true;
                        
                        SerialBT.println("111");
                        lastShootTime = currentMillis;
                    }
                }
            }
            lastResult = result;
        }
    }
}