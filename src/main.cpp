#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <arduinoFFT.h>

// ================ 配置 ================
#define MIC_PIN 34          // 麦克风输入
#define LED_PIN 32          // LED数据线
#define NUM_LEDS 64         // 8x8灯板

// FFT配置
#define SAMPLES 64          // FFT点数
#define SAMPLE_RATE 10000   // 采样率
ArduinoFFT<float> FFT = ArduinoFFT<float>();

// ================ 全局变量 ================
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// FFT数据
float vReal[SAMPLES];
float vImag[SAMPLES] = {0};

// 8条光柱
#define NUM_BARS 8
#define LEDS_PER_BAR 8

float barLevels[NUM_BARS] = {0};
float barTargets[NUM_BARS] = {0};

// 帧率控制
uint32_t frameCount = 0;
uint32_t lastFpsTime = 0;
float fps = 0;

// ================ S形排列映射 ================
// 预计算索引表
uint8_t LED_INDEX_MAP[8][8];

void initLedIndexMap() {
  for (int column = 0; column < 8; column++) {
    for (int row = 0; row < 8; row++) {
      if (column % 2 == 0) {
        int base = (column / 2) * 16;
        LED_INDEX_MAP[column][row] = base + row;
      } else {
        int base = ((column - 1) / 2) * 16 + 15;
        LED_INDEX_MAP[column][row] = base - row;
      }
    }
  }
}

inline uint8_t getLedIndex(uint8_t column, uint8_t row) {
  return LED_INDEX_MAP[column][row];
}

// 颜色：蓝色->紫色->粉色渐变
const uint32_t BAR_COLORS[8] = {
  strip.Color(0, 100, 255),    // 1. 深蓝
  strip.Color(30, 80, 255),    // 2. 蓝
  strip.Color(60, 60, 255),    // 3. 蓝紫
  strip.Color(100, 40, 255),   // 4. 紫
  strip.Color(140, 20, 255),   // 5. 紫红
  strip.Color(180, 0, 230),    // 6. 红紫
  strip.Color(220, 30, 180),   // 7. 粉紫
  strip.Color(255, 60, 150)    // 8. 粉红
};

// 预计算亮度表
uint32_t colorBrightness[8][8];

void initColorBrightness() {
  for (int col = 0; col < 8; col++) {
    uint32_t baseColor = BAR_COLORS[col];
    uint8_t baseR = (baseColor >> 16) & 0xFF;
    uint8_t baseG = (baseColor >> 8) & 0xFF;
    uint8_t baseB = baseColor & 0xFF;
    
    for (int row = 0; row < 8; row++) {
      float brightness = 1.0f - (float)row / 7.0f * 0.5f;
      uint8_t r = (uint8_t)(baseR * brightness);
      uint8_t g = (uint8_t)(baseG * brightness);
      uint8_t b = (uint8_t)(baseB * brightness);
      colorBrightness[col][row] = strip.Color(r, g, b);
    }
  }
}

// ================ 初始化 ================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== 8x8 FFT频谱柱 (低频修正版) ===");
  
  // 初始化映射表
  initLedIndexMap();
  initColorBrightness();
  
  // LED初始化
  strip.begin();
  strip.setBrightness(150);
  
  // 快速启动测试
  strip.clear();
  for (int col = 0; col < 8; col++) {
    for (int row = 0; row < 8; row++) {
      strip.setPixelColor(getLedIndex(col, row), BAR_COLORS[col]);
    }
  }
  strip.show();
  delay(500);
  strip.clear();
  strip.show();
  
  // ADC初始化
  analogReadResolution(12);
  pinMode(MIC_PIN, INPUT);
  
  // 采集一些样本来计算DC偏移
  Serial.println("校准DC偏移...");
  float dcOffset = 0;
  for (int i = 0; i < 100; i++) {
    int raw = analogRead(MIC_PIN);
    dcOffset += (raw / 4095.0f) * 3.3f;
    delay(1);
  }
  dcOffset = dcOffset / 100.0f;
  Serial.printf("DC偏移: %.3fV\n", dcOffset);
  
  Serial.println("系统启动完成");
}

// ================ 改进的音频采集 ================
void collectAudio() {
  uint32_t startTime = micros();
  static float dcOffset = 1.65f;  // 默认值，会在运行时调整
  
  for (int i = 0; i < SAMPLES; i++) {
    // 读取ADC
    int raw = analogRead(MIC_PIN);
    
    // 动态DC偏移校准
    static float dcFilter = 1.65f;
    float currentValue = (raw / 4095.0f) * 3.3f;
    dcFilter = dcFilter * 0.99f + currentValue * 0.01f;  // 低通滤波器
    
    // 减去DC偏移
    vReal[i] = currentValue - dcFilter;
    vImag[i] = 0.0f;
    
    // 时间控制
    while ((micros() - startTime) < (i * 100)) {
      asm volatile ("nop");
    }
  }
}

// ================ 改进的FFT ================
void performFFT() {
  // 应用高通滤波器去除超低频噪声
  static float lastValue = 0;
  float alpha = 0.9f;  // 高通滤波器系数
  
  for (int i = 0; i < SAMPLES; i++) {
    // 一阶高通滤波器
    float filtered = vReal[i] - lastValue;
    lastValue = vReal[i];
    vReal[i] = filtered * alpha;
  }
  
  // 加汉宁窗
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
  
  // 执行FFT
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
}

// ================ 改进的频段提取 - 抑制低频噪声 ================
void extractFrequencyBands() {
  // 清空目标值
  for (int i = 0; i < NUM_BARS; i++) {
    barTargets[i] = 0.0f;
  }
  
  // 跳过DC分量（bin 0），从bin 1开始
  // 频段1: 50-100Hz (降低权重)
  barTargets[0] = vReal[1] * 0.3f + vReal[2] * 0.4f + vReal[3] * 0.3f;
  barTargets[0] *= 0.5f;  // 额外降低低频增益
  
  // 频段2: 100-200Hz
  barTargets[1] = vReal[4] * 0.3f + vReal[5] * 0.4f + vReal[6] * 0.3f;
  barTargets[1] *= 0.7f;  // 适当降低
  
  // 频段3: 200-400Hz (正常)
  barTargets[2] = vReal[7] * 0.25f + vReal[8] * 0.3f + vReal[9] * 0.25f + vReal[10] * 0.2f;
  
  // 频段4: 400-800Hz
  barTargets[3] = vReal[11] * 0.2f + vReal[12] * 0.3f + vReal[13] * 0.3f + vReal[14] * 0.2f;
  
  // 频段5: 800-1600Hz
  barTargets[4] = vReal[15] * 0.2f + vReal[16] * 0.3f + vReal[17] * 0.3f + vReal[18] * 0.2f;
  
  // 频段6: 1600-3200Hz
  barTargets[5] = vReal[19] * 0.2f + vReal[20] * 0.3f + vReal[21] * 0.3f + vReal[22] * 0.4f;
  
  // 频段7: 3200-5000Hz
  barTargets[6] = vReal[23] * 0.1f + vReal[24] * 0.4f + vReal[25] * 0.3f + vReal[26] * 0.2f;
  
  // 频段8: 5000-8000Hz (提高权重，通常音乐高频能量较低)
  barTargets[7] = vReal[27] * 0.2f + vReal[28] * 0.2f + vReal[29] * 0.3f + vReal[30] * 0.2f;
  barTargets[7] *= 1.005f;  // 提高高频增益
  
  // 改进的动态归一化 - 分别处理低频和高频
  static float maxLevelLow = 0.1f;   // 低频最大值
  static float maxLevelMid = 0.1f;   // 中频最大值
  static float maxLevelHigh = 0.08f;  // 高频最大值
  
  // 分别计算各频段组的最大值
  float currentMaxLow = 0.0f;
  float currentMaxMid = 0.0f;
  float currentMaxHigh = 0.0f;
  
  // 低频组 (0-1)
  for (int i = 0; i < 2; i++) {
    if (barTargets[i] > currentMaxLow) currentMaxLow = barTargets[i];
  }
  
  // 中频组 (2-5)
  for (int i = 2; i < 6; i++) {
    if (barTargets[i] > currentMaxMid) currentMaxMid = barTargets[i];
  }
  
  // 高频组 (6-7)
  for (int i = 6; i < 8; i++) {
    if (barTargets[i] > currentMaxHigh) currentMaxHigh = barTargets[i];
  }
  
  // 分别更新各组的最大值
  if (currentMaxLow > maxLevelLow) {
    maxLevelLow = maxLevelLow * 0.7f + currentMaxLow * 0.3f;
  } else {
    maxLevelLow = maxLevelLow * 0.99f;
  }
  
  if (currentMaxMid > maxLevelMid) {
    maxLevelMid = maxLevelMid * 0.7f + currentMaxMid * 0.3f;
  } else {
    maxLevelMid = maxLevelMid * 0.995f;
  }
  
  if (currentMaxHigh > maxLevelHigh) {
    maxLevelHigh = maxLevelHigh * 0.7f + currentMaxHigh * 0.3f;
  } else {
    maxLevelHigh = maxLevelHigh * 1.0f;
  }
  
  // 确保最小值
  if (maxLevelLow < 0.01f) maxLevelLow = 0.01f;
  if (maxLevelMid < 0.01f) maxLevelMid = 0.01f;
  if (maxLevelHigh < 0.01f) maxLevelHigh = 0.01f;
  
  // 分别归一化
  for (int i = 0; i < NUM_BARS; i++) {
    float maxLevel;
    if (i < 2) {
      maxLevel = maxLevelLow;      // 低频
    } else if (i < 6) {
      maxLevel = maxLevelMid;      // 中频
    } else {
      maxLevel = maxLevelHigh;     // 高频
    }
    
    if (maxLevel > 0.0f) {
      float normalized = barTargets[i] / maxLevel;
      
      // 应用不同的压缩曲线
      if (i < 2) {
        // 低频：更强的压缩，抑制噪声
        barTargets[i] = pow(normalized, 0.4f) * 0.8f;
      } else if (i < 6) {
        // 中频：正常压缩
        barTargets[i] = pow(normalized, 0.6f) * 1.2f;
      } else {
        // 高频：更敏感的压缩
        barTargets[i] = pow(normalized, 0.7f) * 1.5f;
      }
    } else {
      barTargets[i] = 0.0f;
    }
    
    // 最终限制
    if (barTargets[i] > 1.0f) barTargets[i] = 1.0f;
    if (barTargets[i] < 0.0f) barTargets[i] = 0.0f;
  }
  
  // 额外的低频抑制：如果只有低频有信号，整体降低
  float lowSum = barTargets[0] + barTargets[1];
  float midSum = barTargets[2] + barTargets[3] + barTargets[4] + barTargets[5];
  float highSum = barTargets[6] + barTargets[7];
  
  if (lowSum > 0.8f && midSum < 0.3f && highSum < 0.2f) {
    // 只有低频有信号，可能是噪声
    barTargets[0] *= 0.3f;
    barTargets[1] *= 0.3f;
  }
}

// ================ 更新光柱 ================
void updateBarLevels() {
  float speed = 0.3f;
  
  for (int bar = 0; bar < NUM_BARS; bar++) {
    float diff = barTargets[bar] - barLevels[bar];
    barLevels[bar] += diff * speed;
    
    if (barLevels[bar] < 0.001f) barLevels[bar] = 0.0f;
    if (barLevels[bar] > 1.0f) barLevels[bar] = 1.0f;
  }
}

// ================ 显示光柱 ================
void displayBars() {
  for (uint8_t column = 0; column < NUM_BARS; column++) {
    uint8_t ledsToLight = (uint8_t)(barLevels[column] * LEDS_PER_BAR + 0.5f);
    if (ledsToLight > LEDS_PER_BAR) ledsToLight = LEDS_PER_BAR;
    
    // 点亮LED
    for (uint8_t row = 0; row < ledsToLight; row++) {
      strip.setPixelColor(LED_INDEX_MAP[column][row], colorBrightness[column][row]);
    }
    
    // 关闭LED
    for (uint8_t row = ledsToLight; row < LEDS_PER_BAR; row++) {
      strip.setPixelColor(LED_INDEX_MAP[column][row], 0);
    }
  }
  
  strip.show();
}

// ================ 计算FPS ================
void calculateFPS() {
  frameCount++;
  uint32_t currentTime = millis();
  
  if (currentTime - lastFpsTime >= 1000) {
    fps = frameCount * 1000.0f / (currentTime - lastFpsTime);
    lastFpsTime = currentTime;
    frameCount = 0;
  }
}

// ================ 主循环 ================
void loop() {
  uint32_t frameStart = micros();
  
  // 1. 采集音频
  collectAudio();
  
  // 2. 执行FFT
  performFFT();
  
  // 3. 提取频段
  extractFrequencyBands();
  
  // 4. 更新光柱
  updateBarLevels();
  
  // 5. 显示光柱
  displayBars();
  
  // 6. 计算FPS
  calculateFPS();
  
  // 7. 调试信息
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    Serial.print("光柱高度: ");
    for (int i = 0; i < NUM_BARS; i++) {
      Serial.printf("%.2f ", barLevels[i]);
    }
    Serial.printf(" | FPS: %.1f\n", fps);
    lastDebug = millis();
  }
  
  // 8. 帧率控制
  uint32_t frameTime = micros() - frameStart;
  const uint32_t targetFrameTime = 6250; // 160FPS
  
  if (frameTime < targetFrameTime) {
    delayMicroseconds(targetFrameTime - frameTime);
  }
}