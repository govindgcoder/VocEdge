#include "roboeyes.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

DisplayMode mode = MODE_IDLE;

float eyeOffsetX    = 0;
float eyeOffsetY    = 0;
float targetX       = 0;
float targetY       = 0;
float eyeWidth      = EYE_W;
float eyeHeight     = EYE_H;
float targetWidth   = EYE_W;
float targetHeight  = EYE_H;

bool blinking     = false;
uint32_t lastBlink = 0;

uint32_t nextMovement = 0;
uint32_t lastFrame    = 0;

void drawEye(int x, int y, int w, int h) {
  display.fillRoundRect(x, y, w, h, 6, SSD1306_WHITE);
}

void renderEyes() {
  display.clearDisplay();
  int xOffset = round(eyeOffsetX);
  int yOffset = round(eyeOffsetY);
  int w = round(eyeWidth);
  int h = round(eyeHeight);
  drawEye(LEFT_EYE_X + xOffset, EYE_Y + yOffset, w, h);
  drawEye(RIGHT_EYE_X + xOffset, EYE_Y + yOffset, w, h);
  display.display();
}

void renderBlink() {
  display.clearDisplay();
  int xOffset = round(eyeOffsetX);
  int yOffset = round(eyeOffsetY);
  int w = round(eyeWidth);
  int y = EYE_Y + yOffset + round(eyeHeight / 2.0f);
  display.fillRoundRect(LEFT_EYE_X + xOffset, y, w, 3, 2, SSD1306_WHITE);
  display.fillRoundRect(RIGHT_EYE_X + xOffset, y, w, 3, 2, SSD1306_WHITE);
  display.display();
}

void updateEyeAnimation() {
  eyeOffsetX += (targetX - eyeOffsetX) * MOVE_SMOOTHNESS;
  eyeOffsetY += (targetY - eyeOffsetY) * MOVE_SMOOTHNESS;
  eyeWidth   += (targetWidth - eyeWidth) * SIZE_SMOOTHNESS;
  eyeHeight  += (targetHeight - eyeHeight) * SIZE_SMOOTHNESS;
}

void chooseMovement() {
  switch (random(8)) {
    case 0: targetX = -10; targetY = 0;  break;
    case 1: targetX = 10;  targetY = 0;  break;
    case 2: targetX = -9;  targetY = -7; break;
    case 3: targetX = 9;   targetY = -7; break;
    case 4: targetX = -9;  targetY = 7;  break;
    case 5: targetX = 9;   targetY = 7;  break;
    default: targetX = 0;  targetY = 0;  break;
  }
}

void idle() {

  if (mode != MODE_IDLE) {
    mode = MODE_IDLE;
    targetX = 0; targetY = 0;
    targetWidth = EYE_W; targetHeight = EYE_H;
    nextMovement = millis() + 1000;
  }
  uint32_t now = millis();
  if (now - lastFrame < FRAME_TIME) return;
  lastFrame = now;
  if (!blinking && now >= nextMovement) {
    chooseMovement();
    nextMovement = now + random(1500, 3500);
  }
  if (!blinking && now - lastBlink >= BLINK_INTERVAL) {
    blinking = true;
    lastBlink = now;
  }
  if (blinking && now - lastBlink >= BLINK_DURATION) {
    blinking = false;
    lastBlink = now;
  }
  updateEyeAnimation();
  if (blinking) renderBlink();
  else renderEyes();
}

bool focus() {
  mode = MODE_FOCUS;
  uint32_t now = millis();
  if (now - lastFrame < FRAME_TIME) return false;
  lastFrame = now;
  blinking = false;
  targetX = 0; targetY = 0;
  targetWidth = FOCUS_W; targetHeight = FOCUS_H;
  updateEyeAnimation();
  renderEyes();
  bool positionDone = fabs(eyeOffsetX - targetX) < 0.5f && fabs(eyeOffsetY - targetY) < 0.5f;
  bool sizeDone = fabs(eyeWidth - targetWidth) < 0.5f && fabs(eyeHeight - targetHeight) < 0.5f;
  if (positionDone && sizeDone) {
    eyeOffsetX = 0; eyeOffsetY = 0;
    eyeWidth = FOCUS_W; eyeHeight = FOCUS_H;
    return true;
  }
  return false;
}
