#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "graph.h" // Include the graph.h header file

// Display configuration
#define SCREEN_WIDTH        128     // OLED display width, in pixels
#define SCREEN_HEIGHT       64      // OLED display height, in pixels
#define SCREEN_I2C_ADDR     0x3C    // I2C address for the display (might be 0x3C or 0x3D)

// Button pins
#define BTN_LEFT            0
#define BTN_SELECT          1
#define BTN_UNUSED          2
#define BTN_RIGHT           3
#define BTN_DOWN            4
#define BTN_UP              7
#define BTN_HOME            10

// Icon definitions - keep frame width/height, use constants from graph.h
#define FRAME_DELAY         42
// FRAME_WIDTH and FRAME_HEIGHT are already defined in graph.h

// Menu states
#define MENU_SETTINGS       0
#define MENU_TEMPERATURE    1
#define MENU_COMPASS        2
#define MENU_CLOCK          3
#define MENU_AIRCRAFT       4
#define MENU_LIST_VIEW      5
#define MENU_WIFI_SEARCH    6
#define MENU_COUNT          7       // Updated count for total menu items

// Interface constants
#define MENU_ITEM_WIDTH     60      // Width of a menu item including spacing
#define ICON_Y_POS          8       // Y position of icons

// Animation constants
#define ANIMATION_STEPS     10      // Steps for smooth scrolling animation
#define SCROLL_SPEED        6       // Pixels to move per animation step

// FreeRTOS task priorities
#define PRIORITY_DISPLAY    3
#define PRIORITY_INPUT      4
#define PRIORITY_ANIMATION  2
#define PRIORITY_SCROLL     2

// FreeRTOS stack sizes
#define STACK_DISPLAY       2048
#define STACK_INPUT         1024
#define STACK_ANIMATION     1024
#define STACK_SCROLL        1024

// Global variables
// Using U8g2 library with I2C and no reset pin
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Menu state
typedef struct {
  uint8_t selected;         // Currently selected menu item
  uint8_t active;           // Currently active (animating) menu item, 255 if none
  uint8_t current_frame;    // Current animation frame
  bool animation_running;   // Whether an animation is currently running
  int scrollOffset;         // Current horizontal scroll offset
  int targetOffset;         // Target horizontal scroll offset
  bool is_scrolling;        // Whether scrolling animation is in progress
  bool is_visible[MENU_COUNT]; // Track which items are currently visible
  bool centered[MENU_COUNT]; // Track which items are centered (in scope)
} MenuState;

// Global state and synchronization primitives
MenuState menuState;
SemaphoreHandle_t menuStateMutex;

// Function prototypes
void drawHorizontalMenu(MenuState* state);
void animationTask(void* parameter);
void scrollTask(void* parameter);
void displayTask(void* parameter);
void inputTask(void* parameter);
void drawBitmap(u8g2_uint_t x, u8g2_uint_t y, const byte* bitmap, uint8_t w, uint8_t h);

// ===== Bitmap Drawing Helper =====

// Function to draw a bitmap using U8g2 library
void drawBitmap(u8g2_uint_t x, u8g2_uint_t y, const byte* bitmap, uint8_t w, uint8_t h) {
  // Calculate bytes per row
  uint8_t bytesPerRow = (w + 7) / 8;
  
  for (uint8_t row = 0; row < h; row++) {
    for (uint8_t col = 0; col < w; col++) {
      uint16_t byteIndex = row * bytesPerRow + col / 8;
      uint8_t bitIndex = 7 - (col % 8); // MSB first
      
      if (pgm_read_byte(&bitmap[byteIndex]) & (1 << bitIndex)) {
        u8g2.drawPixel(x + col, y + row);
      }
    }
  }
}

// ===== Menu Drawing Functions =====
void drawSelectionScope(int x, int y) {
  // Center the 64x64 scope around the icon
  x = x - (64 - FRAME_WIDTH)/2;
  y = y - (64 - FRAME_HEIGHT)/2;
  
  drawBitmap(x, y, selection_scope, 64, 64);
}

void drawHorizontalMenu(MenuState* state) {
  u8g2.clearBuffer();
  
  // Reset visibility and centered tracking
  for (int i = 0; i < MENU_COUNT; i++) {
    state->is_visible[i] = false;
    state->centered[i] = false;
  }
  
  // Draw each menu item
  for (int i = 0; i < MENU_COUNT; i++) {
    // Calculate the x position based on the item index and scroll offset
    int x = (SCREEN_WIDTH / 2) - FRAME_WIDTH / 2 + (i * MENU_ITEM_WIDTH) - state->scrollOffset;
    
    // Skip if completely out of view
    if (x + FRAME_WIDTH < 0 || x > SCREEN_WIDTH) {
      continue;
    }
    
    // Mark this item as visible
    state->is_visible[i] = true;
    
    // Determine if this item is centered (selected)
    bool isCentered = (abs(x - (SCREEN_WIDTH / 2 - FRAME_WIDTH / 2)) < 5);
    state->centered[i] = isCentered;
    
    if (isCentered) {
      state->selected = i;
      
      // If item is centered, make it active for animation (regardless of selection)
      if (!state->animation_running || state->active != i) {
        state->active = i;
        state->animation_running = true;
      }
    }
    
    // Draw the appropriate icon
    if (i == MENU_SETTINGS) {
      // Draw animated icon if it's centered/active
      if (state->centered[i]) {
        drawBitmap(x, ICON_Y_POS, gear_frames[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
      } else {
        drawBitmap(x, ICON_Y_POS, gear_frames[0], FRAME_WIDTH, FRAME_HEIGHT);
      }
    }
    else if (i == MENU_COMPASS) {
      // Draw animated icon if it's centered/active
      if (state->centered[i]) {
        drawBitmap(x, ICON_Y_POS, compass_frames[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
      } else {
        drawBitmap(x, ICON_Y_POS, compass_frames[0], FRAME_WIDTH, FRAME_HEIGHT);
      }
    }
    else if (i == MENU_CLOCK) {
      if (state->centered[i]) {
        drawBitmap(x, ICON_Y_POS, clock_frames[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
      } else {
        drawBitmap(x, ICON_Y_POS, clock_frames[0], FRAME_WIDTH, FRAME_HEIGHT);
      }
    }
    else if (i == MENU_TEMPERATURE) {
      if (state->centered[i]) {
        drawBitmap(x, ICON_Y_POS, temp_frames[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
      } else {
        drawBitmap(x, ICON_Y_POS, temp_frames[0], FRAME_WIDTH, FRAME_HEIGHT);
      }
    }
    else if (i == MENU_AIRCRAFT) {
      if (state->centered[i]) {
        drawBitmap(x, ICON_Y_POS, aircraft_frames[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
      } else {
        drawBitmap(x, ICON_Y_POS, aircraft_frames[0], FRAME_WIDTH, FRAME_HEIGHT);
      }
    }
    else if (i == MENU_LIST_VIEW) {
      if (state->centered[i]) {
        drawBitmap(x, ICON_Y_POS, icon_list_view[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
      } else {
        drawBitmap(x, ICON_Y_POS, icon_list_view[0], FRAME_WIDTH, FRAME_HEIGHT);
      }
    }
    else if (i == MENU_WIFI_SEARCH) {
      if (state->centered[i]) {
        drawBitmap(x, ICON_Y_POS, wifi_search[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
      } else {
        drawBitmap(x, ICON_Y_POS, wifi_search[0], FRAME_WIDTH, FRAME_HEIGHT);
      }
    }
    
    // If this is the centered item, draw selection indicator
    if (isCentered) {
      drawSelectionScope(x, ICON_Y_POS);
    }
  }
  
  u8g2.sendBuffer();
}

// ===== FreeRTOS Tasks =====

// Animation task - handles animation of the active icon
void animationTask(void* parameter) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    bool shouldAnimate = false;
    
    if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
      // Check if any centered item is visible
      for (int i = 0; i < MENU_COUNT; i++) {
        if (menuState.centered[i] && menuState.is_visible[i]) {
          shouldAnimate = true;
          menuState.active = i;
          
          // Increment frame counter
          menuState.current_frame++;
          
          // Check if we need to reset frame count
          if (i == MENU_SETTINGS && 
              menuState.current_frame >= GEAR_FRAME_COUNT) {
            menuState.current_frame = 0;
          }
          else if (i == MENU_TEMPERATURE && 
                     menuState.current_frame >= THERMO_FRAME_COUNT) {
            menuState.current_frame = 0;
          }
          else if (i == MENU_COMPASS && 
                     menuState.current_frame >= COMPASS_FRAME_COUNT) {
            menuState.current_frame = 0;
          }
          else if (i == MENU_CLOCK && 
                     menuState.current_frame >= CLOCK_FRAME_COUNT) {
            menuState.current_frame = 0;
          }
          else if (i == MENU_AIRCRAFT && 
                     menuState.current_frame >= AIRCRAFT_FRAME_COUNT) {
            menuState.current_frame = 0;
          }
          else if (i == MENU_LIST_VIEW && 
                     menuState.current_frame >= LIST_VIEW_FRAME_COUNT) {
            menuState.current_frame = 0;
          }
          else if (i == MENU_WIFI_SEARCH && 
                     menuState.current_frame >= WIFI_SEARCH_FRAME_COUNT) {
            menuState.current_frame = 0;
          }
          break;
        }
      }
      
      xSemaphoreGive(menuStateMutex);
    }
    
    // Animation frame rate control - only delay for full frame time if animating
    if (shouldAnimate) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FRAME_DELAY));
    } else {
      // If no animation is running or visible, just wait a bit with a short delay
      vTaskDelay(pdMS_TO_TICKS(100));
      xLastWakeTime = xTaskGetTickCount(); // Reset the wake time
    }
  }
}

// Scroll animation task - handles smooth horizontal scrolling
void scrollTask(void* parameter) {
  while (1) {
    bool needsUpdate = false;
    
    if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
      if (menuState.is_scrolling) {
        // Calculate the difference between current offset and target offset
        int diff = menuState.targetOffset - menuState.scrollOffset;
        
        if (abs(diff) <= SCROLL_SPEED) {
          // Close enough, snap to target
          menuState.scrollOffset = menuState.targetOffset;
          menuState.is_scrolling = false;
        } else {
          // Move towards target
          if (diff > 0) {
            menuState.scrollOffset += SCROLL_SPEED;
          } else {
            menuState.scrollOffset -= SCROLL_SPEED;
          }
          
          needsUpdate = true;
        }
      }
      
      xSemaphoreGive(menuStateMutex);
    }
    
    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(16));
  }
}

// Display task - updates the display
void displayTask(void* parameter) {
  bool lastAnimationState = false;
  uint8_t lastFrame = 255;
  int lastOffset = -1;
  
  while (1) {
    bool updateNeeded = false;
    
    if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
      // Check if we need to update the display
      if (lastAnimationState != menuState.animation_running ||
          lastFrame != menuState.current_frame ||
          lastOffset != menuState.scrollOffset) {
        
        updateNeeded = true;
        lastAnimationState = menuState.animation_running;
        lastFrame = menuState.current_frame;
        lastOffset = menuState.scrollOffset;
      }
      
      xSemaphoreGive(menuStateMutex);
    }
    
    // Update display if needed
    if (updateNeeded) {
      drawHorizontalMenu(&menuState);
    }
    
    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(33)); // ~30fps
  }
}

// Input task - handles button inputs
void inputTask(void* parameter) {
  // Button states
  bool leftPressed = false;
  bool rightPressed = false;
  bool upPressed = false;
  bool downPressed = false;
  bool selectPressed = false;
  bool homePressed = false;
  
  // Initialize button pins
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_HOME, INPUT_PULLUP);
  pinMode(BTN_UNUSED, INPUT_PULLUP); // Initialize unused button
  
  while (1) {
    // Read button states (active LOW)
    bool leftState = !digitalRead(BTN_LEFT);
    bool rightState = !digitalRead(BTN_RIGHT);
    bool upState = !digitalRead(BTN_UP);
    bool downState = !digitalRead(BTN_DOWN);
    bool selectState = !digitalRead(BTN_SELECT);
    bool homeState = !digitalRead(BTN_HOME);
    
    // Handle LEFT button
    if (leftState && !leftPressed) {
      leftPressed = true;
      
      if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
        // Scroll menu right (showing items to the left)
        if (menuState.selected > 0) {
          menuState.targetOffset -= MENU_ITEM_WIDTH;
          menuState.is_scrolling = true;
        }
        
        xSemaphoreGive(menuStateMutex);
      }
    } else if (!leftState) {
      leftPressed = false;
    }
    
    // Handle RIGHT button
    if (rightState && !rightPressed) {
      rightPressed = true;
      
      if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
        // Scroll menu left (showing items to the right)
        if (menuState.selected < MENU_COUNT - 1) {
          menuState.targetOffset += MENU_ITEM_WIDTH;
          menuState.is_scrolling = true;
        }
        
        xSemaphoreGive(menuStateMutex);
      }
    } else if (!rightState) {
      rightPressed = false;
    }
    
    // Handle SELECT button
    if (selectState && !selectPressed) {
      selectPressed = true;
      
      if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
        // Launch the selected app/functionality
        Serial.print("Selected item: ");
        Serial.println(menuState.selected);
        
        xSemaphoreGive(menuStateMutex);
      }
    } else if (!selectState) {
      selectPressed = false;
    }
    
    // Handle HOME button
    if (homeState && !homePressed) {
      homePressed = true;
      
      if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
        // Return to first menu item
        menuState.targetOffset = 0;
        menuState.is_scrolling = true;
        
        Serial.println("Home button pressed, returning to first menu item");
        
        xSemaphoreGive(menuStateMutex);
      }
    } else if (!homeState) {
      homePressed = false;
    }
    
    // Handle UP and DOWN buttons (currently just logging)
    if (upState && !upPressed) {
      upPressed = true;
      Serial.println("Up button pressed");
    } else if (!upState) {
      upPressed = false;
    }
    
    if (downState && !downPressed) {
      downPressed = true;
      Serial.println("Down button pressed");
    } else if (!downState) {
      downPressed = false;
    }
    
    // Avoid consuming too much CPU
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ===== Main Setup and Loop =====

void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("ESP32-C3 Horizontal Menu with FreeRTOS");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize display
  u8g2.begin();
  
  // Cyberpunk display text
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_tenstamps_mf); // Tech/distorted font

  // Get string widths to center
  int cyber_width = u8g2.getStrWidth("CYBER");
  int to_width = u8g2.getStrWidth("TO");
  int real_width = u8g2.getStrWidth("REAL");

  // Draw strings centered horizontally
  u8g2.drawStr((128-cyber_width)/2, 20, "CYBER");
  u8g2.drawStr((128-to_width)/2, 35, "TO");
  u8g2.drawStr((128-real_width)/2, 50, "REAL");

  u8g2.sendBuffer();
  delay(2000);

  
  // Initialize menu state
  menuState.selected = 0;
  menuState.active = 0; // Start with first item active
  menuState.current_frame = 0;
  menuState.animation_running = true; // Start animation immediately
  menuState.scrollOffset = 0;
  menuState.targetOffset = 0;
  menuState.is_scrolling = false;
  
  // Initialize tracking arrays
  for (int i = 0; i < MENU_COUNT; i++) {
    menuState.is_visible[i] = false;
    menuState.centered[i] = false;
  }
  menuState.centered[0] = true; // First item is centered initially
  
  // Create RTOS primitives
  menuStateMutex = xSemaphoreCreateMutex();
  
  // Create tasks
  xTaskCreate(
    displayTask,
    "DisplayTask",
    STACK_DISPLAY,
    NULL,
    PRIORITY_DISPLAY,
    NULL
  );
  
  xTaskCreate(
    inputTask,
    "InputTask",
    STACK_INPUT,
    NULL,
    PRIORITY_INPUT,
    NULL
  );
  
  xTaskCreate(
    animationTask,
    "AnimationTask",
    STACK_ANIMATION,
    NULL,
    PRIORITY_ANIMATION,
    NULL
  );
  
  xTaskCreate(
    scrollTask,
    "ScrollTask",
    STACK_SCROLL,
    NULL,
    PRIORITY_SCROLL,
    NULL
  );
  
  Serial.println("Setup complete, FreeRTOS tasks running");
}

void loop() {
  // Nothing to do here - FreeRTOS tasks handle everything
  vTaskDelay(pdMS_TO_TICKS(1000));
}
