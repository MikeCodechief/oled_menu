#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "graph.h" // Include the graph.h header file with bitmap data

// Display configuration
#define SCREEN_WIDTH        128     // OLED display width, in pixels
#define SCREEN_HEIGHT       64      // OLED display height, in pixels
#define SCREEN_I2C_ADDR     0x3C    // I2C address for the display (might be 0x3C or 0x3D)

// Button pins
#define BTN_LEFT            0
#define BTN_SELECT          1
#define BTN_GOBACK          2
#define BTN_RIGHT           3
#define BTN_DOWN            4
#define BTN_UP              7
#define BTN_HOME            10

// Animation and interface constants
#define FRAME_DELAY         42      // Frame delay for animations in milliseconds
#define MENU_ITEM_WIDTH     60      // Width of a menu item including spacing
#define ICON_Y_POS          8       // Y position of icons
#define ANIMATION_STEPS     10      // Steps for smooth scrolling animation
#define SCROLL_SPEED        6       // Pixels to move per animation step

// Menu structure constants
#define MAX_MAIN_ITEMS      4       // Maximum number of main menu items
#define MAX_SUB_ITEMS       8       // Maximum number of sub-items per main item
#define MAX_MENU_LAYERS     2       // Maximum number of menu layers (expandable)

// FreeRTOS task priorities
#define PRIORITY_DISPLAY    3
#define PRIORITY_INPUT      4
#define PRIORITY_ANIMATION  2
#define STACK_SENSORS        1024
#define PRIORITY_SENSORS     2



// FreeRTOS stack sizes
#define STACK_DISPLAY       2048
#define STACK_INPUT         1024
#define STACK_ANIMATION     1024
#define STACK_SCROLL        1024
#define STACK_SENSORS       1024

// Global variables
// Using U8g2 library with I2C and no reset pin
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Menu item structure
typedef struct {
  const char* name;       // Menu item name
  uint8_t frame_count;    // Number of animation frames for this item
  const byte (*frames)[288]; // Array of frame bitmaps (288 bytes per frame)
} MenuItem;

// Menu structure - indexed by [layer][index]
MenuItem menuItems[MAX_MENU_LAYERS][MAX_MAIN_ITEMS][MAX_SUB_ITEMS];
uint8_t menuItemCounts[MAX_MENU_LAYERS][MAX_MAIN_ITEMS]; // Number of items per menu/submenu

// Menu state
typedef struct {
  uint8_t layer;          // Current menu layer (0 for main, 1+ for sub-menus)
  uint8_t parent_index;   // Parent menu index when in a sub-menu
  uint8_t selected;       // Currently selected menu item
  uint8_t current_frame;  // Current animation frame
  bool animation_running; // Whether an animation is currently running
  int scrollOffset;       // Current horizontal scroll offset
  int targetOffset;       // Target horizontal scroll offset
  bool is_scrolling;      // Whether scrolling animation is in progress
  bool is_visible[MAX_SUB_ITEMS]; // Track which items are currently visible
  bool centered[MAX_SUB_ITEMS];   // Track which items are centered (in scope)
} MenuState;

// Global state and synchronization primitives
MenuState menuState;
SemaphoreHandle_t menuStateMutex;

// Sensor update task - refreshes hardware info periodically
void sensorUpdateTask(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    // Only force a display update when we're in the Tasks submenu
    bool needsUpdate = false;
    
    if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
      // Check if we're in the Tasks submenu
      if (menuState.layer == 1 && menuState.parent_index == 1) {
        needsUpdate = true;
      }
      
      xSemaphoreGive(menuStateMutex);
    }
    
    if (needsUpdate) {
      // Signal the display task to update by slightly changing a state value
      // This is a hack to force a refresh - in a real app you might use a queue or event
      if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
        // Toggle animation_running state to trigger update
        menuState.animation_running = !menuState.animation_running;
        xSemaphoreGive(menuStateMutex);
      }
    }
    
    // Update every second
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

// Function prototypes
void drawHorizontalMenu(MenuState* state);
void animationTask(void* parameter);
void scrollTask(void* parameter);
void displayTask(void* parameter);
void inputTask(void* parameter);
void sensorUpdateTask(void* parameter);
void drawBitmap(u8g2_uint_t x, u8g2_uint_t y, const uint8_t* bitmap, uint8_t w, uint8_t h);
void drawSelectionScope(int x, int y);
void enterSubMenu(uint8_t parent_item);
void goBackToParentMenu();
void initializeMenuStructure();

// ===== Bitmap Drawing Helper =====

// Function to draw a bitmap using U8g2 library
void drawBitmap(u8g2_uint_t x, u8g2_uint_t y, const uint8_t* bitmap, uint8_t w, uint8_t h) {
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
  
  // Get current menu item count based on layer and parent
  uint8_t currentCount;
  if (state->layer == 0) {
    currentCount = menuItemCounts[0][0]; // Main menu count
  } else {
    currentCount = menuItemCounts[state->layer][state->parent_index]; // Sub-menu count
  }
  
  // Reset visibility and centered tracking
  for (int i = 0; i < currentCount; i++) {
    state->is_visible[i] = false;
    state->centered[i] = false;
  }
  
  // Draw each menu item
  for (int i = 0; i < currentCount; i++) {
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
      
      // If item is centered, make it active for animation
      if (!state->animation_running) {
        state->animation_running = true;
      }
    }
    
    // Get the appropriate menu item
    const MenuItem* item;
    if (state->layer == 0) {
      item = &menuItems[0][0][i]; // First layer - main menu items
    } else {
      item = &menuItems[state->layer][state->parent_index][i]; // Sub-menu items
    }
    
    // Draw the appropriate icon
    if (state->centered[i]) {
      // Draw animated frame for selected item
      drawBitmap(x, ICON_Y_POS, item->frames[state->current_frame], FRAME_WIDTH, FRAME_HEIGHT);
    } else {
      // Draw first frame for non-selected items
      drawBitmap(x, ICON_Y_POS, item->frames[0], FRAME_WIDTH, FRAME_HEIGHT);
    }
    
    // If this is the centered item, draw selection indicator
    if (isCentered) {
      drawSelectionScope(x, ICON_Y_POS);
    }
  }
  
  // Draw menu name at bottom of screen
  const char* menuName;
  if (state->layer == 0) {
    menuName = "Main Menu";
  } else {
    menuName = menuItems[0][0][state->parent_index].name;
  }
  
  char layerStr[40];
  sprintf(layerStr, "Layer: %d - %s", state->layer + 1, menuName);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 60, layerStr);
  
  u8g2.sendBuffer();
}

// ===== Menu Navigation Functions =====

// Enter sub-menu for the selected item
void enterSubMenu(uint8_t parent_item) {
  if (xSemaphoreTake(menuStateMutex, (TickType_t)100) == pdTRUE) {
    menuState.parent_index = parent_item;
    menuState.layer = 1;
    menuState.selected = 0;
    menuState.scrollOffset = 0;
    menuState.targetOffset = 0;
    
    xSemaphoreGive(menuStateMutex);
  }
}

// Go back to parent menu
void goBackToParentMenu() {
  if (xSemaphoreTake(menuStateMutex, (TickType_t)100) == pdTRUE) {
    menuState.layer = 0;
    menuState.selected = menuState.parent_index;
    menuState.scrollOffset = menuState.selected * MENU_ITEM_WIDTH;
    menuState.targetOffset = menuState.scrollOffset;
    
    xSemaphoreGive(menuStateMutex);
  }
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
      for (int i = 0; i < MAX_SUB_ITEMS; i++) {
        if (menuState.centered[i] && menuState.is_visible[i]) {
          shouldAnimate = true;
          
          // Get the appropriate menu item to check frame count
          const MenuItem* item;
          if (menuState.layer == 0) {
            item = &menuItems[0][0][i]; // First layer - main menu items
          } else {
            item = &menuItems[menuState.layer][menuState.parent_index][i]; // Sub-menu items
          }
          
          // Increment frame counter
          menuState.current_frame++;
          
          // Check if we need to reset frame count
          if (menuState.current_frame >= item->frame_count) {
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
  bool goBackPressed = false;
  bool homePressed = false;
  
  // Initialize button pins
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_GOBACK, INPUT_PULLUP);
  pinMode(BTN_HOME, INPUT_PULLUP);
  
  while (1) {
    // Read button states (active LOW)
    bool leftState = !digitalRead(BTN_LEFT);
    bool rightState = !digitalRead(BTN_RIGHT);
    bool upState = !digitalRead(BTN_UP);
    bool downState = !digitalRead(BTN_DOWN);
    bool selectState = !digitalRead(BTN_SELECT);
    bool goBackState = !digitalRead(BTN_GOBACK);
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
        uint8_t currentCount;
        if (menuState.layer == 0) {
          currentCount = menuItemCounts[0][0]; // Main menu count
        } else {
          currentCount = menuItemCounts[menuState.layer][menuState.parent_index]; // Sub-menu count
        }
        
        if (menuState.selected < currentCount - 1) {
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
        // If in the first layer, enter the submenu
        if (menuState.layer == 0) {
          // Get the selected item and release mutex before enterSubMenu
          uint8_t selected = menuState.selected;
          xSemaphoreGive(menuStateMutex);
          enterSubMenu(selected);
        } else {
          // In submenu, launch the selected function
          Serial.print("Selected sub-menu item: ");
          Serial.print(menuState.selected);
          Serial.print(" of parent: ");
          Serial.println(menuState.parent_index);
          
          // Here you would call the appropriate function for the selected item
          // menuItems[menuState.layer][menuState.parent_index][menuState.selected].function();
          
          xSemaphoreGive(menuStateMutex);
        }
      }
    } else if (!selectState) {
      selectPressed = false;
    }
    
    // Handle GO BACK button
    if (goBackState && !goBackPressed) {
      goBackPressed = true;
      
      if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
        // If in a submenu, go back to parent menu
        if (menuState.layer > 0) {
          xSemaphoreGive(menuStateMutex);
          goBackToParentMenu();
        } else {
          xSemaphoreGive(menuStateMutex);
        }
      }
    } else if (!goBackState) {
      goBackPressed = false;
    }
    
    // Handle HOME button
    if (homeState && !homePressed) {
      homePressed = true;
      
      if (xSemaphoreTake(menuStateMutex, (TickType_t)10) == pdTRUE) {
        // Return to first layer, first menu item
        menuState.layer = 0;
        menuState.selected = 0;
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

// Initialize menu structure
void initializeMenuStructure() {
  // Main menu items (layer 0)
  menuItems[0][0][0] = (MenuItem){"List View", LIST_VIEW_FRAME_COUNT, icon_list_view};
  menuItems[0][0][1] = (MenuItem){"Tasks", TASKS_FRAME_COUNT, tasks_frames};
  menuItems[0][0][2] = (MenuItem){"Folder", FOLDER_WITH_FILE_FRAME_COUNT, folder_with_file_frames};
  menuItems[0][0][3] = (MenuItem){"Settings", GEAR_FRAME_COUNT, gear_frames};
  menuItemCounts[0][0] = 4; // Number of main menu items
  
  // List View sub-items (layer 1, parent 0)
  menuItems[1][0][0] = (MenuItem){"Clock", CLOCK_FRAME_COUNT, clock_frames};
  menuItems[1][0][1] = (MenuItem){"Temperature", TEMP_FRAME_COUNT, temp_frames};
  menuItems[1][0][2] = (MenuItem){"Compass", COMPASS_FRAME_COUNT, compass_frames};
  menuItemCounts[1][0] = 3;
  
  // Tasks sub-items (layer 1, parent 1)
  menuItems[1][1][0] = (MenuItem){"Memory", TASKS_FRAME_COUNT, tasks_frames};
  menuItems[1][1][1] = (MenuItem){"CPU Temp", TASKS_FRAME_COUNT, tasks_frames};
  menuItems[1][1][2] = (MenuItem){"Battery", TASKS_FRAME_COUNT, tasks_frames};
  menuItemCounts[1][1] = 3;
  
  // Folder sub-items (layer 1, parent 2)
  menuItems[1][2][0] = (MenuItem){"Folder Item 1", FOLDER_WITH_FILE_FRAME_COUNT, folder_with_file_frames};
  menuItems[1][2][1] = (MenuItem){"Folder Item 2", FOLDER_WITH_FILE_FRAME_COUNT, folder_with_file_frames};
  menuItems[1][2][2] = (MenuItem){"Folder Item 3", FOLDER_WITH_FILE_FRAME_COUNT, folder_with_file_frames};
  menuItemCounts[1][2] = 3;
  
  // Settings sub-items (layer 1, parent 3)
  menuItems[1][3][0] = (MenuItem){"Settings Item 1", GEAR_FRAME_COUNT, gear_frames};
  menuItems[1][3][1] = (MenuItem){"Settings Item 2", GEAR_FRAME_COUNT, gear_frames};
  menuItems[1][3][2] = (MenuItem){"Settings Item 3", GEAR_FRAME_COUNT, gear_frames};
  menuItemCounts[1][3] = 3;
}

// ===== Hardware Monitoring Functions =====

// Get free heap memory in bytes
uint32_t getFreeMemory() {
  return ESP.getFreeHeap();
}

// Get CPU temperature (for ESP32 devices that support it)
float getCPUTemperature() {
  // ESP32-C3 might not have a temperature sensor
  // This is a placeholder - if your specific ESP32 variant supports it
  #ifdef CONFIG_IDF_TARGET_ESP32
    // Regular ESP32 temperature sensor access
    return temperatureRead();
  #else
    // For ESP32-C3 and other variants without temp sensor
    return 0.0; // Return 0 as placeholder
  #endif
}

// Get battery/power level (example - adjust for your hardware)
float getBatteryLevel() {
  // Assuming you have a battery connected to ADC pin (e.g., GPIO34)
  // You'll need to modify this based on your actual hardware setup
  #define BATTERY_PIN 34
  
  // Read analog value (if you have voltage divider setup)
  int rawValue = analogRead(BATTERY_PIN);
  
  // Convert to voltage (example - adjust for your setup)
  // assuming 3.3V reference and voltage divider if needed
  float voltage = rawValue * (3.3 / 4095.0); 
  
  // Example calculation - modify based on your battery specs
  // Assuming 4.2V is 100% and 3.3V is 0%
  float percentage = (voltage - 3.3) / (4.2 - 3.3) * 100.0;
  
  // Ensure valid range
  if (percentage > 100.0) percentage = 100.0;
  if (percentage < 0.0) percentage = 0.0;
  
  return percentage;
}

void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("ESP32-C3 Nested Menu with FreeRTOS");
  
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
  
  // Initialize menu structure
  initializeMenuStructure();
  
  // Initialize menu state
  menuState.layer = 0;           // Start at first layer
  menuState.parent_index = 0;    // No parent menu initially
  menuState.selected = 0;
  menuState.current_frame = 0;
  menuState.animation_running = true; // Start animation immediately
  menuState.scrollOffset = 0;
  menuState.targetOffset = 0;
  menuState.is_scrolling = false;
  
  // Initialize tracking arrays
  for (int i = 0; i < MAX_SUB_ITEMS; i++) {
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
  
  // Using numeric value 2 instead of PRIORITY_SCROLL constant to avoid compilation issue
  xTaskCreate(
    scrollTask,
    "ScrollTask",
    STACK_SCROLL,
    NULL,
    2, // Same priority value as defined for PRIORITY_SCROLL
    NULL
  );
  
  xTaskCreate(
    sensorUpdateTask,
    "SensorTask",
    STACK_SENSORS,
    NULL,
    PRIORITY_SENSORS,
    NULL
  );
  
  Serial.println("Setup complete, FreeRTOS tasks running");
}

void loop() {
  // Nothing to do here - FreeRTOS tasks handle everything
  vTaskDelay(pdMS_TO_TICKS(1000));
}
