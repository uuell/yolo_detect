import time
from RPLCD.i2c import CharLCD 

# --- CONFIGURATION ---
# IMPORTANT: Change the address below if yours is not 0x27 (e.g., use 0x3F)
LCD_ADDRESS = 0x27 
LCD_PORT = 1      # I2C port 1 on Raspberry Pi 2/3/4/Zero
LCD_COLS = 16     # 16 characters wide
LCD_ROWS = 2      # 2 lines tall

# --- INITIALIZATION ---
try:
    # Initialize the LCD object using the PCF8574 I2C expander chip
    lcd = CharLCD(i2c_expander='PCF8574', 
                  address=LCD_ADDRESS, 
                  port=LCD_PORT, 
                  cols=LCD_COLS, 
                  rows=LCD_ROWS)
    print(f"✅ LCD initialized successfully at address {LCD_ADDRESS}.")

except Exception as e:
    print(f"❌ ERROR: Could not initialize LCD. Check I2C address, wiring, and I2C setup in raspi-config.")
    print(f"Error details: {e}")
    exit()

# --- TEST LOOP ---
try:
    # 1. Clear the screen
    lcd.clear() 
    
    # 2. Write a message on the first line
    lcd.cursor_pos = (0, 0) # Row 0, Col 0
    lcd.write_string('I2C LCD Test OK!')

    print("Starting counter on line 2. Press Ctrl+C to stop.")

    # 3. Start a counter on the second line
    counter = 0
    while True:
        # Clear the second line only
        lcd.cursor_pos = (1, 0)
        lcd.write_string(' '*16) 
        
        # Display the counter
        lcd.cursor_pos = (1, 0)
        lcd.write_string(f'Count: {counter}')
        
        counter += 1
        time.sleep(0.5) # Update every half second

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    # 4. Cleanup: Clear the screen before exiting
    lcd.clear()
    print("LCD display cleared and resources released.")