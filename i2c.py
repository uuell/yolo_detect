import time
from RPLCD.i2c import CharLCD 

# --- LCD CONFIGURATION ---
# IMPORTANT: Use the correct address for your LCD (usually 0x27 or 0x3F)
LCD_ADDRESS = 0x27 
LCD_PORT = 1 # I2C port 1 on Raspberry Pi 
LCD_COLS = 16
LCD_ROWS = 2

# Initialize LCD object
try:
    # Use the appropriate expander for your I2C module (PCF8574 is common)
    lcd = CharLCD(i2c_expander='PCF8574', address=LCD_ADDRESS, port=LCD_PORT, cols=LCD_COLS, rows=LCD_ROWS)
    LCD_ENABLED = True
    print(f"✅ LCD initialized at address {LCD_ADDRESS}.")
except Exception as e:
    print(f"⚠️ Could not initialize LCD: {e}. Check wiring and I2C address.")
    LCD_ENABLED = False


# --- Main Display Loop Logic ---

# Initialize your running counters
total_good_beans = 0
total_mold_beans = 0

def update_lcd(good_count, mold_count):
    """
    Function to format and display the counts on the 16x2 LCD.
    
    This function should be called inside your main YOLO inference loop 
    whenever a new bean has been processed and sorted.
    """
    if not LCD_ENABLED:
        return

    # Line 1: Good Beans (Max 16 characters)
    line1 = f"GOOD: {good_count:<10}" # Left-align count, filling with spaces

    # Line 2: Mold Beans (Max 16 characters)
    line2 = f"MOLD: {mold_count:<10}" # Left-align count, filling with spaces
    
    try:
        # Update LCD
        lcd.clear() # Clearing is sometimes necessary to prevent artifacts
        
        # Display Line 1
        lcd.cursor_pos = (0, 0) # Row 0, Col 0
        lcd.write_string(line1[:LCD_COLS]) # Ensure it doesn't exceed 16 chars

        # Display Line 2
        lcd.cursor_pos = (1, 0) # Row 1, Col 0
        lcd.write_string(line2[:LCD_COLS])

    except Exception as e:
        print(f"LCD Write Error: {e}")


# --- Simulation of the YOLO Loop ---
# In your actual script, you would increment total_good_beans or total_mold_beans
# in the servo control section of your main loop, and then call update_lcd().

if LCD_ENABLED:
    print("Starting LCD update simulation. Press Ctrl+C to stop.")
    try:
        while True:
            # SIMULATED DETECTION/SORTING LOGIC
            # In your actual script, these counts would come from your detection logic
            total_good_beans += 1 
            if total_good_beans % 7 == 0:
                total_mold_beans += 1
                
            # Call the function to update the display
            update_lcd(total_good_beans, total_mold_beans)
            
            time.sleep(1) # Update interval (adjust as needed)
            
    except KeyboardInterrupt:
        pass
        
    finally:
        lcd.clear()
        print("LCD cleanup complete.")