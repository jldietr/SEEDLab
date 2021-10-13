import smbus
from smbus2 import SMBus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


#LCD initialization
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.text_direction = lcd.LEFT_TO_RIGHT

#I2C setup
bus = smbus.SMBus(1)
address = 0x04
def readNumber():
    number = bus.read_byte(address)
    return number
