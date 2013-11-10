using System;
using Microsoft.SPOT;
using System.Text;

namespace NetduinoApplication1
{

    /*
     * Wrapper class for LCD display over I2C interface.
     * This is to implement the protected-only members of the package.
     */

    public class LCD_Display : LiquidCrystal_I2C
    {
        public LCD_Display(byte lcd_Addr, byte lcd_cols, byte lcd_rows)
            : base(lcd_Addr, lcd_cols, lcd_rows)
        {
        }

        public void writeValue(string s)
        {
            Encoding enc = Encoding.UTF8;

            this.setCursor(0, 0);
            this.backlight();
            this.blink();
            this.write(enc.GetBytes(s));
        }


    }
}
