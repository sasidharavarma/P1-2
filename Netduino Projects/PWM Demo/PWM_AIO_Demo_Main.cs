using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;
using System.Text;


/*
 * Implementation note : http://forums.netduino.com/index.php?/topic/8630-pwm-freezing-due-to-pwmchannels-mapping/
 * 
 * PWM channels : 5,6,9,10.
 * 
 * Attempt to map to others will cause chip to freze and require reflash
 */
namespace NetduinoApplication1
{


    /*
     * Class intended for use spawned in separate thread.
     */
    public class AnalogReadClass
    {
        Dht11Sensor dhtSensor = new Dht11Sensor(Pins.GPIO_PIN_D1, Pins.GPIO_PIN_D2, PullUpResistor.External);

        private double toDegrees(double celcius)
        {
            return celcius * 1.8 + 32;
        }


        public void PollTempHumidity()
        {
            while(true)
            {
                Thread.Sleep(2000);
                if (dhtSensor.Read())
                {
                    Debug.Print("DHT sensor Read() ok, RH = " + dhtSensor.Humidity.ToString("F1") + "%, Temp = " + toDegrees(dhtSensor.Temperature).ToString("F1") + "°F");
                }
                else
                {
                    Debug.Print("DHT sensor Read() failed");
                }
            }
        }
    }

    public class PWM_AIO_Demo_Main
    {
        private void PollAndWriteLoop(AnalogInput pot, PWM led, OutputPort relay )
        {
            double startValue = 0;
            bool laststate = false;

            while (true)
            {
                double potValue = 0.0;
                potValue = pot.Read();

                startValue += .5 * potValue;

                if (startValue > 2 * System.Math.PI)
                {
                    startValue = 0;
                    laststate = !laststate;
                    //relay.Write(laststate);
                }
                Thread.Sleep(5);


                led.DutyCycle = System.Math.Max(0, System.Math.Sin(startValue));

            }
        }
        public PWM_AIO_Demo_Main()
        {
                    
            //AIO pin connected to arbitrary range potentiometer.
            AnalogInput pot = new AnalogInput(AnalogChannels.ANALOG_PIN_A0);
            pot.Scale = 1; //sets range value that is returned by aio read()

            /* 
             * Spawn task to poll DHT11 temp/humid sensor.
             * This polling takes time and the quality of the hardware/driver mean the
             * polling takes inconsistent amounts of time. Also, the read operation is
             * subject to timeouts which should not be allowed to suspend other tasks.
             */
            Thread analogReadTask = new Thread(new ThreadStart(new AnalogReadClass().PollTempHumidity));
            analogReadTask.Start();

            //Spin untill task spawn completes
            while (!analogReadTask.IsAlive) ;

            //initialize PWM properties before handoff to forever loop
            double freq = 1000;
            double duty = .5;

            OutputPort onboardLed = new OutputPort(Pins.ONBOARD_LED, false);
            PWM led = new PWM(PWMChannels.PWM_PIN_D5, freq, duty, false);

            //Another output port to drive transistor, which drives coil of relay.
            OutputPort relay = new OutputPort(Pins.GPIO_PIN_D0, false);

            //Starts pwm channel at initialized rate/duty
            //Will be modified by pollwrite loop
            led.Start();

            LCD_Display Display = new LCD_Display(0x27, 20, 4);
            Display.writeValue("Good");

            //loop forever to get AIO value and set led pwm
            PollAndWriteLoop(pot, led, relay);


        }

        /*
         * Wrapper class for LCD display over I2C interface.
         * This is to implement the protected-only members of the package.
         */

        internal class LCD_Display : LiquidCrystal_I2C
        {
            public LCD_Display(byte lcd_Addr, byte lcd_cols, byte lcd_rows)
                : base( lcd_Addr,  lcd_cols,  lcd_rows)
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



        public static void Main()
        {
            new PWM_AIO_Demo_Main();
        }

    }
}
