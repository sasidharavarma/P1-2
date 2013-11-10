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
        LCD_Display display;
        int period;
        public AnalogReadClass(LCD_Display display, int period)
        {
            this.display = display;
            this.period = period;
        }

        Dht11Sensor dhtSensor = new Dht11Sensor(Pins.GPIO_PIN_D1, Pins.GPIO_PIN_D2, PullUpResistor.External);

        private double toDegrees(double celcius)
        {
            return celcius * 1.8 + 32;
        }


        public void PollTempHumidity()
        {
            while(true)
            {
                Thread.Sleep(period);
                if (dhtSensor.Read())
                {
                    Debug.Print("DHT sensor Read() ok, RH = " + dhtSensor.Humidity.ToString("F1") + "%, Temp = " + toDegrees(dhtSensor.Temperature).ToString("F1") + "°F");
                    //Also write to LCD display
                    if (display != null )
                        display.writeValue("RH = " + dhtSensor.Humidity.ToString("F1") + "%,         Temp = " + toDegrees(dhtSensor.Temperature).ToString("F1") + "F");
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
        //I2C address to LCD display
        const byte LCD_I2C_ADDRESS = 0x27;

        //
        private int getIdealFreq(double potValue)
        {
            return (int)System.Math.Round((.5 * potValue) / (2 * System.Math.PI * .005));
        }

        private int getFreq(TimeSpan lastPeak)
        {
          return   (Utility.GetMachineTime() - lastPeak).Milliseconds;
        }


        //This loop polls the aio value set by the trimpot to determine the duty cycle which 
        // determines the amplitude on the sin curve of a given itteration.
        private void SinLEDLoop(AnalogInput pot, PWM led, OutputPort relay, LCD_Display display )
        {
            double startValue = 0;
            bool laststate = false;
            double potValue = 0.0;

            TimeSpan lastPeak = Utility.GetMachineTime();

            while (true)
            {
                potValue = pot.Read();

                startValue += .5 * potValue;

                if (startValue > 2 * System.Math.PI)
                {
                    startValue = 0;
                    laststate = !laststate;
                    //relay.Write(laststate);

                    if (display != null)
                        display.writeValue("Frequency: " + getFreq(lastPeak)  + " Hz");
                    lastPeak = Utility.GetMachineTime();
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


            //Initialize I2C interface for HD44780 LCD.
            // Disabled while tinkering with MPU6050
            //LCD_Display Display = new LCD_Display(LCD_I2C_ADDRESS, 20, 4);

            IMU_I2C imu = new IMU_I2C(IMU_I2C.MPU6050_DEFAULT_ADDRESS);


            /* 
             * Spawn task to poll DHT11 temp/humid sensor.
             * This polling takes time and the quality of the hardware/driver mean the
             * polling takes inconsistent amounts of time. Also, the read operation is
             * subject to timeouts which should not be allowed to suspend other tasks.
             * 
             * Constructor accepts handle to LCD display 
             */
            const int readPeriod = 1000;
            //Thread analogReadTask = new Thread(new ThreadStart(new AnalogReadClass(null, readPeriod).PollTempHumidity));
            //analogReadTask.Start();

            //Spin untill task spawn completes
            //while (!analogReadTask.IsAlive) ;

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


            SensorData imuData =  imu.getSensorData();

            while (true)
            {
                Thread.Sleep(1);
                imuData = imu.getSensorData();

                Debug.Print("X Y Z X' Y' Z' " + imuData.Gyroscope_X.ToString() + " " + imuData.Gyroscope_Y.ToString() + " " + imuData.Gyroscope_Z.ToString() + " " + imuData.Acceleration_X.ToString() + " " + imuData.Acceleration_Y.ToString() + " " + imuData.Acceleration_Z.ToString());

            }


            //loop forever to get AIO value and set led pwm
            //Also writes to LCD display providing LED frequency
           // SinLEDLoop(pot, led, relay, null);


        }




        public static void Main()
        {
            new PWM_AIO_Demo_Main();
        }

    }
}
