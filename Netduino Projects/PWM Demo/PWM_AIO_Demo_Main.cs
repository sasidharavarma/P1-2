using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;

/*
 * Implementation note : http://forums.netduino.com/index.php?/topic/8630-pwm-freezing-due-to-pwmchannels-mapping/
 * 
 * PWM channels : 5,6,9,10.
 * 
 * Attempt to map to others will cause chip to freze and require reflash
 */
namespace NetduinoApplication1
{

    public class AnalogReadClass
    {
         public double potValue = 0.0;

        AnalogInput pot = new AnalogInput(AnalogChannels.ANALOG_PIN_A0);

        public void ReadAndSetPWM()
        {
            pot.Scale = 1;
            while(true)
            {
                potValue = pot.Read();
          
            }
        }

       
    }

    public class PWM_AIO_Demo_Main
    {
        static PWM led;
        public static void Main()
        {
  
            AnalogReadClass ARclass = new AnalogReadClass();

            Thread analogReadTask = new Thread(new ThreadStart(ARclass.ReadAndSetPWM));
            analogReadTask.Start();

            while (!analogReadTask.IsAlive) ;

            double freq = 50.0;
            double duty = .5;


            OutputPort onboardLed = new OutputPort(Pins.ONBOARD_LED, false);

            led = new PWM(PWMChannels.PWM_PIN_D5, freq, duty ,false);
           


            led.Frequency = 1000;
            led.DutyCycle = .5;
            led.Start();




            double startValue = 0;

            while (true)
            {
                /*
                onboardLed.Write(true);
                Thread.Sleep(250);
                onboardLed.Write(false);
                Thread.Sleep(250); */

                startValue += .5 * ARclass.potValue;

                if (startValue > 2 * System.Math.PI )
                {
                    startValue = 0;
                }
                Thread.Sleep(5);


                led.DutyCycle = System.Math.Max(0, System.Math.Sin(startValue));


          /*      for (startValue = 4.712; startValue < 10.995; startValue += 0.1)
                {
                    endValue = System.Math.Sin(startValue) * ARclass.potValue + .5;
                    led.DutyCycle = endValue;
                    Thread.Sleep(5);
                    if (startValue == 4.712)
                    {
                        Thread.Sleep(1000);
                    }
                }*/
                
            }

        }

    }
}
