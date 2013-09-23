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
    public class PWM_AIO_Demo_Main
    {
        static PWM led;
        public static void Main()
        {
  
            double freq = 50.0;
            double duty = .5;


            OutputPort onboardLed = new OutputPort(Pins.ONBOARD_LED, false);

            led = new PWM(PWMChannels.PWM_PIN_D5, freq, duty ,false);
            AnalogInput  pot = new AnalogInput(AnalogChannels.ANALOG_PIN_A0);

            double potValue = 0.0;
            led.Frequency = 1000;
            led.DutyCycle = .5;
            led.Start();

            pot.Scale = 1;

            while (true)
            {
                /*
                onboardLed.Write(true);
                Thread.Sleep(250);
                onboardLed.Write(false);
                Thread.Sleep(250); */
                potValue = pot.Read();
                

                led.DutyCycle = potValue;


            }

        }

    }
}
