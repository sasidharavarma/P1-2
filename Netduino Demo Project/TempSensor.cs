/*####################################################################
 Taken from FILE: dht11.h - Library for the Virtuabotix DHT11 Sensor.
 VERSION: 2S0A

 PURPOSE: Measure and return temperature & Humidity. Additionally provides conversions.

 LICENSE: GPL v3 (http://www.gnu.org/licenses/gpl.html)
 GET UPDATES: https://www.virtuabotix.com/

      --##--##--##--##--##--##--##--##--##--##--
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      | ##  ##  ##  ##  ##  ##  ##  ##  ##  ## |
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      | ##  ##  ##  ##  ##  ##  ##  ##  ##  ## |
      ##  ##  ##  ## DHT11 SENSOR ##  ##  ##  ##
      ##  ##  ##  ##  ##FRONT ##  ##  ##  ##  ##
      | ##  ##  ##  ##  ##  ##  ##  ##  ##  ## |
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      | ##  ##  ##  ##  ##  ##  ##  ##  ##  ## |
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
      --##--##--##--##--##--##--##--##--##--##--
          ||       ||          || (Not    ||
          ||       ||          || Used)   ||
        VDD(5V)   Readout(I/O)          Ground

  HISTORY:
  Joseph Dattilo (Virtuabotix LLC) - Version 2S0A (27 May 12)
  -Rewritten to with more powerful Versalino functionality
  Joseph Dattilo (Virtuabotix LLC) - Version 0.4.5 (11/11/11)
  -Made Library Arduino 1.0 Compatible
  Joseph Dattilo (Virtuabotix LLC) - Version 0.4.0 (06/11/11)
  -Fixed bugs (squish)
  Mod by Rob Tillaart - Version 0.3 (28/03/2011)
  Mod by SimKard - Version 0.2 (24/11/2010)
 George Hadjikyriacou - Original version (??)
 * 
 * source from https://www.virtuabotix.com/reference/index.php?title=DHT11_Wiki
 * 
#######################################################################*/

using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;



namespace NetduinoApplication1
{
    public class TempSensor
    {
        public int humidity;
        public int temperature;

        private Cpu.Pin IOPin;
        TristatePort Triport;
        public TempSensor(Cpu.Pin Input)
        {
            IOPin = Input;
            Triport = new TristatePort(IOPin, false, false, Port.ResistorMode.Disabled);
        }

        private void DelayMicroseconds(int microseconds)
        {
            long delayTime = microseconds * 10;
            long delayStart = Utility.GetMachineTime().Ticks;
            while ((Utility.GetMachineTime().Ticks - delayStart) < delayTime) ;
        }

        private long Microseconds()
        {
            return Utility.GetMachineTime().Ticks / 10;
        }

        public int read()
        {
            const int buffersize = 5;

            int[] buffer = new int[buffersize];
            int cnt = 7;
            int idx = 0;

            //clear buffer
            for (int i = 0; i <  buffersize; i++)
                buffer[i] = 0;




            //Write frame delimiter to sensor
            Triport.Active = true;
            Triport.Write(false);
            Thread.Sleep(18); //sleep 18 miliseconds
            Triport.Write(true);

            //Now switch port to input mode and wait for signal
            long time = Microseconds();

            //(40) ~= 61 microseconds
            //DelayMicroseconds(1);
            long delta = Microseconds() - time;
            delta = Microseconds() - time;

            Debug.Print((delta/10).ToString());


            Triport.Active = false; //set tristate to disabled
            
            //InputPort Inport = new InputPort(IOPin,true,

            // Countdown values are pulled from arduiono code, may require tuning
            int loopcount = 10000;
            while(Triport.Read() == false)
                if( loopcount-- == 0) return -2; 

             loopcount = 10000;
            while(Triport.Read() == true)
                if (loopcount-- == 0) return -3;


            //read 40 bits or timeout
            for (int i = 0; i < 40; i++)
            {
                loopcount = 10000;
                while (Triport.Read() == false)
                    if (loopcount-- == 0) return -4;

                long t = Microseconds();

                loopcount = 10000;
                while (Triport.Read() == true)
                    if (loopcount-- == 0) return -5;


                if ((Microseconds() - t) > 40) buffer[idx] |= (1 << cnt);
                if (cnt == 0) //next byte?
                {
                    cnt = 7;
                    idx++;
                }
                else cnt--;

            }

            //write to vars
            humidity = buffer[0];
            temperature = buffer[2];
            int sum = buffer[0] + buffer[2];

            if (buffer[4] != sum) return -1;
            return 0;

        }

    }
}
