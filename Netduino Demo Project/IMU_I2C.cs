using System;
using System.Threading;
using System.Collections;
using Microsoft.SPOT;
using Toolbox.NETMF;
using Toolbox.NETMF.Hardware;
/*
 * Driver for MPU-6050 3 axis gyro/accy over i2c interface
 * 
 * Sources : https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * http://forums.netduino.com/index.php?/topic/9185-mpu6050-with-netduino/
 * http://www.botched.co.uk/pic-tutorials/mpu6050-setup-data-aquisition/
 */
namespace NetduinoApplication1
{

    /// <summary>
    /// Upon construction, converts given buffer and offset into raw format gyro and accelerometer values
    /// </summary>
    public class SensorData
    {
        public double Acceleration_X = 0;
        public double Acceleration_Y = 0;
        public double Acceleration_Z = 0;
        public double Temperature = 0;

        public double Gyroscope_X = 0;
        public double Gyroscope_Y = 0;
        public double Gyroscope_Z = 0;


        public SensorData(byte[] buffer, Int16[] GyroOffsets, bool normalize)
        {
            // Result of the acceleration axis
            Acceleration_X = (Int16)((((UInt16)buffer[0]) << 8) | buffer[1]);  //field responds to breadboard longitudinal axis orientation
            Acceleration_Y = (Int16)((((UInt16)buffer[2]) << 8) | buffer[3]);
            Acceleration_Z = (Int16)((((UInt16)buffer[4]) << 8) | buffer[5]);

            // Restult of temperature
            Temperature = (Int16)((((UInt16)buffer[6]) << 8) | buffer[7]);

            // Result of the gyroscope axis
            Gyroscope_X = (Int16)((((UInt16)buffer[8]) << 8) | buffer[9]) - GyroOffsets[0];
            Gyroscope_Y = (Int16)((((UInt16)buffer[10]) << 8) | buffer[11]) - GyroOffsets[1];
            Gyroscope_Z = (Int16)((((UInt16)buffer[12]) << 8) | buffer[13]) - GyroOffsets[2];
        }
    }


    /// <summary>
    /// Encapsulates communication to MPU6050 IMU. Initializes and configures device.
    /// </summary>
    class IMU_I2C
    {

        //Interface types
        private const int MAX_FREQ = 100;

        private MultiI2C _I2C;

        const byte Expected_I2C_Address = 0x68;

        Int16[] GyroOffsets = new Int16[3];
        int[] Gyrosums = new int[3];


        /// <summary>
        /// Sets gyro offset array based on average of many samples
        /// </summary>
        private void Callibrate()
        {
            const Int16 AverageGyroBufferSize = 1000;

            for (int i = 0; i < AverageGyroBufferSize; i++)
            {

                byte[] gyro = new byte[6];

                _I2C.Write(new byte[] { MPU6050_RA_GYRO_XOUT_H });
                _I2C.Read(gyro);

                Gyrosums[0] += (Int16)((UInt16)(gyro[0]) << 8 | gyro[1]);
                Gyrosums[1] += (Int16)((UInt16)(gyro[2]) << 8 | gyro[3]);
                Gyrosums[2] += (Int16)((UInt16)(gyro[4]) << 8 | gyro[5]);

                Thread.Sleep(1); // Small delay to accumulate average over 1 second
            }

            for (int i = 0; i < 3; i++)
                GyroOffsets[i] = (Int16)((double)Gyrosums[i] / (double)AverageGyroBufferSize);

            Debug.Print("Offset X " + GyroOffsets[0].ToString());
            Debug.Print("Offset Y " + GyroOffsets[1].ToString());
            Debug.Print("Offset Z " + GyroOffsets[2].ToString());
        }


        /// <summary>
        /// Writes hardcoded initialization values to MPU. Then verifies set values.
        /// Writes to console whether initialization passed or failed.
        /// </summary>
        private void Initialize()
        {
            //The following configuration is taken from 

            UInt16[] InitializationTable = new UInt16[100];

            //Data transfer to and from the FIFO buffer
            InitializationTable[0] = (UInt16)MPU6050_RA_FIFO_R_W << 8 | 0x00;
            //Sets sample rate to 8000/1+7 = 1000Hz
            InitializationTable[1] = (UInt16)MPU6050_RA_SMPLRT_DIV << 8 | 0x07;
            //Disable FSync, 256Hz DLPF
            InitializationTable[2] = (UInt16)MPU6050_RA_CONFIG << 8 | 0x00;
            //Disable gyro self tests, scale of 500 degrees/s
            InitializationTable[3] = (UInt16)MPU6050_RA_GYRO_CONFIG << 8 | 0x08;
            //Disable accel self tests, scale of +-2g, no DHPF
            InitializationTable[4] = (UInt16)MPU6050_RA_ACCEL_CONFIG << 8 | 0x00;
            //Freefall threshold of |0mg|
            InitializationTable[5] = (UInt16)MPU6050_RA_FF_THR << 8 | 0x00;
            //Freefall duration limit of 0
            InitializationTable[6] = (UInt16)MPU6050_RA_FF_DUR << 8 | 0x00;
            //Motion threshold of 0mg
            InitializationTable[7] = (UInt16)MPU6050_RA_MOT_THR << 8 | 0x00;
            //Motion duration of 0s
            InitializationTable[8] = (UInt16)MPU6050_RA_MOT_DUR << 8 | 0x00;
            //Zero motion threshold
            InitializationTable[9] = (UInt16)MPU6050_RA_ZRMOT_THR << 8 | 0x00;
            //Zero motion duration threshold
            InitializationTable[10] = (UInt16)MPU6050_RA_ZRMOT_DUR << 8 | 0x00;
            //Disable sensor output to FIFO buffer
            InitializationTable[11] = (UInt16)MPU6050_RA_FIFO_EN << 8 | 0x00;

            //AUX I2C setup
            //Sets AUX I2C to single master control, plus other config
            InitializationTable[12] = (UInt16)MPU6050_RA_I2C_MST_CTRL << 8 | 0x00;
            //Setup AUX I2C slaves
            InitializationTable[13] = (UInt16)MPU6050_RA_I2C_SLV0_ADDR << 8 | 0x00;
            InitializationTable[14] = (UInt16)MPU6050_RA_I2C_SLV0_REG << 8 | 0x00;
            InitializationTable[15] = (UInt16)MPU6050_RA_I2C_SLV0_CTRL << 8 | 0x00;
            InitializationTable[16] = (UInt16)MPU6050_RA_I2C_SLV1_ADDR << 8 | 0x00;
            InitializationTable[17] = (UInt16)MPU6050_RA_I2C_SLV1_REG << 8 | 0x00;
            InitializationTable[18] = (UInt16)MPU6050_RA_I2C_SLV1_CTRL << 8 | 0x00;
            InitializationTable[19] = (UInt16)MPU6050_RA_I2C_SLV2_ADDR << 8 | 0x00;
            InitializationTable[20] = (UInt16)MPU6050_RA_I2C_SLV2_REG << 8 | 0x00;
            InitializationTable[21] = (UInt16)MPU6050_RA_I2C_SLV2_CTRL << 8 | 0x00;
            InitializationTable[22] = (UInt16)MPU6050_RA_I2C_SLV3_ADDR << 8 | 0x00;
            InitializationTable[23] = (UInt16)MPU6050_RA_I2C_SLV3_REG << 8 | 0x00;
            InitializationTable[24] = (UInt16)MPU6050_RA_I2C_SLV3_CTRL << 8 | 0x00;
            InitializationTable[25] = (UInt16)MPU6050_RA_I2C_SLV4_ADDR << 8 | 0x00;
            InitializationTable[26] = (UInt16)MPU6050_RA_I2C_SLV4_REG << 8 | 0x00;
            InitializationTable[27] = (UInt16)MPU6050_RA_I2C_SLV4_DO << 8 | 0x00;
            InitializationTable[28] = (UInt16)MPU6050_RA_I2C_SLV4_CTRL << 8 | 0x00;
            InitializationTable[29] = (UInt16)MPU6050_RA_I2C_SLV4_DI << 8 | 0x00;
            //MPU6050_RA_I2C_MST_STATUS //Read-only
            //Setup INT pin and AUX I2C pass through
            InitializationTable[30] = (UInt16)MPU6050_RA_INT_PIN_CFG << 8 | 0x00;
            //Enable data ready interrupt
            InitializationTable[31] = (UInt16)MPU6050_RA_INT_ENABLE << 8 | 0x00;


            //Slave out, dont care
            InitializationTable[32] = (UInt16)MPU6050_RA_I2C_SLV0_DO << 8 | 0x00;
            InitializationTable[33] = (UInt16)MPU6050_RA_I2C_SLV1_DO << 8 | 0x00;
            InitializationTable[34] = (UInt16)MPU6050_RA_I2C_SLV2_DO << 8 | 0x00;
            InitializationTable[35] = (UInt16)MPU6050_RA_I2C_SLV3_DO << 8 | 0x00;
            //More slave config
            InitializationTable[36] = (UInt16)MPU6050_RA_I2C_MST_DELAY_CTRL << 8 | 0x00;
            //Reset sensor signal paths
            InitializationTable[37] = (UInt16)MPU6050_RA_SIGNAL_PATH_RESET << 8 | 0x00;
            //Motion detection control
            InitializationTable[38] = (UInt16)MPU6050_RA_MOT_DETECT_CTRL << 8 | 0x00;
            //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
            InitializationTable[39] = (UInt16)MPU6050_RA_USER_CTRL << 8 | 0x00;

            int InitializationTableLength = 40;

            bool InitializationFailure = false;


            //Note : comments mirrored from soruce

            // Sensor init
            // Sleep und Reset execute:
            // The HEX value 0x6B is for the Power Management 1
            // The secound Hex value includes the 'Bit' for reset.
            _I2C.Write(new byte[] { MPU6050_RA_PWR_MGMT_1, 0x80 });

            Thread.Sleep(30);

            _I2C.Write(new byte[] { MPU6050_RA_PWR_MGMT_2, 0x00 });


            // Stop sleep and Clock setup:
            // The HEX value 0x6B is for the Power Management 1
            // The secound Hex value 0x00 set the 'Clock Select'
            // to 'Internal 8MHz oscillator'
            // If you don't need the temperature sens,
            // you can set the HEX value 0x08.
            _I2C.Write(new byte[] { MPU6050_RA_PWR_MGMT_1, 0x00 });

            // Configuration setup:
            // This Configuration activates the Low Pass Filter (DLPF).
            // Setting => Acc=5Hz, Delay=19.0ms, Gyro=5Hz, Delay=18.6ms, Fs=1kHz
            //_I2C.Write(new byte[] { MPU6050_RA_CONFIG, 0x06 }); //is now set to zero from another source 

            //Read I2C address register. If not expected value, raise initialization exception
            byte[] returnvalue = new byte[1];
            _I2C.WriteRead(new byte[] { MPU6050_RA_WHO_AM_I }, returnvalue);

            if (returnvalue[0] != Expected_I2C_Address)
            {
                Debug.Print("MPU-6050 Initialization failure. Returned : " + returnvalue[0].ToString());
                InitializationFailure = true;
            }

            //Write the initialization vector to the device
            for (int i = 0; i < InitializationTableLength; i++)
            {
                UInt16 pair = InitializationTable[i];
                //_I2C.Write(new UInt16[] { pair });
                _I2C.Write(new byte[] { (byte)(pair >> 8), (byte)(pair & 0x0F) });

            }

            //Assuming a 10 millisecond delay is appropreate between assignments and read
            Thread.Sleep(10);

            //Verify assignments were sucessful
            for (int i = 0; i < InitializationTableLength; i++)
            {
                UInt16 pair = InitializationTable[i];
                Thread.Sleep(1);

                _I2C.WriteRead(new byte[] { (byte)(pair >> 8) }, returnvalue);
                if ((pair & 0x0F) != (returnvalue[0]))
                {
                    Debug.Print("MPU6050 Initialization Sanity Check Fail: 0x" + (pair).ToString("X4") + " was read as 0x" + returnvalue[0].ToString("X4"));
                    InitializationFailure = true;
                }

            }

            if (!InitializationFailure)
                Debug.Print("MPU6050 Setup Complete");
            else
                Debug.Print("MPU6050 Setup Failed");
        }


        /// <summary>
        /// Establishes I2C connection to MPU6050, Initializes and gets Gyro offsets.
        /// </summary>
        public IMU_I2C(byte address)
        {
            _I2C = new MultiI2C(address, MAX_FREQ);
            Thread.Sleep(10);
            Initialize();
            Thread.Sleep(10);
            Callibrate();
        }


        /// <summary>
        /// Returns sensordata class containing most recent raw inertal data.
        /// </summary>
        public SensorData getSensorData()
        {
            byte[] buffer = new byte[14];
            buffer[0] = MPU6050_RA_ACCEL_XOUT_H;


            //Read the 14 bytes starting from this register: XH/L, YH/L, XY/L, TempH/L, GyroH/L(x,y,z)
            _I2C.Write(new byte[] { buffer[0] });
            _I2C.Read(buffer);

            return new SensorData(buffer, GyroOffsets, false);
        }


        # region AddressConstants
        public const byte MPU6050_ADDRESS_AD0_LOW = 0x68;
        public const byte MPU6050_ADDRESS_AD0_HIGH = 0x69;
        public const byte MPU6050_DEFAULT_ADDRESS = MPU6050_ADDRESS_AD0_LOW;

        public const byte MPU6050_RA_XG_OFFS_TC = 0x00; //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
        public const byte MPU6050_RA_YG_OFFS_TC = 0x01; //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
        public const byte MPU6050_RA_ZG_OFFS_TC = 0x02;//[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
        public const byte MPU6050_RA_X_FINE_GAIN = 0x03; //[7:0] X_FINE_GAIN
        public const byte MPU6050_RA_Y_FINE_GAIN = 0x04; //[7:0] Y_FINE_GAIN
        public const byte MPU6050_RA_Z_FINE_GAIN = 0x05; //[7:0] Z_FINE_GAIN
        public const byte MPU6050_RA_XA_OFFS_H = 0x06; //[15:0] XA_OFFS
        public const byte MPU6050_RA_XA_OFFS_L_TC = 0x07;
        public const byte MPU6050_RA_YA_OFFS_H = 0x08; //[15:0] YA_OFFS
        public const byte MPU6050_RA_YA_OFFS_L_TC = 0x09;
        public const byte MPU6050_RA_ZA_OFFS_H = 0x0A; //[15:0] ZA_OFFS
        public const byte MPU6050_RA_ZA_OFFS_L_TC = 0x0B;
        public const byte MPU6050_RA_XG_OFFS_USRH = 0x13; //[15:0] XG_OFFS_USR
        public const byte MPU6050_RA_XG_OFFS_USRL = 0x14;
        public const byte MPU6050_RA_YG_OFFS_USRH = 0x15; //[15:0] YG_OFFS_USR
        public const byte MPU6050_RA_YG_OFFS_USRL = 0x16;
        public const byte MPU6050_RA_ZG_OFFS_USRH = 0x17; //[15:0] ZG_OFFS_USR
        public const byte MPU6050_RA_ZG_OFFS_USRL = 0x18;
        public const byte MPU6050_RA_SMPLRT_DIV = 0x19;
        public const byte MPU6050_RA_CONFIG = 0x1A;
        public const byte MPU6050_RA_GYRO_CONFIG = 0x1B;
        public const byte MPU6050_RA_ACCEL_CONFIG = 0x1C;
        public const byte MPU6050_RA_FF_THR = 0x1D;
        public const byte MPU6050_RA_FF_DUR = 0x1E;
        public const byte MPU6050_RA_MOT_THR = 0x1F;
        public const byte MPU6050_RA_MOT_DUR = 0x20;
        public const byte MPU6050_RA_ZRMOT_THR = 0x21;
        public const byte MPU6050_RA_ZRMOT_DUR = 0x22;
        public const byte MPU6050_RA_FIFO_EN = 0x23;
        public const byte MPU6050_RA_I2C_MST_CTRL = 0x24;
        public const byte MPU6050_RA_I2C_SLV0_ADDR = 0x25;
        public const byte MPU6050_RA_I2C_SLV0_REG = 0x26;
        public const byte MPU6050_RA_I2C_SLV0_CTRL = 0x27;
        public const byte MPU6050_RA_I2C_SLV1_ADDR = 0x28;
        public const byte MPU6050_RA_I2C_SLV1_REG = 0x29;
        public const byte MPU6050_RA_I2C_SLV1_CTRL = 0x2A;
        public const byte MPU6050_RA_I2C_SLV2_ADDR = 0x2B;
        public const byte MPU6050_RA_I2C_SLV2_REG = 0x2C;
        public const byte MPU6050_RA_I2C_SLV2_CTRL = 0x2D;
        public const byte MPU6050_RA_I2C_SLV3_ADDR = 0x2E;
        public const byte MPU6050_RA_I2C_SLV3_REG = 0x2F;
        public const byte MPU6050_RA_I2C_SLV3_CTRL = 0x30;
        public const byte MPU6050_RA_I2C_SLV4_ADDR = 0x31;
        public const byte MPU6050_RA_I2C_SLV4_REG = 0x32;
        public const byte MPU6050_RA_I2C_SLV4_DO = 0x33;
        public const byte MPU6050_RA_I2C_SLV4_CTRL = 0x34;
        public const byte MPU6050_RA_I2C_SLV4_DI = 0x35;
        public const byte MPU6050_RA_I2C_MST_STATUS = 0x36;
        public const byte MPU6050_RA_INT_PIN_CFG = 0x37;
        public const byte MPU6050_RA_INT_ENABLE = 0x38;
        public const byte MPU6050_RA_DMP_INT_STATUS = 0x39;
        public const byte MPU6050_RA_INT_STATUS = 0x3A;
        public const byte MPU6050_RA_ACCEL_XOUT_H = 0x3B;
        public const byte MPU6050_RA_ACCEL_XOUT_L = 0x3C;
        public const byte MPU6050_RA_ACCEL_YOUT_H = 0x3D;
        public const byte MPU6050_RA_ACCEL_YOUT_L = 0x3E;
        public const byte MPU6050_RA_ACCEL_ZOUT_H = 0x3F;
        public const byte MPU6050_RA_ACCEL_ZOUT_L = 0x40;
        public const byte MPU6050_RA_TEMP_OUT_H = 0x41;
        public const byte MPU6050_RA_TEMP_OUT_L = 0x42;
        public const byte MPU6050_RA_GYRO_XOUT_H = 0x43;
        public const byte MPU6050_RA_GYRO_XOUT_L = 0x44;
        public const byte MPU6050_RA_GYRO_YOUT_H = 0x45;
        public const byte MPU6050_RA_GYRO_YOUT_L = 0x46;
        public const byte MPU6050_RA_GYRO_ZOUT_H = 0x47;
        public const byte MPU6050_RA_GYRO_ZOUT_L = 0x48;
        public const byte MPU6050_RA_EXT_SENS_DATA_00 = 0x49;
        public const byte MPU6050_RA_EXT_SENS_DATA_01 = 0x4A;
        public const byte MPU6050_RA_EXT_SENS_DATA_02 = 0x4B;
        public const byte MPU6050_RA_EXT_SENS_DATA_03 = 0x4C;
        public const byte MPU6050_RA_EXT_SENS_DATA_04 = 0x4D;
        public const byte MPU6050_RA_EXT_SENS_DATA_05 = 0x4E;
        public const byte MPU6050_RA_EXT_SENS_DATA_06 = 0x4F;
        public const byte MPU6050_RA_EXT_SENS_DATA_07 = 0x50;
        public const byte MPU6050_RA_EXT_SENS_DATA_08 = 0x51;
        public const byte MPU6050_RA_EXT_SENS_DATA_09 = 0x52;
        public const byte MPU6050_RA_EXT_SENS_DATA_10 = 0x53;
        public const byte MPU6050_RA_EXT_SENS_DATA_11 = 0x54;
        public const byte MPU6050_RA_EXT_SENS_DATA_12 = 0x55;
        public const byte MPU6050_RA_EXT_SENS_DATA_13 = 0x56;
        public const byte MPU6050_RA_EXT_SENS_DATA_14 = 0x57;
        public const byte MPU6050_RA_EXT_SENS_DATA_15 = 0x58;
        public const byte MPU6050_RA_EXT_SENS_DATA_16 = 0x59;
        public const byte MPU6050_RA_EXT_SENS_DATA_17 = 0x5A;
        public const byte MPU6050_RA_EXT_SENS_DATA_18 = 0x5B;
        public const byte MPU6050_RA_EXT_SENS_DATA_19 = 0x5C;
        public const byte MPU6050_RA_EXT_SENS_DATA_20 = 0x5D;
        public const byte MPU6050_RA_EXT_SENS_DATA_21 = 0x5E;
        public const byte MPU6050_RA_EXT_SENS_DATA_22 = 0x5F;
        public const byte MPU6050_RA_EXT_SENS_DATA_23 = 0x60;
        public const byte MPU6050_RA_MOT_DETECT_STATUS = 0x61;
        public const byte MPU6050_RA_I2C_SLV0_DO = 0x63;
        public const byte MPU6050_RA_I2C_SLV1_DO = 0x64;
        public const byte MPU6050_RA_I2C_SLV2_DO = 0x65;
        public const byte MPU6050_RA_I2C_SLV3_DO = 0x66;
        public const byte MPU6050_RA_I2C_MST_DELAY_CTRL = 0x67;
        public const byte MPU6050_RA_SIGNAL_PATH_RESET = 0x68;
        public const byte MPU6050_RA_MOT_DETECT_CTRL = 0x69;
        public const byte MPU6050_RA_USER_CTRL = 0x6A;
        public const byte MPU6050_RA_PWR_MGMT_1 = 0x6B;
        public const byte MPU6050_RA_PWR_MGMT_2 = 0x6C;
        public const byte MPU6050_RA_BANK_SEL = 0x6D;
        public const byte MPU6050_RA_MEM_START_ADDR = 0x6E;
        public const byte MPU6050_RA_MEM_R_W = 0x6F;
        public const byte MPU6050_RA_DMP_CFG_1 = 0x70;
        public const byte MPU6050_RA_DMP_CFG_2 = 0x71;
        public const byte MPU6050_RA_FIFO_COUNTH = 0x72;
        public const byte MPU6050_RA_FIFO_COUNTL = 0x73;
        public const byte MPU6050_RA_FIFO_R_W = 0x74;
        public const byte MPU6050_RA_WHO_AM_I = 0x75;

        public const byte MPU6050_TC_PWR_MODE_BIT = 7;
        public const byte MPU6050_TC_OFFSET_BIT = 6;
        public const byte MPU6050_TC_OFFSET_LENGTH = 6;
        public const byte MPU6050_TC_OTP_BNK_VLD_BIT = 0;

        public const byte MPU6050_VDDIO_LEVEL_VLOGIC = 0;
        public const byte MPU6050_VDDIO_LEVEL_VDD = 1;

        public const byte MPU6050_CFG_EXT_SYNC_SET_BIT = 5;
        public const byte MPU6050_CFG_EXT_SYNC_SET_LENGTH = 3;
        public const byte MPU6050_CFG_DLPF_CFG_BIT = 2;
        public const byte MPU6050_CFG_DLPF_CFG_LENGTH = 3;

        public const byte MPU6050_EXT_SYNC_DISABLED = 0x0;
        public const byte MPU6050_EXT_SYNC_TEMP_OUT_L = 0x1;
        public const byte MPU6050_EXT_SYNC_GYRO_XOUT_L = 0x2;
        public const byte MPU6050_EXT_SYNC_GYRO_YOUT_L = 0x3;
        public const byte MPU6050_EXT_SYNC_GYRO_ZOUT_L = 0x4;
        public const byte MPU6050_EXT_SYNC_ACCEL_XOUT_L = 0x5;
        public const byte MPU6050_EXT_SYNC_ACCEL_YOUT_L = 0x6;
        public const byte MPU6050_EXT_SYNC_ACCEL_ZOUT_L = 0x7;

        public const byte MPU6050_DLPF_BW_256 = 0x00;
        public const byte MPU6050_DLPF_BW_188 = 0x01;
        public const byte MPU6050_DLPF_BW_98 = 0x02;
        public const byte MPU6050_DLPF_BW_42 = 0x03;
        public const byte MPU6050_DLPF_BW_20 = 0x04;
        public const byte MPU6050_DLPF_BW_10 = 0x05;
        public const byte MPU6050_DLPF_BW_5 = 0x06;

        public const byte MPU6050_GCONFIG_FS_SEL_BIT = 4;
        public const byte MPU6050_GCONFIG_FS_SEL_LENGTH = 2;

        public const byte MPU6050_GYRO_FS_250 = 0x00;
        public const byte MPU6050_GYRO_FS_500 = 0x01;
        public const byte MPU6050_GYRO_FS_1000 = 0x02;
        public const byte MPU6050_GYRO_FS_2000 = 0x03;

        public const byte MPU6050_ACONFIG_XA_ST_BIT = 7;
        public const byte MPU6050_ACONFIG_YA_ST_BIT = 6;
        public const byte MPU6050_ACONFIG_ZA_ST_BIT = 5;
        public const byte MPU6050_ACONFIG_AFS_SEL_BIT = 4;
        public const byte MPU6050_ACONFIG_AFS_SEL_LENGTH = 2;
        public const byte MPU6050_ACONFIG_ACCEL_HPF_BIT = 2;
        public const byte MPU6050_ACONFIG_ACCEL_HPF_LENGTH = 3;

        public const byte MPU6050_ACCEL_FS_2 = 0x00;
        public const byte MPU6050_ACCEL_FS_4 = 0x01;
        public const byte MPU6050_ACCEL_FS_8 = 0x02;
        public const byte MPU6050_ACCEL_FS_16 = 0x03;

        public const byte MPU6050_DHPF_RESET = 0x00;
        public const byte MPU6050_DHPF_5 = 0x01;
        public const byte MPU6050_DHPF_2P5 = 0x02;
        public const byte MPU6050_DHPF_1P25 = 0x03;
        public const byte MPU6050_DHPF_0P63 = 0x04;
        public const byte MPU6050_DHPF_HOLD = 0x07;

        public const byte MPU6050_TEMP_FIFO_EN_BIT = 7;
        public const byte MPU6050_XG_FIFO_EN_BIT = 6;
        public const byte MPU6050_YG_FIFO_EN_BIT = 5;
        public const byte MPU6050_ZG_FIFO_EN_BIT = 4;
        public const byte MPU6050_ACCEL_FIFO_EN_BIT = 3;
        public const byte MPU6050_SLV2_FIFO_EN_BIT = 2;
        public const byte MPU6050_SLV1_FIFO_EN_BIT = 1;
        public const byte MPU6050_SLV0_FIFO_EN_BIT = 0;

        public const byte MPU6050_MULT_MST_EN_BIT = 7;
        public const byte MPU6050_WAIT_FOR_ES_BIT = 6;
        public const byte MPU6050_SLV_3_FIFO_EN_BIT = 5;
        public const byte MPU6050_I2C_MST_P_NSR_BIT = 4;
        public const byte MPU6050_I2C_MST_CLK_BIT = 3;
        public const byte MPU6050_I2C_MST_CLK_LENGTH = 4;

        public const byte MPU6050_CLOCK_DIV_348 = 0x0;
        public const byte MPU6050_CLOCK_DIV_333 = 0x1;
        public const byte MPU6050_CLOCK_DIV_320 = 0x2;
        public const byte MPU6050_CLOCK_DIV_308 = 0x3;
        public const byte MPU6050_CLOCK_DIV_296 = 0x4;
        public const byte MPU6050_CLOCK_DIV_286 = 0x5;
        public const byte MPU6050_CLOCK_DIV_276 = 0x6;
        public const byte MPU6050_CLOCK_DIV_267 = 0x7;
        public const byte MPU6050_CLOCK_DIV_258 = 0x8;
        public const byte MPU6050_CLOCK_DIV_500 = 0x9;
        public const byte MPU6050_CLOCK_DIV_471 = 0xA;
        public const byte MPU6050_CLOCK_DIV_444 = 0xB;
        public const byte MPU6050_CLOCK_DIV_421 = 0xC;
        public const byte MPU6050_CLOCK_DIV_400 = 0xD;
        public const byte MPU6050_CLOCK_DIV_381 = 0xE;
        public const byte MPU6050_CLOCK_DIV_364 = 0xF;

        public const byte MPU6050_I2C_SLV_RW_BIT = 7;
        public const byte MPU6050_I2C_SLV_ADDR_BIT = 6;
        public const byte MPU6050_I2C_SLV_ADDR_LENGTH = 7;
        public const byte MPU6050_I2C_SLV_EN_BIT = 7;
        public const byte MPU6050_I2C_SLV_BYTE_SW_BIT = 6;
        public const byte MPU6050_I2C_SLV_REG_DIS_BIT = 5;
        public const byte MPU6050_I2C_SLV_GRP_BIT = 4;
        public const byte MPU6050_I2C_SLV_LEN_BIT = 3;
        public const byte MPU6050_I2C_SLV_LEN_LENGTH = 4;

        public const byte MPU6050_I2C_SLV4_RW_BIT = 7;
        public const byte MPU6050_I2C_SLV4_ADDR_BIT = 6;
        public const byte MPU6050_I2C_SLV4_ADDR_LENGTH = 7;
        public const byte MPU6050_I2C_SLV4_EN_BIT = 7;
        public const byte MPU6050_I2C_SLV4_INT_EN_BIT = 6;
        public const byte MPU6050_I2C_SLV4_REG_DIS_BIT = 5;
        public const byte MPU6050_I2C_SLV4_MST_DLY_BIT = 4;
        public const byte MPU6050_I2C_SLV4_MST_DLY_LENGTH = 5;

        public const byte MPU6050_MST_PASS_THROUGH_BIT = 7;
        public const byte MPU6050_MST_I2C_SLV4_DONE_BIT = 6;
        public const byte MPU6050_MST_I2C_LOST_ARB_BIT = 5;
        public const byte MPU6050_MST_I2C_SLV4_NACK_BIT = 4;
        public const byte MPU6050_MST_I2C_SLV3_NACK_BIT = 3;
        public const byte MPU6050_MST_I2C_SLV2_NACK_BIT = 2;
        public const byte MPU6050_MST_I2C_SLV1_NACK_BIT = 1;
        public const byte MPU6050_MST_I2C_SLV0_NACK_BIT = 0;

        public const byte MPU6050_INTCFG_INT_LEVEL_BIT = 7;
        public const byte MPU6050_INTCFG_INT_OPEN_BIT = 6;
        public const byte MPU6050_INTCFG_LATCH_INT_EN_BIT = 5;
        public const byte MPU6050_INTCFG_INT_RD_CLEAR_BIT = 4;
        public const byte MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT = 3;
        public const byte MPU6050_INTCFG_FSYNC_INT_EN_BIT = 2;
        public const byte MPU6050_INTCFG_I2C_BYPASS_EN_BIT = 1;
        public const byte MPU6050_INTCFG_CLKOUT_EN_BIT = 0;

        public const byte MPU6050_INTMODE_ACTIVEHIGH = 0x00;
        public const byte MPU6050_INTMODE_ACTIVELOW = 0x01;

        public const byte MPU6050_INTDRV_PUSHPULL = 0x00;
        public const byte MPU6050_INTDRV_OPENDRAIN = 0x01;

        public const byte MPU6050_INTLATCH_50USPULSE = 0x00;
        public const byte MPU6050_INTLATCH_WAITCLEAR = 0x01;

        public const byte MPU6050_INTCLEAR_STATUSREAD = 0x00;
        public const byte MPU6050_INTCLEAR_ANYREAD = 0x01;

        public const byte MPU6050_INTERRUPT_FF_BIT = 7;
        public const byte MPU6050_INTERRUPT_MOT_BIT = 6;
        public const byte MPU6050_INTERRUPT_ZMOT_BIT = 5;
        public const byte MPU6050_INTERRUPT_FIFO_OFLOW_BIT = 4;
        public const byte MPU6050_INTERRUPT_I2C_MST_INT_BIT = 3;
        public const byte MPU6050_INTERRUPT_PLL_RDY_INT_BIT = 2;
        public const byte MPU6050_INTERRUPT_DMP_INT_BIT = 1;
        public const byte MPU6050_INTERRUPT_DATA_RDY_BIT = 0;

        // TODO: figure out what these actually do
        // UMPL source code is not very obivous
        public const byte MPU6050_DMPINT_5_BIT = 5;
        public const byte MPU6050_DMPINT_4_BIT = 4;
        public const byte MPU6050_DMPINT_3_BIT = 3;
        public const byte MPU6050_DMPINT_2_BIT = 2;
        public const byte MPU6050_DMPINT_1_BIT = 1;
        public const byte MPU6050_DMPINT_0_BIT = 0;

        public const byte MPU6050_MOTION_MOT_XNEG_BIT = 7;
        public const byte MPU6050_MOTION_MOT_XPOS_BIT = 6;
        public const byte MPU6050_MOTION_MOT_YNEG_BIT = 5;
        public const byte MPU6050_MOTION_MOT_YPOS_BIT = 4;
        public const byte MPU6050_MOTION_MOT_ZNEG_BIT = 3;
        public const byte MPU6050_MOTION_MOT_ZPOS_BIT = 2;
        public const byte MPU6050_MOTION_MOT_ZRMOT_BIT = 0;

        public const byte MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT = 7;
        public const byte MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT = 4;
        public const byte MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT = 3;
        public const byte MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT = 2;
        public const byte MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT = 1;
        public const byte MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT = 0;

        public const byte MPU6050_PATHRESET_GYRO_RESET_BIT = 2;
        public const byte MPU6050_PATHRESET_ACCEL_RESET_BIT = 1;
        public const byte MPU6050_PATHRESET_TEMP_RESET_BIT = 0;

        public const byte MPU6050_DETECT_ACCEL_ON_DELAY_BIT = 5;
        public const byte MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH = 2;
        public const byte MPU6050_DETECT_FF_COUNT_BIT = 3;
        public const byte MPU6050_DETECT_FF_COUNT_LENGTH = 2;
        public const byte MPU6050_DETECT_MOT_COUNT_BIT = 1;
        public const byte MPU6050_DETECT_MOT_COUNT_LENGTH = 2;

        public const byte MPU6050_DETECT_DECREMENT_RESET = 0x0;
        public const byte MPU6050_DETECT_DECREMENT_1 = 0x1;
        public const byte MPU6050_DETECT_DECREMENT_2 = 0x2;
        public const byte MPU6050_DETECT_DECREMENT_4 = 0x3;

        public const byte MPU6050_USERCTRL_DMP_EN_BIT = 7;
        public const byte MPU6050_USERCTRL_FIFO_EN_BIT = 6;
        public const byte MPU6050_USERCTRL_I2C_MST_EN_BIT = 5;
        public const byte MPU6050_USERCTRL_I2C_IF_DIS_BIT = 4;
        public const byte MPU6050_USERCTRL_DMP_RESET_BIT = 3;
        public const byte MPU6050_USERCTRL_FIFO_RESET_BIT = 2;
        public const byte MPU6050_USERCTRL_I2C_MST_RESET_BIT = 1;
        public const byte MPU6050_USERCTRL_SIG_COND_RESET_BIT = 0;

        public const byte MPU6050_PWR1_DEVICE_RESET_BIT = 7;
        public const byte MPU6050_PWR1_SLEEP_BIT = 6;
        public const byte MPU6050_PWR1_CYCLE_BIT = 5;
        public const byte MPU6050_PWR1_TEMP_DIS_BIT = 3;
        public const byte MPU6050_PWR1_CLKSEL_BIT = 2;
        public const byte MPU6050_PWR1_CLKSEL_LENGTH = 3;

        public const byte MPU6050_CLOCK_INTERNAL = 0x00;
        public const byte MPU6050_CLOCK_PLL_XGYRO = 0x01;
        public const byte MPU6050_CLOCK_PLL_YGYRO = 0x02;
        public const byte MPU6050_CLOCK_PLL_ZGYRO = 0x03;
        public const byte MPU6050_CLOCK_PLL_EXT32K = 0x04;
        public const byte MPU6050_CLOCK_PLL_EXT19M = 0x05;
        public const byte MPU6050_CLOCK_KEEP_RESET = 0x07;

        public const byte MPU6050_PWR2_LP_WAKE_CTRL_BIT = 7;
        public const byte MPU6050_PWR2_LP_WAKE_CTRL_LENGTH = 2;
        public const byte MPU6050_PWR2_STBY_XA_BIT = 5;
        public const byte MPU6050_PWR2_STBY_YA_BIT = 4;
        public const byte MPU6050_PWR2_STBY_ZA_BIT = 3;
        public const byte MPU6050_PWR2_STBY_XG_BIT = 2;
        public const byte MPU6050_PWR2_STBY_YG_BIT = 1;
        public const byte MPU6050_PWR2_STBY_ZG_BIT = 0;

        public const byte MPU6050_WAKE_FREQ_1P25 = 0x0;
        public const byte MPU6050_WAKE_FREQ_2P5 = 0x1;
        public const byte MPU6050_WAKE_FREQ_5 = 0x2;
        public const byte MPU6050_WAKE_FREQ_10 = 0x3;

        public const byte MPU6050_BANKSEL_PRFTCH_EN_BIT = 6;
        public const byte MPU6050_BANKSEL_CFG_USER_BANK_BIT = 5;
        public const byte MPU6050_BANKSEL_MEM_SEL_BIT = 4;
        public const byte MPU6050_BANKSEL_MEM_SEL_LENGTH = 5;

        public const byte MPU6050_WHO_AM_I_BIT = 6;
        public const byte MPU6050_WHO_AM_I_LENGTH = 6;

        public const byte MPU6050_DMP_MEMORY_BANKS = 8;
        public const int MPU6050_DMP_MEMORY_BANK_SIZE = 256;
        public const byte MPU6050_DMP_MEMORY_CHUNK_SIZE = 16;
        #endregion
    }


}