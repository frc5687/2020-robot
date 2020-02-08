package org.frc5687.infiniterecharge.robot.util;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;

/*
A lot of this was lifted from:
https://github.com/OliviliK/FTC_Library/blob/master/TCS34725_ColorSensor.java
 */
public class AdafruitColorSensor {
    I2C _device;

    static final byte
    ADDRESS             = 0x29,         // I2C device address
    // ------ CONFIGURATION REGISTERS ------
    // In TCS3472x, the actual register address is
    // in range from 0x80 - 0x9B.  The bit 7 is turned on
    // by the -128 operation,  For example 0x00-128 = 0x80
    ENABLE              = 0x00 - 128,   // Enable Register
    ENABLE_AIEN         = 0x10,         // All Interrupts Enabled
    ENABLE_WEN          = 0x08,         // Wait enable between readings
    ENABLE_ACEN         = 0x02,         // All channels Enabled (
    ENABLE_PON          = 0x01,         // Power ON
    ATIME               = 0x01 - 128,   // ADC Integration Time= 256 - time/2.4 ms
    // Time range= 2.4 - 614.4 ms
    // Register 0x02 is undefined
    WTIME               = 0x03 - 128,   // Wait time between readings depends on Reg 0x0D bit1
    // WLONG= 0, Wait= 256 - time/2.4 ms
    // WLONG= 1, Wait= 256 - time/28.8 ms
    // Low range= 2.4 - 614.4 ms
    // High range= 28.8 - 7,372.8 ms
    AILTL               = 0x04 - 128,   // Clear channel Alarm Interrupt Low Limit Low byte
    AILTH               = 0x05 - 128,   // Low Limit High Byte
    AIHTL               = 0x06 - 128,   // High Limit Low Byte
    AIHTH               = 0x07 - 128,   // High Limit High Byte
    // Register 0x08 - 0x0B are undefined
    PERS                = 0x0C - 128,   // Persistence register. 7:4= PPERS, 3:0= APERS
    // APERS values
    // 0= every cycle interrupt
    // 1-3= Clear channel on every N violations
    // 4-15= Clear channel on every 5*(N-3) violations
    CONFIG              = 0x0D - 128,   // Configure register 7:2= 0, 1:1= WLONG, 0:0= 0
    CONFIG_WLONG        = 0x02,             // Wait time in 12x2.4 (28.8) ms increments
    CONTROL             = 0x0F - 128,   // Control register 7:2= 0 1:0= Amplifier Gain
    CONTROL_AGAIN1      = 0x00,             // Amplifier Gain= 1
    CONTROL_AGAIN4      = 0x01,             // Amplifier Gain= 4
    CONTROL_AGAIN16     = 0x02,             // Amplifier Gain= 16
    CONTROL_AGAIN60     = 0x03,             // Amplifier Gain= 60
    // Registers 0x10 - 0x11 are undefined

    // ------ READ ONLY REGISTERS ------
    DATA_START          = 0x14 - 128,   // Start of first color byte
    DATA_LENGTH         = 0x08,             // 2 bytes per colors: Clear Red, Green, Blue

    ID                  = 0x12 - 128,   // 0x44= TCS34721/25, 0x4D= TCS34723/27
    STATUS              = 0x13 - 128,   // Status 7:5= 0, 4:4= AINT, 3:1= 0, 0:0= AVALID
    STATUS_AINT         = 0x10,             // Clear channel interrupt
    STATUS_AVALID       = 0x01,             // All channels valid (ADC integration complete)
    // RGBC channel values
    CDATAL              = 0x14 - 128,   // Clear Data Low Byte
    CDATAH              = 0x15 - 128,   // Clear Data High Byte
    RDATAL              = 0x16 - 128,   // Red Data Low Byte
    RDATAH              = 0x17 - 128,   // Red Data High Byte
    GDATAL              = 0x18 - 128,   // Green Data Low Byte
    GDATAH              = 0x19 - 128,   // Green Data High Byte
    BDATAL              = 0x1A - 128,   // Blue Data Low Byte
    BDATAH              = 0x1B - 128;   // Blue Data High Byte

    // Cache buffer constant values
    static final byte
    READ_MODE           = 0x00 - 128,   // MSB = 1
    WRITE_MODE          = 0x00;         // MSB = 0

    // Cache buffer index values
    static final int
    CACHE_MODE          = 0,        // MSB = 1 when read mode is active
    DEV_ADDR            = 1,        // Device address
    REG_NUMBER          = 2,        // Register address
    REG_COUNT           = 3,        // Register count
    DATA_OFFSET         = 4,        // First byte of transferred data
    ACTION_FLAG         = 31;       // 0 = idle, -1 = transfer is active

    // --------------------------------- CLASS VARIABLES ----------------------------------------------
    //private ArrayQueue<I2cTransfer> transferQueue;      // Buffer all register writes and
    //private I2cDevice               csDev;              // Color Sensor I2C Device Object
    private byte                    csDevAddr;          // Color Sensor address = 2*0x29
    private byte[]                  rCache;             // Read Cache
    private byte[]                  wCache;             // Write Cache
    //private Lock                    rLock;              // Lock for Read Cache
    //private Lock                    wLock;              // Lock for Write Cache & request queue

    // Read results from the sensor
    private int                     clear       = 0;    // Color clear (a.k.a. alpha)
    private int                     red         = 0;    // Color red
    private int                     green       = 0;    // Color green
    private int                     blue        = 0;    // Color blue
    private int                     aTimeValue  = 0;    // Received ATIME register value
    private int                     controlValue= 0;    // Received CONTROL register value
    private int _idValue = 0;    // Received ID register value

    public AdafruitColorSensor() {
        ByteBuffer buf = ByteBuffer.allocate(256);
        I2C.Port port = I2C.Port.kOnboard;

        _device = new I2C(port, ADDRESS);
        _device.write(ENABLE, (byte)(ENABLE_PON | ENABLE_ACEN)); // Enable power & all colors

        //_device.read(ID, 1, buf);
        //_idValue = buf.getInt();
        checkDeviceID();

        //setGain(16);                        // Set default amplification gain
        //setIntegrationTime(50);             // Set default ADC integration time
    }

    public void setGain(int gain) {
        byte amplifierGain;
        if (gain < 4) {
            amplifierGain = CONTROL_AGAIN1;
        } else if (gain < 16) {
            amplifierGain = CONTROL_AGAIN4;
        } else if (gain < 60) {
            amplifierGain = CONTROL_AGAIN16;
        } else {
            amplifierGain   = CONTROL_AGAIN60;
        }
        _device.write(CONTROL, amplifierGain);
    }

    public void setIntegrationTime(int milliseconds) {
        int count = (int)(milliseconds/2.4);
        if (count < 1) {
            count = 1;          // Clamp the time range
        } else if (count > 256) {
            count = 256;
        }
        _device.write(ATIME, (byte) (256 - count));
    }

    private boolean checkDeviceID() {
        ByteBuffer raw = ByteBuffer.allocate(1);
        if (_device.read(ID, 1, raw)) {
            DriverStation.reportError(">>>>>>>>>>>>>>>>>> Could not find adafruit color sensor", false);
            return false;
        }

        // TODO I DON'T KNOW WHAT TO EXPECT HERE... 0x44 MAYBE?
        if (ID != raw.get()) {
            DriverStation.reportError(">>>>>>>>>>>>>>>>>> Unknown device found with same I2C address as adafruit color sensor", false);
            return false;
        }

        return true;
    }

//    private int colorTemperature() {
//        // Ref: https://en.wikipedia.org/wiki/Color_temperature
//        //      https://en.wikipedia.org/wiki/CIE_1931_color_space
//
//
//
//
//        if (red + green + blue == 0) return 999;    // Prevent divide by zero
//
//        double  r   = red;      // Units or scale is irrelevant, freeze the current values
//        double  g   = green;
//        double  b   = blue;
//        // Convert from RGB to CIE XYZ color space
//        double  x   = (-0.14282 * r) + (1.54924 * g) + (-0.95641 * b);
//        double  y   = (-0.32466 * r) + (1.57837 * g) + (-0.73191 * b);
//        double  z   = (-0.68202 * r) + (0.77073 * g) + ( 0.56332 * b);
//        // Convert to chromatic X and Y coordinates
//        double  xyz = x + y + z;
//
//        double  xC  = x/xyz;
//        double  yC  = y/xyz;
//        // Convert to Correlated Color Temparature (CCT)
//        // Using McCamy's cubic approximation
//        // Based on the inverse slope n at epicenter
//        double  n   = (xC - 0.3320) / (0.1858 - yC);
//        double  CCT =  449.0 *n*n*n + 3525.0 *n*n + 6823.3 * n + 5520.33;
//        // This formula is not working properly with high
//        // green values, clamp the result to 1,000 - 29,999
//        if (CCT > 29999) CCT = 29999;
//        if (CCT < 1000) CCT = 1000;
//        return  (int) CCT;
//    }

    public String getColorRaw() {
        ByteBuffer buf = ByteBuffer.allocate(256);
        _device.read(CDATAL, 8, buf);
        return buf.asCharBuffer().toString();
    }

    public int[] getColor() {
        ByteBuffer buf = ByteBuffer.allocate(256);
        _device.read(CDATAL, 8, buf);
        int[] crgb = new int[4];
        for (int i = 0; i < 4; i++) {
            crgb[i] = (buf.get(2*i) & 0xFF) + (buf.get(2*i+1) & 0xFF) * 256;
        }
        return crgb;
    }

    public int getId() {
        return _idValue;
    }
}
