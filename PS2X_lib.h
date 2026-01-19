// clang-format off
/******************************************************************
*  Super amazing PS2 controller Arduino Library v1.8
*		details and example sketch: 
*			http://www.billporter.info/?p=240
*
*    Original code by Shutter on Arduino Forums
*
*    Revamped, made into lib by and supporting continued development:
*              Bill Porter
*              www.billporter.info
*
*	 Contributers:
*		Eric Wetzel (thewetzel@gmail.com)
*		Kurt Eckhardt
*
*  Lib version history
*    0.1 made into library, added analog stick support. 
*    0.2 fixed config_gamepad miss-spelling
*        added new functions:
*          NewButtonState();
*          NewButtonState(uint16_t);
*          ButtonPressed(uint16_t);
*          ButtonReleased(uint16_t);
*        removed 'PS' from beginning of ever function
*    1.0 found and fixed bug that wasn't configuring controller
*        added ability to define pins
*        added time checking to reconfigure controller if not polled enough
*        Analog sticks and pressures all through 'ps2x.Analog()' function
*        added:
*          enableRumble();
*          enablePressures();
*    1.1  
*        added some debug stuff for end user. Reports if no controller found
*        added auto-increasing sentence delay to see if it helps compatibility.
*    1.2
*        found bad math by Shutter for original clock. Was running at 50kHz, not the required 500kHz. 
*        fixed some of the debug reporting. 
*	1.3 
*	    Changed clock back to 50kHz. CuriousInventor says it's suppose to be 500kHz, but doesn't seem to work for everybody. 
*	1.4
*		Removed redundant functions.
*		Fixed mode check to include two other possible modes the controller could be in.
*       Added debug code enabled by compiler directives. See below to enable debug mode.
*		Added button definitions for shapes as well as colors.
*	1.41
*		Some simple bug fixes
*		Added Keywords.txt file
*	1.5
*		Added proper Guitar Hero compatibility
*		Fixed issue with DEBUG mode, had to send serial at once instead of in bits
*	1.6
*		Changed config_gamepad() call to include rumble and pressures options
*			This was to fix controllers that will only go into config mode once
*			Old methods should still work for backwards compatibility 
*    1.7
*		Integrated Kurt's fixes for the interrupts messing with servo signals
*		Reorganized directory so examples show up in Arduino IDE menu
*    1.8
*		Added Arduino 1.0 compatibility. 
*    1.9
*       Kurt - Added detection and recovery from dropping from analog mode, plus
*       integrated Chipkit (pic32mx...) support
*
*
*
*This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
<http://www.gnu.org/licenses/>
*  
******************************************************************/
// clang-format on

// $$$$$$$$$$$$ DEBUG ENABLE SECTION $$$$$$$$$$$$$$$$
// to debug ps2 controller, uncomment these two lines to print out debug to uart
// #define PS2X_DEBUG
// #define PS2X_COM_DEBUG

#pragma once

#include <Arduino.h>
#include <SPI.h>


class PS2X
{
    /* SPI timing configuration */
    // SPI bitrate (Hz)
    static constexpr uint32_t CTRL_BITRATE{250'000UL};

    // delay duration between SCK high and low
    static constexpr uint16_t CTRL_CLK{5};

    // delay duration between byte reads (uS)
    static constexpr uint16_t CTRL_BYTE_DELAY{18};

    // delay duration between packets (mS) - according to playstation.txt this
    // should be set to 16mS, but it seems that it can go down to 4mS without
    // problems
    static constexpr uint16_t CTRL_PACKET_DELAY{16};

public:
    enum class Type
    {
        Unknown,
        GuitarHero,
        DualShock,
        WirelessDualShock,
        Other
    };

    enum class Button
    {
        Select    = 0x0001,
        L3        = 0x0002,
        R3        = 0x0004,
        Start     = 0x0008,
        Pad_Up    = 0x0010,
        Pad_Right = 0x0020,
        Pad_Down  = 0x0040,
        Pad_Left  = 0x0080,
        L2        = 0x0100,
        R2        = 0x0200,
        L1        = 0x0400,
        R1        = 0x0800,
        Green     = 0x1000,
        Red       = 0x2000,
        Blue      = 0x4000,
        Pink      = 0x8000,
        Triangle  = 0x1000,
        Circle    = 0x2000,
        Cross     = 0x4000,
        Square    = 0x8000,
    };

    enum class AnalogButton
    {
        Stick_Rx  = 5,
        Stick_Ry  = 6,
        Stick_Lx  = 7,
        Stick_Ly  = 8,
        Pad_Right = 9,
        Pad_Up    = 11,
        Pad_Down  = 12,
        Pad_Left  = 10,
        L2        = 19,
        R2        = 20,
        L1        = 17,
        R1        = 18,
        Green     = 13,
        Red       = 14,
        Blue      = 15,
        Pink      = 16,
        Triangle  = 13,
        Circle    = 14,
        Cross     = 15,
        Square    = 16,
    };

    Type readType();

    bool readGamepad(bool motor1 = false, uint8_t motor2 = 0);

    bool isPressed(Button button);

    bool wasAnyToggled();
    bool wasToggled(Button button);
    bool wasPressed(Button button);
    bool wasReleased(Button button);

    uint8_t analog(AnalogButton button);

    void enableRumble();
    bool enablePressures();

    // software SPI
    uint8_t begin(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures = false, bool rumble = false);

#if defined(SPI_HAS_TRANSACTION)
    // explicit hardware SPI
    uint8_t begin(SPIClass* spi, uint8_t att, bool pressures = false, bool rumble = false, bool begin = true);

    // default hardware SPI
    uint8_t begin_spi(uint8_t att, bool pressures = false, bool rumble = false);
    // default hardware SPI with custom pins
    uint8_t begin_spi(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures = false, bool rumble = false);

    // HSPI
    uint8_t begin_hspi(uint8_t att, bool pressures = false, bool rumble = false);
    // HSPI with custom pins
    uint8_t begin_hspi(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures = false, bool rumble = false);

    // VSPI
    uint8_t begin_vspi(uint8_t att, bool pressures = false, bool rumble = false);
    // VSPI with custom pins
    uint8_t begin_vspi(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures = false, bool rumble = false);
#endif

    uint16_t ButtonDataByte();
    void     reconfig_gamepad();

private:
    void CLK_SET();
    void CLK_CLR();
    void CMD_SET();
    void CMD_CLR();
    void ATT_SET();
    void ATT_CLR();
    bool DAT_CHK();

    void BEGIN_SPI_NOATT();
    void END_SPI_NOATT();

    void BEGIN_SPI();
    void END_SPI();

    // common gamepad initialization sequence
    uint8_t config_gamepad_stub(bool pressures, bool rumble);

    uint8_t shiftInOut(char byte);
    void    sendCommandString(const uint8_t* string, uint8_t len);

    uint8_t  PS2data[21];
    uint16_t last_buttons;
    uint16_t buttons;

    // pin I/O configuration, mostly relevant to software SPI support (except ATT
    // which is used in both software and hardware SPI)
    int _clk_pin;
    int _cmd_pin;
    int _att_pin;
    int _dat_pin;

    // SPI configuration
    SPIClass* _spi;    // hardware SPI class (null = software SPI)
#if defined(SPI_HAS_TRANSACTION)
    SPISettings _spi_settings;    // hardware SPI transaction settings
#endif

    uint32_t t_last_att;    // time since last ATT inactive

    uint32_t last_read;
    uint8_t  read_delay;
    uint8_t  controller_type;
    bool     en_Rumble;
    bool     en_Pressures;
};


inline void PS2X::CLK_SET()
{
    digitalWrite(_clk_pin, HIGH);
}

inline void PS2X::CLK_CLR()
{
    digitalWrite(_clk_pin, LOW);
}

inline void PS2X::CMD_SET()
{
    digitalWrite(_cmd_pin, HIGH);
}

inline void PS2X::CMD_CLR()
{
    digitalWrite(_cmd_pin, LOW);
}

inline void PS2X::ATT_SET()
{
    digitalWrite(_att_pin, HIGH);
}

inline void PS2X::ATT_CLR()
{
    digitalWrite(_att_pin, LOW);
}

inline bool PS2X::DAT_CHK()
{
    return digitalRead(_dat_pin) ? true : false;
}

inline void PS2X::BEGIN_SPI_NOATT()
{
    if (_spi != NULL)
    {
        _spi->beginTransaction(_spi_settings);
    }
    else
    {
        CMD_SET();
        CLK_SET();
    }
}

inline void PS2X::BEGIN_SPI()
{
    BEGIN_SPI_NOATT();
    while (millis() - t_last_att < CTRL_PACKET_DELAY)
        ;
    ATT_CLR();    // low enable joystick
    delayMicroseconds(CTRL_BYTE_DELAY);
}

inline void PS2X::END_SPI_NOATT()
{
    if (_spi != NULL)
    {
        _spi->endTransaction();
    }
    else
    {
        CMD_SET();
        CLK_SET();
    }
}

inline void PS2X::END_SPI()
{
    END_SPI_NOATT();
    ATT_SET();
    t_last_att = millis();
}
