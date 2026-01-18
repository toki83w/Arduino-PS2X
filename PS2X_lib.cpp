#include "PS2X_lib.h"
#include <math.h>

#define SET(x, y) (x |= (1 << y))
#define CLR(x, y) (x &= (~(1 << y)))
#define CHK(x, y) (x & (1 << y))
#define TOG(x, y) (x ^= (1 << y))
#define U16C(x)   (static_cast<uint16_t>(x))

namespace
{
    constexpr uint8_t enter_config[]    = {0x01, 0x43, 0x00, 0x01, 0x00};
    constexpr uint8_t set_mode[]        = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    constexpr uint8_t set_bytes_large[] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
    constexpr uint8_t exit_config[]     = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    constexpr uint8_t enable_rumble[]   = {0x01, 0x4D, 0x00, 0x00, 0x01};
    constexpr uint8_t type_read[]       = {0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
}    // namespace

bool PS2X::wasAnyToggled()
{
    return ((last_buttons ^ buttons) > 0);
}

bool PS2X::wasToggled(Button button)
{
    return ((last_buttons ^ buttons) & U16C(button)) > 0;
}

bool PS2X::wasPressed(Button button)
{
    return wasToggled(button) && isPressed(button);
}

bool PS2X::wasReleased(Button button)
{
    return wasToggled(button) && ((~last_buttons & U16C(button)) > 0);
}

bool PS2X::isPressed(Button button)
{
    return (~buttons & U16C(button)) > 0;
}

uint16_t PS2X::ButtonDataByte()
{
    return ~buttons;
}

uint8_t PS2X::analog(AnalogButton button)
{
    return PS2data[U16C(button)];
}

uint8_t PS2X::shiftInOut(char byte)
{
    if (_spi == NULL)
    {
        /* software SPI */
        uint8_t tmp = 0;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (CHK(byte, i))
                CMD_SET();
            else
                CMD_CLR();

            CLK_CLR();
            delayMicroseconds(CTRL_CLK);

            //if(DAT_CHK()) SET(tmp,i);
            if (DAT_CHK())
                bitSet(tmp, i);

            CLK_SET();
            delayMicroseconds(CTRL_CLK);
        }
        CMD_SET();
        delayMicroseconds(CTRL_BYTE_DELAY);
        return tmp;
    }
    else
    {
        uint8_t tmp = _spi->transfer(byte);    // hardware SPI
        delayMicroseconds(CTRL_BYTE_DELAY);
        return tmp;
    }
}

bool PS2X::readGamepad(bool motor1, uint8_t motor2)
{
    double temp = millis() - last_read;

    if (temp > 1500)    //waited to long
        reconfig_gamepad();

    if (temp < read_delay)    //waited too short
        delay(read_delay - temp);

    if (motor2 != 0x00)
        motor2 = map(motor2, 0, 255, 0x40, 0xFF);    //noting below 40 will make it spin

    uint8_t dword[9]   = {0x01, 0x42, 0, motor1, motor2, 0, 0, 0, 0};
    uint8_t dword2[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Try a few times to get valid data...
    for (uint8_t RetryCnt = 0; RetryCnt < 5; RetryCnt++)
    {
        BEGIN_SPI();
        //Send the command to send button and joystick data;
        for (int i = 0; i < 9; i++)
        {
            PS2data[i] = shiftInOut(dword[i]);
        }

        if (PS2data[1] == 0x79)
        {    //if controller is in full data return mode, get the rest of data
            for (int i = 0; i < 12; i++)
            {
                PS2data[i + 9] = shiftInOut(dword2[i]);
            }
        }

        END_SPI();

        // Check to see if we received valid data or not.
        // We should be in analog mode for our data to be valid (analog == 0x7_)
        if ((PS2data[1] & 0xf0) == 0x70)
            break;

        // If we got to here, we are not in analog mode, try to recover...
        reconfig_gamepad();    // try to get back into Analog mode.
        delay(read_delay);
    }

    // If we get here and still not in analog mode (=0x7_), try increasing the read_delay...
    if ((PS2data[1] & 0xf0) != 0x70)
    {
        if (read_delay < 10)
            read_delay++;    // see if this helps out...
    }

#ifdef PS2X_COM_DEBUG
    Serial.print("OUT:IN ");
    for (int i = 0; i < 9; i++)
    {
        Serial.print(dword[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i], HEX);
        Serial.print(" ");
    }
    for (int i = 0; i < 12; i++)
    {
        Serial.print(dword2[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i + 9], HEX);
        Serial.print(" ");
    }
    Serial.println("");
#endif

    last_buttons = buttons;    //store the previous buttons states

    buttons   = (uint16_t) (PS2data[4] << 8) + PS2data[3];    //store as one value for multiple functions
    last_read = millis();
    return ((PS2data[1] & 0xf0) == 0x70);    // 1 = OK = analog mode - 0 = NOK
}

uint8_t PS2X::begin(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble)
{
    _clk_pin = clk;
    _cmd_pin = cmd;
    _att_pin = att;
    _dat_pin = dat;

    pinMode(clk, OUTPUT);    //configure ports
    pinMode(att, OUTPUT);
    ATT_SET();
    pinMode(cmd, OUTPUT);
    pinMode(dat, INPUT_PULLUP);    // enable pull-up

    CLK_SET();

    return config_gamepad_stub(pressures, rumble);
}

#if defined(SPI_HAS_TRANSACTION)
uint8_t PS2X::begin(SPIClass* spi, uint8_t att, bool pressures, bool rumble, bool begin)
{
    _spi     = spi;
    _att_pin = att;

    pinMode(att, OUTPUT);
    ATT_SET();

    _spi_settings = SPISettings(CTRL_BITRATE, LSBFIRST, SPI_MODE0);

    if (begin)
        _spi->begin();    // begin SPI with default settings

    /* some hardware SPI implementations incorrectly hold CLK low before the first transaction, so we'll try to fix that */
    BEGIN_SPI_NOATT();
    _spi->transfer(0x55);    // anything will work here
    END_SPI_NOATT();

    return config_gamepad_stub(pressures, rumble);
}
#endif

uint8_t PS2X::config_gamepad_stub(bool pressures, bool rumble)
{
    uint8_t temp[sizeof(type_read)];


    //new error checking. First, read gamepad a few times to see if it's talking
    readGamepad();
    readGamepad();

    //see if it talked - see if mode came back.
    //If still anything but 41, 73 or 79, then it's not talking
    if (PS2data[1] != 0x41 && PS2data[1] != 0x42 && PS2data[1] != 0x73 && PS2data[1] != 0x79)
    {
#ifdef PS2X_DEBUG
        Serial.println("Controller mode not matched or no controller found");
        Serial.print("Expected 0x41, 0x42, 0x73 or 0x79, but got ");
        Serial.println(PS2data[1], HEX);
#endif
        return 1;    //return error code 1
    }

    //try setting mode, increasing delays if need be.
    read_delay = 1;

    t_last_att = millis() + CTRL_PACKET_DELAY;    // start reading right away

    for (int y = 0; y <= 10; y++)
    {
        sendCommandString(enter_config, sizeof(enter_config));    //start config run

        //read type
        delayMicroseconds(CTRL_BYTE_DELAY);

        //CLK_SET(); // CLK should've been set to HIGH already
        BEGIN_SPI();


        for (int i = 0; i < 9; i++)
        {
            temp[i] = shiftInOut(type_read[i]);
        }

        END_SPI();


        controller_type = temp[3];

        sendCommandString(set_mode, sizeof(set_mode));
        if (rumble)
        {
            sendCommandString(enable_rumble, sizeof(enable_rumble));
            en_Rumble = true;
        }
        if (pressures)
        {
            sendCommandString(set_bytes_large, sizeof(set_bytes_large));
            en_Pressures = true;
        }
        sendCommandString(exit_config, sizeof(exit_config));

        readGamepad();

        if (pressures)
        {
            if (PS2data[1] == 0x79)
                break;
            if (PS2data[1] == 0x73)
                return 3;
        }

        if (PS2data[1] == 0x73)
            break;

        if (y == 10)
        {
#ifdef PS2X_DEBUG
            Serial.println("Controller not accepting commands");
            Serial.print("mode still set at");
            Serial.println(PS2data[1], HEX);
#endif
            return 2;    //exit function with error
        }
        read_delay += 1;    //add 1ms to read_delay
    }
    return 0;    //no error if here
}

void PS2X::sendCommandString(const uint8_t* string, uint8_t len)
{
#ifdef PS2X_COM_DEBUG
    uint8_t temp[len];
    BEGIN_SPI();

    for (int y = 0; y < len; y++)
        temp[y] = shiftInOut(string[y]);

    END_SPI();

    delay(read_delay);    //wait a few

    Serial.println("OUT:IN Configure");
    for (int i = 0; i < len; i++)
    {
        Serial.print(string[i], HEX);
        Serial.print(":");
        Serial.print(temp[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
#else
    BEGIN_SPI();
    for (int y = 0; y < len; y++)
        shiftInOut(string[y]);
    END_SPI();

    delay(read_delay);    //wait a few
#endif
}

PS2X::Type PS2X::readType()
{
    Serial.print("Controller_type: ");
    Serial.println(controller_type, HEX);

    if (controller_type == 0x03)
        return Type::DualShock;
    else if (controller_type == 0x01 && PS2data[1] == 0x42)
        return Type::Other;
    else if (controller_type == 0x01 && PS2data[1] != 0x42)
        return Type::GuitarHero;
    else if (controller_type == 0x0C)
        return Type::WirelessDualShock;

    return Type::Unknown;
}

void PS2X::enableRumble()
{
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(enable_rumble, sizeof(enable_rumble));
    sendCommandString(exit_config, sizeof(exit_config));
    en_Rumble = true;
}

bool PS2X::enablePressures()
{
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));

    readGamepad();
    readGamepad();

    if (PS2data[1] != 0x79)
        return false;

    en_Pressures = true;
    return true;
}

void PS2X::reconfig_gamepad()
{
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_mode, sizeof(set_mode));
    if (en_Rumble)
        sendCommandString(enable_rumble, sizeof(enable_rumble));
    if (en_Pressures)
        sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));
}

#if defined(SPI_HAS_TRANSACTION)
uint8_t PS2X::begin_spi(uint8_t att, bool pressures, bool rumble)
{
    return begin(&SPI, att, pressures, rumble);
}

uint8_t PS2X::begin_spi(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble)
{
    SPIClass* spi_class = &SPI;
    spi_class->begin(clk, dat, cmd, att);
    return begin(spi_class, att, pressures, rumble, false);
}

#    if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#        define VSPI FSPI
#    endif

uint8_t PS2X::begin_hspi(uint8_t att, bool pressures, bool rumble)
{
    return begin(new SPIClass(HSPI), att, pressures, rumble);
}

uint8_t PS2X::begin_hspi(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble)
{
    SPIClass* spi_class = new SPIClass(HSPI);
    spi_class->begin(clk, dat, cmd, att);
    return begin(spi_class, att, pressures, rumble, false);
}

uint8_t PS2X::begin_vspi(uint8_t att, bool pressures, bool rumble)
{
    return begin(new SPIClass(VSPI), att, pressures, rumble);
}

uint8_t PS2X::begin_vspi(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble)
{
    SPIClass* spi_class = new SPIClass(VSPI);
    spi_class->begin(clk, dat, cmd, att);
    return begin(spi_class, att, pressures, rumble, false);
}
#endif
