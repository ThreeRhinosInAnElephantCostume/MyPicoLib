#pragma once
#include <stdio.h>
#include <locale>
#include <math.h>
#include <string.h>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <functional>
#include <cstring>
#include <unordered_map>

using byte = uint8_t;
using int8 = int8_t;
using uint8 = uint8_t;

using int16 = int16_t;
using uint16 = uint16_t;

using int32 = int32_t;
using uint32 = uint32_t;

using int64 = int64_t;
using uint64 = uint64_t;
namespace PIN
{
    constexpr uint8 LED = PICO_DEFAULT_LED_PIN;
}

#define ArraySize(a) (sizeof(a)/sizeof(a[0]))

#define ARRAYSIZE(a) (sizeof(a)/sizeof(a[0]))

#define EVERY_N(N) static int __EVERY_##N##_TIMES_CTR = 0;\
    __EVERY_##N##_TIMES_CTR = (__EVERY_##N##_TIMES_CTR + 1) % N;\
    if(!__EVERY_##N##_TIMES_CTR)


inline void clear_console()
{
    puts("\033[2J");
}

#ifndef DEBUG
    #define DEBUG 0
#endif
inline void failure(std::string dt="")
{
    printf("\nFAILURE!!!\n");
    if(dt.size() > 0)
        printf((dt + "\n").c_str());
    if constexpr(DEBUG)
    {
        while(true);
    }
    assert(false);
}

template<typename T>
inline T sign(T v)
{
    return (v < 0) ? -1 : 1;
}
#define DebounceRoutine() \
    static uint64 __lasttime = 0;\
    uint64 __tm = to_us_since_boot(get_absolute_time());\
    uint64 __d = __tm - __lasttime;\
    __lasttime = __tm;\
    if (__d < 1000 * 20)\
        return;


inline bool is_pin_unused(uint pin)
{
    return (pin >= (uint8)-1);
}
inline bool is_pin_used(uint pin)
{
    return !is_pin_unused(pin);
}

inline uint64 gettime()
{
    return to_us_since_boot(get_absolute_time());
}
#ifdef _HARDWARE_I2C_H
struct I2C_DEVICE
{
    decltype(i2c0) I2C_PORT;
    uint PIN_SDA;
    uint PIN_SCL;
    uint datarate;
    uint8 address;
    void read_registers(uint8 reg, uint8* buf, uint8 len)
    {
        // for(uint i = 0; i < len; i++)
        // {
        //     buf[i] = read_register(reg+i);
        // }
        i2c_write_blocking(I2C_PORT, address, &reg, 1, true);   
        i2c_read_blocking(I2C_PORT, address, buf, len, false);
    }
    uint8* read_registers(uint8 reg, uint8 len)
    {
        uint8* buf = new uint8[len];
        read_registers(reg, buf, len);
        return buf;
    }
    uint read_register_32(uint8 reg)
    {
        uint dt;
        i2c_write_blocking(I2C_PORT, address, &reg, 1, true);   
        i2c_read_blocking(I2C_PORT, address, (uint8*)&dt, 4, false);
        return dt;
    }
    uint16 read_register_16(uint8 reg)
    {
        uint16 dt;
        i2c_write_blocking(I2C_PORT, address, &reg, 1, true);   
        i2c_read_blocking(I2C_PORT, address, (uint8*)&dt, 2, false);
        return dt;
    }
    uint8 read_register(uint8 reg)
    {
        uint8 dt;
        i2c_write_blocking(I2C_PORT, address, &reg, 1, true);   
        i2c_read_blocking(I2C_PORT, address, &dt, 1, false);
        //i2c_read_raw_blocking(I2C_PORT, &dt, 1);
        return dt;
    }
    void write_registers(uint8 reg, uint8* dt, uint8 len)
    {
        i2c_write_blocking(I2C_PORT, address, &reg, 1, true);   
        i2c_write_blocking(I2C_PORT, address, dt, len, false);   
    }
    void write_register(uint8 reg, uint8* dt, uint8 len = 1)
    {
        write_registers(reg, dt, len);
    }
    void write_register(uint8 reg, uint8 dt)
    {
        uint8 rd[] = {reg, dt};
        i2c_write_blocking(I2C_PORT, address, rd, 2, false); 
    }
    void write_register16(uint8 reg, uint16 dt)
    {
        uint8* dtp = (uint8*)&dt;
        uint8 rd[] = {reg, dtp[0], dtp[1]};
        i2c_write_blocking(I2C_PORT, address, rd, 3, false); 
    }
    void write_register32(uint8 reg, uint32 dt)
    {
        uint8* dtp = (uint8*)&dt;
        uint8 rd[] = {reg, dtp[0], dtp[1], dtp[2], dtp[3]};
        i2c_write_blocking(I2C_PORT, address, rd, 5, false); 
    }
    void write_raw(uint8* dt, uint8 len)
    {
        i2c_write_blocking(I2C_PORT, address, dt, len, false);
    }
    uint8 modify_register(uint8 reg, uint8 val, uint8 mask)
    {
        uint8 cval = read_register(reg);
        cval = val | (cval & ~mask);
        write_register(reg, cval);
        return cval;
    }
    uint8 set_register_bit(uint8 reg, uint8 bit, bool val)
    {
        return modify_register(reg, ((uint8)(!!val) << bit), 1 << bit);
    }
    bool get_register_bit(uint8 reg, uint8 bit)
    {
        return !!(read_register(reg) & (1 << bit));
    }
    void i2cdeviceinit()
    {
        i2c_init(I2C_PORT, datarate);
        gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(PIN_SDA);
        gpio_pull_up(PIN_SCL);
    }   
    I2C_DEVICE(decltype(i2c0) I2C_PORT, uint8 address, uint8 PIN_SDA, uint8 PIN_SCL, uint8 datarate, bool autoinit=true)
    {
        this->PIN_SDA = PIN_SDA;
        this->PIN_SCL = PIN_SCL;
        this->datarate = datarate;
        this->I2C_PORT = I2C_PORT;
        this->address = address;
        if(autoinit)
            i2cdeviceinit();
    }

};
#endif
#ifdef _HARDWARE_SPI_H
#define CS_AUTO() for (uint __cs = cs_select(); __cs; __cs = cs_deselect())
template<uint8 READ_FLAG, uint8 WRITE_FLAG>
struct SPI_DEVICE
{
    spi_inst_t* SPI_PORT;

    uint PIN_MISO;
    uint PIN_MOSI;
    uint PIN_SCK;
    uint PIN_CS;

    uint datarate;


    uint cs_select()
    {
        gpio_put(PIN_CS, false);
        return 1;
    }
    uint cs_deselect()
    {
        gpio_put(PIN_CS, true);
        return 0;
    }
    uint8 read_register(uint8 reg)
    {
        reg |= READ_FLAG;
        uint8 out = 0xFF;
        CS_AUTO()
        {
            spi_write_blocking(SPI_PORT, &reg, 1);
            spi_read_blocking(SPI_PORT, 0, &out, 1);
        }
        return out;
    }
    void read_register(uint8 reg, byte* data, uint8 len)
    {
        reg |= READ_FLAG;
        CS_AUTO()
        {
            spi_write_blocking(SPI_PORT, &reg, 1);
            spi_read_blocking(SPI_PORT, 0, data, len);
        }
    }
    uint8 write_register_with_output(uint8 reg, uint8 val)
    {
        reg |= WRITE_FLAG;
        uint8 out;
        CS_AUTO()
        {

            spi_write_read_blocking(SPI_PORT, &reg, &out, 1);
            spi_write_blocking(SPI_PORT, &val, 1);
        }
        return out;
    }
    void write_register(uint8 reg, uint8 val)
    {
        reg |= WRITE_FLAG;
        uint8 out;
        CS_AUTO()
        {

            spi_write_read_blocking(SPI_PORT, &reg, &out, 1);
            volatile uint8 _out = out;
            spi_write_blocking(SPI_PORT, &val, 1);
            printf("0:    %i\n", out);
        }
    }
    void write_register(uint8 reg, byte* val, uint8 len)
    {
        reg |= WRITE_FLAG;
        CS_AUTO()
        {
            spi_write_blocking(SPI_PORT, &reg, 1);
            spi_write_blocking(SPI_PORT, (uint8*)val, len);
        }
    }
    void write_register(uint8 reg, uint8 val, uint mask)
    {
        write_register(reg, (read_register(reg) & ~mask) | val);
    }
    void raw_write(uint8 val)
    {
        spi_write_blocking(SPI_PORT, &val, 1);
    }
    void raw_write(uint8* val, uint8 n)
    {
        spi_write_blocking(SPI_PORT, val, n);
    }
    uint8 raw_read()
    {
        uint8 out;
        spi_read_blocking(SPI_PORT, 0, &out, 1);
        return out;
    }
    void raw_read(uint8* buf, uint8 n)
    {
        spi_read_blocking(SPI_PORT, 0, buf, n);
    }
    uint8 raw_read_write(uint8 v)
    {
        uint8 o;
        spi_write_read_blocking(SPI_PORT, &v, &o, 1);
        return o;
    }
    void spidevinit()
    {

        spi_init(SPI_PORT, datarate);
        gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
        gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
        gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

        gpio_init(PIN_CS);
        gpio_set_dir(PIN_CS, GPIO_OUT);
        gpio_put(PIN_CS, 1);
    }
    SPI_DEVICE(spi_inst_t* SPI_PORT, uint PIN_MISO, uint PIN_MOSI, uint PIN_SCK, uint PIN_CS, uint datarate, bool autoinit = true)
    {
        this->SPI_PORT = SPI_PORT;
        this->PIN_MISO = PIN_MISO;
        this->PIN_MOSI = PIN_MOSI;
        this->PIN_SCK = PIN_SCK;
        this->PIN_CS = PIN_CS;
        this->datarate = datarate;
        if (autoinit)
            spidevinit();
    }
};


#endif
inline void init_in(uint pin, bool pullup = false, bool pulldown = false)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_pulls(pin, pullup, pulldown);
}
inline void init_in(std::vector<uint> pins, bool pullup = false, bool pulldown = false)
{
    for (auto it : pins)
    {
        init_in(it, pullup, pulldown);
    }
}
inline void init_in(uint pin, int pulldir)
{
    init_in(pin, (pulldir > 0), (pulldir < 0));
}
inline void init_in(std::vector<uint> pin, int pulldir)
{
    init_in(pin, (pulldir > 0), (pulldir < 0));
}
inline void init_out(uint pin, bool state)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, state);
}
inline void init_out(std::vector<uint> pins, bool state)
{
    for (auto it : pins)
    {
        init_out(it, state);
    }
}
inline uint pin_from_adc(uint adc)
{
    if (adc > 3 || adc < 0)
        return -1;
    return adc + 26;
}
inline uint adc_from_pin(uint pin)
{
    if (pin > 29 || pin < 26)
        return -1;
    return pin - 26;
}
inline void init_adc_pin(uint adc)
{
    static bool adc_inited = false;
    if (!adc_inited)
    {
        adc_init();
        adc_inited = true;
    }
    adc_gpio_init(pin_from_adc(adc));
    adc_select_input(adc);
}
inline float adc_read_voltage(uint8 adc, float multip = 1.0)
{
    adc_select_input(adc);
    uint16 rv = abs(((int)adc_read())-30);

    return (((float)rv)/4096.0f) * 3.3f * multip;
}
template <typename T, int capacity>
class FIFO
{
protected:
    T data[capacity];
    int top = 0;
    int sz = 0;

public:
    int getcapacity()
    {
        return capacity;
    }
    struct iter : public std::iterator<std::input_iterator_tag, FIFO, FIFO, const FIFO*, FIFO&>
    {
        FIFO<T, capacity>* fifo;
        int indx;
        bool operator==(iter& other) const
        {
            return this->indx == other.indx;
        }
        bool operator!=(iter& other) const
        {
            return this->indx != other.indx;
        }
        iter& operator++()
        {
            indx++;
            return *this;
        }
        iter operator++(int)
        {
            auto prev = iter(indx, fifo);
            indx++;
            return prev;
        }
        explicit iter(int indx, FIFO<T, capacity>* f)
        {
            this->fifo = f;
            this->indx = indx;
        }
        T& operator*() const
        {
            return (*fifo)[indx];
        }
    };
    inline iter begin()
    {
        return iter(0, this);
    }
    inline iter end()
    {
        return iter((size()), this);
    }
    inline int size()
    {
        return sz;
    }
    int freesz()
    {
        return capacity - size();
    }
    T& get(int indx)
    {
        return data[(top + indx) % capacity];
    }
    T& operator[](int indx)
    {
        return get(indx);
    }
    void popall()
    {
        sz = 0;
    }
    void popn(int n)
    {
        top += n;
        top %= capacity;
        sz -= n;
    }
    T pop()
    {
        T val = get(0);
        top++;
        sz--;
        top %= capacity;
        return val;
    }
    bool push(T val)
    {
        if (sz == capacity)
        {
            data[top] = val;
            top++;
            top %= capacity;
            return false;
        }
        int indx = (top + sz) % capacity;
        data[indx] = val;
        sz++;
        return true;
    }
    FIFO()
    {
    }
    FIFO(const FIFO&) = delete;
};
#ifndef _NO_CUSTOM_INTERRUPT
namespace interrupt
{
    inline std::function<void(uint, uint)> __interrupts[32];
    static void interrupt_handler(uint gpio, uint32 events)
    {
        __interrupts[gpio](gpio, events);
    }
    inline void set_interrupt(uint gpio, std::function<void(uint, uint)> f, uint mode = GPIO_IRQ_EDGE_RISE)
    {
        gpio_set_irq_enabled(gpio, mode, true);
        __interrupts[gpio] = f;
    }
    inline void set_interrupt(uint gpio, std::function<void()> f, uint mode = GPIO_IRQ_EDGE_RISE)
    {
        set_interrupt(gpio, [&](uint, uint) {f();}, mode);
    }
    inline void init_interrupts()
    {
        gpio_set_irq_enabled_with_callback(0, GPIO_IRQ_EDGE_RISE, true, interrupt_handler);
        gpio_set_irq_enabled(0, GPIO_IRQ_EDGE_RISE, false);
    }
}
#endif

template<typename T> 
inline T ComputeR1(T R2, T Vin, T Vout)
{
    return ((R2 * Vin)/Vin) - R2;
}
template<typename T> 
inline T ComputeR2(T R1, T Vin, T Vout)
{
    return R1 * (1 / ((Vin/Vout)-1));
}

struct HardwareFailureException : std::exception
{
    std::string msg = "";
    HardwareFailureException() _GLIBCXX_NOTHROW 
    { 

    }
    HardwareFailureException(std::string msg) 
    { 
        this->msg = msg;
    }
    virtual const char* what() const _GLIBCXX_TXN_SAFE_DYN _GLIBCXX_NOTHROW
    {
        return msg.c_str();
    }
};