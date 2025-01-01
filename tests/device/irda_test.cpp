#include <gtest/gtest.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/char.h>
#include <86box/serial.h>
#include <86box/irda.h>
#include <86box/random.h>
}

// The ESI-9680 dongle and the IrDA OBEX device, as a COM port's device through
// the real serial host layer and chardev backend. Only the UART is mocked, as
// a chip that owns an endpoint in com_ports[]: it transmits, polls for data and
// reads the modem status through its backend, as the real chips do. The dongle's
// timer is stepped by hand. Frames are built and decoded here independently of
// the device code.

namespace {

constexpr uint8_t BOF  = 0xc0;
constexpr uint8_t EOF_ = 0xc1;
constexpr uint8_t CE   = 0x7d;
constexpr uint8_t XBOF = 0xff;

constexpr double PERIOD_9600   = 1000000.0 / 960.0;
constexpr double PERIOD_19200  = 1000000.0 / 1920.0;
constexpr double PERIOD_115200 = 1000000.0 / 11520.0;

serial_endpoint_t chip;

pc_timer_t             *dongle_timer;
double                  timer_period;
bool                    timer_running;
std::vector<uint8_t>    random_bytes;
size_t                  random_pos;
char_port_t            *active_port;

struct added_t {
    const device_t *dev;
    void           *priv;
    int             inst;
};
std::vector<added_t> added;
int                  current_inst;

const serial_port_ops_t chip_ops = {};

uint16_t
fcs_byte(uint16_t fcs, uint8_t byte)
{
    fcs ^= byte;
    for (int i = 0; i < 8; i++)
        fcs = (fcs & 1) ? ((fcs >> 1) ^ 0x8408) : (fcs >> 1);
    return fcs;
}

uint16_t
fcs_of(const std::vector<uint8_t> &data)
{
    uint16_t fcs = 0xffff;
    for (uint8_t byte : data)
        fcs = fcs_byte(fcs, byte);
    return static_cast<uint16_t>(~fcs);
}

void
put_escaped(std::vector<uint8_t> &out, uint8_t byte)
{
    if ((byte == BOF) || (byte == EOF_) || (byte == CE)) {
        out.push_back(CE);
        byte ^= 0x20;
    }
    out.push_back(byte);
}

std::vector<uint8_t>
wrap(const std::vector<uint8_t> &frame, uint16_t fcs_xor = 0)
{
    std::vector<uint8_t> out(10, XBOF);
    uint16_t             fcs = fcs_of(frame) ^ fcs_xor;

    out.push_back(BOF);
    for (uint8_t byte : frame)
        put_escaped(out, byte);
    put_escaped(out, fcs & 0xff);
    put_escaped(out, fcs >> 8);
    out.push_back(EOF_);
    return out;
}

void
put_le32(std::vector<uint8_t> &out, uint32_t val)
{
    for (int i = 0; i < 4; i++)
        out.push_back((val >> (i * 8)) & 0xff);
}

std::vector<uint8_t>
xid_cmd(uint32_t source, uint32_t dest = 0xffffffff, uint8_t slot = 0)
{
    std::vector<uint8_t> frame = { 0xff, 0x3f, 0x01 };
    put_le32(frame, source);
    put_le32(frame, dest);
    frame.push_back(0x02); /* 8 slots. */
    frame.push_back(slot);
    frame.push_back(0x00);
    return frame;
}

uint32_t
get_le32(const std::vector<uint8_t> &data, size_t pos)
{
    return data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16) | (static_cast<uint32_t>(data[pos + 3]) << 24);
}

/* The frames in a received stream, without their FCS; bad ones are not returned. */
std::vector<std::vector<uint8_t>>
unwrap(const std::vector<uint8_t> &stream)
{
    std::vector<std::vector<uint8_t>> frames;
    std::vector<uint8_t>              cur;
    bool                              in_frame = false;
    bool                              escape   = false;

    for (uint8_t byte : stream) {
        if (byte == BOF) {
            in_frame = true;
            escape   = false;
            cur.clear();
        } else if (byte == EOF_) {
            uint16_t fcs = 0xffff;
            for (uint8_t b : cur)
                fcs = fcs_byte(fcs, b);
            if (in_frame && (cur.size() >= 2) && (fcs == 0xf0b8)) {
                cur.resize(cur.size() - 2);
                frames.push_back(cur);
            }
            in_frame = false;
        } else if (byte == CE) {
            escape = true;
        } else if (in_frame) {
            cur.push_back(escape ? (byte ^ 0x20) : byte);
            escape = false;
        }
    }
    return frames;
}

} // namespace

extern "C" {

uint64_t tsc;
uint64_t TIMER_USEC = 1;

void pclog(const char *, ...) { }
void pclog_ex(const char *, va_list) { }

void
timer_enable(pc_timer_t *timer)
{
    timer->flags |= TIMER_ENABLED;
}

void
timer_disable(pc_timer_t *timer)
{
    timer->flags &= ~TIMER_ENABLED;
    timer_running = false;
}

void
timer_add(pc_timer_t *timer, void (*callback)(void *priv), void *priv, int)
{
    memset(timer, 0, sizeof(*timer));
    timer->callback = callback;
    timer->priv     = priv;
    dongle_timer    = timer;
}

void
timer_on_auto(pc_timer_t *timer, double period)
{
    timer_period  = period;
    timer_running = true;
    timer_enable(timer);
}

void
timer_stop(pc_timer_t *timer)
{
    timer_disable(timer);
}

uint8_t
random_generate(void)
{
    return random_bytes[random_pos++ % random_bytes.size()];
}

int
device_get_instance(void)
{
    return current_inst;
}

void *
device_add_inst(const device_t *dev, int inst)
{
    int const    saved = current_inst;
    size_t const slot  = added.size();

    /* As the real one, the slot is taken before the device's init runs. */
    added.push_back({ dev, nullptr, inst });

    current_inst = inst;
    void *priv   = dev->init(dev);
    current_inst = saved;

    added[slot].priv = priv;
    return priv;
}

/* As the real one: the newest first, and only what matches the flags. */
void
device_close_by_flags_inst(uint32_t flags, int inst)
{
    for (size_t i = added.size(); i-- > 0;) {
        if (((added[i].dev->flags & flags) == flags) && ((inst == -1) || (added[i].inst == inst))) {
            added[i].dev->close(added[i].priv);
            added.erase(added.begin() + i);
        }
    }
}

void *
char_init(char_port_t *port, const device_t *device, int instance)
{
    active_port = port;
    void *priv  = device_add_inst(device, instance);
    active_port = nullptr;
    return priv;
}

char_port_t *
char_attach(uint32_t flags, size_t (*read)(uint8_t *, size_t, void *), size_t (*write)(uint8_t *, size_t, void *),
            uint32_t (*status)(void *), void (*control)(uint32_t, void *), void (*port_config)(void *), void *priv)
{
    active_port->chardev = { flags, priv, read, write, status, control, port_config };
    return active_port;
}

void char_update_status(char_port_t *) { }
const device_t *char_get_device(int id) { return id ? &esi9680_device : nullptr; }
void *char_log_open(char_port_t *, char *) { return nullptr; }
void *log_open(const char *) { return nullptr; }
void log_close(void *) { }
}

class IrdaTest : public testing::Test {
protected:
    void SetUp() override
    {
        serial_devices_close(0);
        memset(com_ports, 0, sizeof(com_ports));
        serial_endpoint_init(&chip, &chip, &chip_ops);
        com_ports[0].ep = &chip;

        irda_reset();
        dongle_timer = nullptr;
        timer_period = 0.0;
        random_bytes = { 0x11, 0x22, 0x33, 0x44 };
        random_pos   = 0;
        added.clear();
        rx.clear();
    }

    void TearDown() override
    {
        /* As device_close_all() does. */
        device_close_by_flags_inst(0, -1);
    }

    void open_dongle()
    {
        com_ports[0].device = 1;
        serial_devices_init();
        priv = added[0].priv;
    }

    void control(uint32_t flags) { chip.be->control(chip.be->priv, flags); }
    void power_on(uint32_t lines = CHAR_COM_RTS) { control(lines); }

    uint32_t status() { return chip.be->status(chip.be->priv); }
    bool     lines_up() { return status() == (CHAR_COM_CTS | CHAR_COM_DSR | CHAR_COM_DCD); }

    void send(const std::vector<uint8_t> &bytes)
    {
        for (uint8_t byte : bytes)
            chip.be->write_byte(chip.be->priv, byte);
    }

    /* The UART's receive timer. */
    int poll() { return chip.be->read_byte(chip.be->priv); }

    void tick()
    {
        timer_disable(dongle_timer);
        dongle_timer->callback(dongle_timer->priv);
    }

    /* Let the dongle hand over one character per timer tick until it has no more. */
    void drain(int max = 1000)
    {
        while ((max-- > 0) && timer_is_enabled(dongle_timer)) {
            tick();
            int const val = poll();
            if (val >= 0)
                rx.push_back(static_cast<uint8_t>(val));
        }
    }

    std::vector<std::vector<uint8_t>> discover(uint32_t source = 0x01020304,
                                               uint32_t dest = 0xffffffff, uint8_t slot = 0)
    {
        rx.clear();
        send(wrap(xid_cmd(source, dest, slot)));
        drain();
        return unwrap(rx);
    }

    void                *priv = nullptr;
    std::vector<uint8_t> rx;
};

TEST(IrdaFcs, MatchesTheX25CheckValue)
{
    /* The helper the tests rely on is itself checked against the published value. */
    EXPECT_EQ(fcs_of({ '1', '2', '3', '4', '5', '6', '7', '8', '9' }), 0x906e);
}

TEST_F(IrdaTest, StaysOffUntilPowered)
{
    open_dongle();

    EXPECT_EQ(status(), 0u);
    EXPECT_FALSE(timer_is_enabled(dongle_timer));
}

TEST_F(IrdaTest, LinesSelectTheSpeed)
{
    struct {
        uint32_t lines;
        double   period;
    } const speeds[] = {
        { CHAR_COM_RTS, PERIOD_9600 },
        { CHAR_COM_DTR, PERIOD_19200 },
        { CHAR_COM_DTR | CHAR_COM_RTS, PERIOD_115200 },
    };

    open_dongle();

    for (const auto &speed : speeds) {
        power_on(speed.lines);
        EXPECT_TRUE(lines_up());

        ASSERT_EQ(discover().size(), 1u);
        EXPECT_NEAR(timer_period, speed.period, 0.001);
    }

    control(0);
    EXPECT_EQ(status(), 0u);
}

TEST_F(IrdaTest, BreakAloneDoesNotDisturbDelivery)
{
    open_dongle();
    power_on();

    send(wrap(xid_cmd(1)));
    double const before = timer_period;

    control(CHAR_COM_RTS | CHAR_COM_BREAK);
    EXPECT_TRUE(timer_is_enabled(dongle_timer));
    EXPECT_EQ(timer_period, before);
}

TEST_F(IrdaTest, AnswersDiscovery)
{
    open_dongle();
    power_on();

    auto const frames = discover(0xaabbccdd);
    ASSERT_EQ(frames.size(), 1u);

    auto const &frame = frames[0];
    ASSERT_EQ(frame.size(), 14u + 12u);

    EXPECT_EQ(frame[0], 0xfe);                       /* Broadcast, response. */
    EXPECT_EQ(frame[1], 0xbf);                       /* XID response, final. */
    EXPECT_EQ(frame[2], 0x01);                       /* Format. */
    EXPECT_EQ(get_le32(frame, 3), 0x44332211u);      /* Our address. */
    EXPECT_EQ(get_le32(frame, 7), 0xaabbccddu);      /* The primary's. */
    EXPECT_EQ(frame[11], 0x02);                      /* Slots, as asked. */
    EXPECT_EQ(frame[12], 0x00);                      /* Slot. */
    EXPECT_EQ(frame[13], 0x00);                      /* Version. */
    EXPECT_EQ(frame[14], 0x84);                      /* Computer, extended. */
    EXPECT_EQ(frame[15], 0x20);                      /* OBEX. */
    EXPECT_EQ(frame[16], 0x00);                      /* ASCII. */
    EXPECT_EQ(std::string(frame.begin() + 17, frame.end()), "IrDA OBEX");
}

TEST_F(IrdaTest, StartsTheResponseWithExtraBofs)
{
    open_dongle();
    power_on();

    discover();
    ASSERT_GE(rx.size(), 11u);
    for (int i = 0; i < 10; i++)
        EXPECT_EQ(rx[i], XBOF);
    EXPECT_EQ(rx[10], BOF);
}

TEST_F(IrdaTest, AddressIsStableAcrossDiscoveries)
{
    open_dongle();
    power_on();

    auto const first = discover();
    ASSERT_EQ(first.size(), 1u);
    uint32_t const address = get_le32(first[0], 3);

    /* A directed XID only works if the address has not changed. */
    auto const second = discover(0x01020304, address);
    ASSERT_EQ(second.size(), 1u);
    EXPECT_EQ(get_le32(second[0], 3), address);
}

TEST_F(IrdaTest, NeverUsesTheBroadcastOrNullAddress)
{
    random_bytes = { 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78 };

    open_dongle();
    power_on();

    auto const frames = discover();
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(get_le32(frames[0], 3), 0x78563412u);
}

TEST_F(IrdaTest, EscapesReservedBytesBothWays)
{
    random_bytes = { BOF, EOF_, CE, 0x20 };

    open_dongle();
    power_on();

    /* The same reserved values in the command must be unescaped on the way in. */
    uint32_t const source = 0xc1c07d20;
    auto const     frames = discover(source);

    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(get_le32(frames[0], 3), 0x207dc1c0u);
    EXPECT_EQ(get_le32(frames[0], 7), source);
}

TEST_F(IrdaTest, IgnoresSomeoneElsesAddress)
{
    open_dongle();
    power_on();

    EXPECT_TRUE(discover(0x01020304, 0x55555555).empty());
}

TEST_F(IrdaTest, OnlyAnswersItsSlot)
{
    open_dongle();
    power_on();

    EXPECT_TRUE(discover(0x01020304, 0xffffffff, 3).empty());
    EXPECT_TRUE(discover(0x01020304, 0xffffffff, 0xff).empty());
}

TEST_F(IrdaTest, IgnoresABadFcs)
{
    open_dongle();
    power_on();

    send(wrap(xid_cmd(1), 0x0001));
    drain();
    EXPECT_TRUE(rx.empty());
}

TEST_F(IrdaTest, IgnoresOtherFrames)
{
    std::vector<uint8_t> snrm = xid_cmd(1);

    open_dongle();
    power_on();

    snrm[1] = 0x93;
    send(wrap(snrm));

    /* An XID response is not a command. */
    auto rsp = xid_cmd(1);
    rsp[0]   = 0xfe;
    rsp[1]   = 0xbf;
    send(wrap(rsp));

    /* Nor is a frame shorter than an XID one an XID. */
    send(wrap({ 0xff, 0x3f, 0x01 }));

    drain();
    EXPECT_TRUE(rx.empty());
}

TEST_F(IrdaTest, AbortedFrameIsDropped)
{
    std::vector<uint8_t> stream = wrap(xid_cmd(1));

    open_dongle();
    power_on();

    stream.back() = CE;
    stream.push_back(EOF_);
    send(stream);
    drain();
    EXPECT_TRUE(rx.empty());
}

TEST_F(IrdaTest, NewBofAbandonsTheFrameBeforeIt)
{
    open_dongle();
    power_on();

    send({ BOF, 0x12, 0x34, CE });
    ASSERT_EQ(discover().size(), 1u);
}

TEST_F(IrdaTest, RecoversFromAnOverlongFrame)
{
    std::vector<uint8_t> flood(0x20000, 0x55);

    open_dongle();
    power_on();

    flood[0] = BOF;
    send(flood);
    send({ EOF_ });
    drain();
    EXPECT_TRUE(rx.empty());

    ASSERT_EQ(discover().size(), 1u);
}

TEST_F(IrdaTest, DeliversOneCharacterPerPeriodHoweverOftenItIsPolled)
{
    open_dongle();
    power_on();

    send(wrap(xid_cmd(1)));
    ASSERT_TRUE(timer_is_enabled(dongle_timer));

    /* The UART polls once per bit: nothing is there before a character time. */
    EXPECT_EQ(poll(), -1);

    tick();
    EXPECT_EQ(poll(), XBOF);
    EXPECT_EQ(poll(), -1);
    EXPECT_TRUE(timer_is_enabled(dongle_timer));

    drain();
    EXPECT_FALSE(timer_is_enabled(dongle_timer));
}

TEST_F(IrdaTest, KeepsItsPaceWhenTheUartPollsLate)
{
    int                  ticks = 0;
    std::vector<uint8_t> stream;

    open_dongle();
    power_on();

    send(wrap(xid_cmd(1)));
    while (timer_is_enabled(dongle_timer)) {
        tick();
        ticks++;
    }

    /* Polling is no part of the timing: every character has arrived by now. */
    for (int i = 0; i < ticks; i++) {
        int const val = poll();
        ASSERT_GE(val, 0);
        stream.push_back(static_cast<uint8_t>(val));
    }
    EXPECT_EQ(poll(), -1);
    EXPECT_EQ(unwrap(stream).size(), 1u);
}

TEST_F(IrdaTest, UnpoweredDongleNeitherSendsNorReceives)
{
    irda_device_t listener = {};
    int           heard    = 0;

    open_dongle();

    listener.write = [](void *p, uint8_t) { ++*static_cast<int *>(p); };
    listener.priv  = &heard;
    irda_register_device(&listener);

    send(wrap(xid_cmd(1)));
    EXPECT_EQ(heard, 0);
    EXPECT_FALSE(timer_is_enabled(dongle_timer));
    EXPECT_EQ(poll(), -1);

    irda_broadcast_data(&listener, 0x55);
    EXPECT_FALSE(timer_is_enabled(dongle_timer));

    irda_unregister_device(&listener);
}

TEST_F(IrdaTest, LosingPowerLosesWhatWasQueued)
{
    open_dongle();
    power_on();

    send(wrap(xid_cmd(1)));
    control(0);
    EXPECT_FALSE(timer_is_enabled(dongle_timer));

    power_on();
    EXPECT_FALSE(timer_is_enabled(dongle_timer));
    drain();
    EXPECT_TRUE(rx.empty());
    EXPECT_EQ(poll(), -1);
}

TEST_F(IrdaTest, LosingPowerForgetsACharacterThatHadArrived)
{
    open_dongle();
    power_on();

    send(wrap(xid_cmd(1)));
    tick();
    control(0);
    power_on();
    EXPECT_EQ(poll(), -1);

    /* New data takes its full time, whatever had arrived before. */
    send(wrap(xid_cmd(1)));
    EXPECT_EQ(poll(), -1);
}

TEST_F(IrdaTest, SurvivesAFullQueue)
{
    open_dongle();
    power_on();

    /* Each discovery queues about 40 bytes, which is well past the queue's size. */
    auto const stream = wrap(xid_cmd(1));
    for (int i = 0; i < 2000; i++)
        send(stream);
    EXPECT_TRUE(timer_is_enabled(dongle_timer));
}

TEST_F(IrdaTest, ResetClearsTheQueueButNotTheLines)
{
    open_dongle();
    power_on();

    send(wrap(xid_cmd(1)));
    esi9680_device.reset(priv);
    EXPECT_FALSE(timer_is_enabled(dongle_timer));
    EXPECT_TRUE(lines_up());

    ASSERT_EQ(discover().size(), 1u);
}

TEST_F(IrdaTest, ResetInTheMiddleOfAFrameStartsClean)
{
    open_dongle();
    power_on();

    send({ BOF, 0x12, CE });
    ASSERT_EQ(added.size(), 2u);
    ASSERT_EQ(added[1].dev, &irda_obex_device);
    added[1].dev->reset(added[1].priv);

    /* The body of this frame would be corrupted by a leftover escape. */
    std::vector<uint8_t> stream = wrap(xid_cmd(1));
    std::vector<uint8_t> body(stream.begin() + 11, stream.end());
    send(body);
    drain();
    EXPECT_TRUE(rx.empty());

    ASSERT_EQ(discover().size(), 1u);
}

TEST_F(IrdaTest, SpeedChangeKeepsDeliveryGoing)
{
    open_dongle();
    power_on();

    send(wrap(xid_cmd(1)));
    esi9680_device.speed_changed(priv);
    EXPECT_TRUE(timer_is_enabled(dongle_timer));

    drain();
    esi9680_device.speed_changed(priv);
    EXPECT_FALSE(timer_is_enabled(dongle_timer));
}

TEST_F(IrdaTest, PortWithoutAChipGetsNoDongle)
{
    com_ports[0].ep = nullptr;

    com_ports[0].device = 1;
    serial_devices_init();

    EXPECT_TRUE(added.empty());
}

TEST_F(IrdaTest, UnpluggingClosesThePeerAndLeavesTheMedium)
{
    irda_device_t speaker = {};
    irda_device_t listener = {};
    int           heard   = 0;

    open_dongle();
    power_on();
    send(wrap(xid_cmd(1)));
    ASSERT_TRUE(timer_running);

    /* The settings change the device of a port while the machine runs. */
    com_ports[0].device = 0;
    serial_devices_reset();

    EXPECT_TRUE(added.empty());
    EXPECT_EQ(chip.be, nullptr);
    EXPECT_FALSE(timer_running); /* It would fire into freed memory. */

    /* Anything still registered would be called through a freed pointer. */
    listener.write = [](void *p, uint8_t) { ++*static_cast<int *>(p); };
    listener.priv  = &heard;
    irda_register_device(&listener);
    irda_register_device(&speaker);
    irda_broadcast_data(&speaker, 0x55);
    EXPECT_EQ(heard, 1);
}

TEST_F(IrdaTest, CanBePluggedBackIn)
{
    open_dongle();
    power_on();

    com_ports[0].device = 0;
    serial_devices_reset();

    com_ports[0].device = 1;
    serial_devices_reset();
    ASSERT_EQ(added.size(), 2u);

    power_on();
    EXPECT_EQ(discover().size(), 1u);
}

TEST_F(IrdaTest, IsHotpluggable)
{
    EXPECT_EQ(esi9680_device.flags & DEVICE_HOTPLUG, static_cast<uint32_t>(DEVICE_HOTPLUG));
    EXPECT_NE(esi9680_device.flags & DEVICE_COM, 0u);
    EXPECT_EQ(esi9680_device.config, nullptr);
}

TEST(IrdaMedium, UnregisterKeepsTheRest)
{
    irda_device_t devs[3] = {};
    int           heard[3] = {};
    irda_device_t speaker = {};

    irda_reset();
    for (int i = 0; i < 3; i++) {
        devs[i].write = [](void *p, uint8_t) { ++*static_cast<int *>(p); };
        devs[i].priv  = &heard[i];
        irda_register_device(&devs[i]);
    }

    irda_unregister_device(&devs[1]);
    irda_broadcast_data(&speaker, 0x55);

    EXPECT_EQ(heard[0], 1);
    EXPECT_EQ(heard[1], 0);
    EXPECT_EQ(heard[2], 1);
    irda_reset();
}

TEST(IrdaMedium, RefusesMoreDevicesThanItHolds)
{
    std::vector<irda_device_t> devs(300);
    irda_device_t              speaker = {};
    int                        heard   = 0;

    irda_reset();
    for (auto &dev : devs) {
        dev.write = [](void *p, uint8_t) { ++*static_cast<int *>(p); };
        dev.priv  = &heard;
        irda_register_device(&dev);
    }

    irda_broadcast_data(&speaker, 0x55);
    EXPECT_EQ(heard, 256);
    irda_reset();
}

TEST(IrdaMedium, DoesNotEchoToTheSender)
{
    irda_device_t dev   = {};
    int           heard = 0;

    irda_reset();
    dev.write = [](void *p, uint8_t) { ++*static_cast<int *>(p); };
    dev.priv  = &heard;
    irda_register_device(&dev);

    irda_broadcast_data(&dev, 0x55);
    EXPECT_EQ(heard, 0);
    irda_reset();
}
