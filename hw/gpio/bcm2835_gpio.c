/*
 * Raspberry Pi GPIO emulation (c) 2015 Omar Rizwan
 * This code is licensed under the GNU GPLv2 and later.
 */

#include "hw/sysbus.h"

#define TYPE_BCM2835_GPIO "bcm2835_gpio"
#define BCM2835_GPIO(obj) OBJECT_CHECK(bcm2835_gpio_state, (obj), TYPE_BCM2835_GPIO)

#define GPFSEL0 0x00
#define GPFSEL0 0x00
#define GPFSEL1 0x04
#define GPFSEL2 0x08
#define GPFSEL3 0x0C
#define GPFSEL4 0x10
#define GPFSEL5 0x14

#define GPSET0 0x1C
#define GPSET1 0x20

#define GPCLR0 0x28
#define GPCLR1 0x2C

#define GPLEV0 0x34
#define GPLEV1 0x38

#define GPEDS0 0x40
#define GPEDS1 0x44

#define GPREN0 0x4C
#define GPREN1 0x50

#define GPFEN0 0x58
#define GPFEN1 0x5C

#define GPHEN0 0x64
#define GPHEN1 0x68

#define GPLEN0 0x70
#define GPLEN1 0x74

#define GPAREN0 0x7C
#define GPAREN1 0x80

#define GPAFEN0 0x88
#define GPAFEN1 0x8C

#define GPPUD 0x94
#define GPPUDCLK0 0x98
#define GPPUDCLK1 0x9C

#define TEST 0xB0

typedef union {
    struct {
        unsigned fsel0 : 3;
        unsigned fsel1 : 3;
        unsigned fsel2 : 3;
        unsigned fsel3 : 3;
        unsigned fsel4 : 3;
        unsigned fsel5 : 3;
        unsigned fsel6 : 3;
        unsigned fsel7 : 3;
        unsigned fsel8 : 3;
        unsigned fsel9 : 3;

        unsigned reserved : 1;
    } gpfsel;

    uint32_t bits;
} bcm2835_gpio_gpfsel;

typedef struct bcm2835_gpio_state {
    SysBusDevice busdev;
    MemoryRegion iomem;

    bcm2835_gpio_gpfsel gpfsel[6];

    uint32_t gpset0;
    uint32_t gpset1;

    uint32_t gpclr0;
    uint32_t gpclr1;

    /* No gplev0, gplev1 -- generated from data. */

    uint32_t gpeds0;
    uint32_t gpeds1;

    uint32_t gpren0;
    uint32_t gpren1;

    uint32_t gpfen0;
    uint32_t gpfen1;

    uint32_t gphen0;
    uint32_t gphen1;

    uint32_t gplen0;
    uint32_t gplen1;

    uint32_t gpafen0;
    uint32_t gpafen1;

    uint32_t gppud;
    uint32_t gppudclk0;
    uint32_t gppudclk1;

    unsigned test : 4;

    /* Lower 54 bits are the GPIO pin levels. */
    uint64_t data;

    /* TODO Make 54 GPIO lines. */

    qemu_irq irq[4];
} bcm2835_gpio_state;

static const VMStateDescription vmstate_bcm2835_gpio = {
    .name = "bcm2835_gpio",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {

    }
};

static uint64_t bcm2835_gpio_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    bcm2835_gpio_state *s = (bcm2835_gpio_state *)opaque;

    switch (offset) {
    case GPFSEL0: /* GPIO Function Select 0 */
        return s->gpfsel[0].bits;
    case GPFSEL1: /* GPIO Function Select 1 */
        return s->gpfsel[1].bits;
    case GPFSEL2: /* GPIO Function Select 2 */
        return s->gpfsel[2].bits;
    case GPFSEL3: /* GPIO Function Select 3 */
        return s->gpfsel[3].bits;
    case GPFSEL4: /* GPIO Function Select 4 */
        return s->gpfsel[4].bits;
    case GPFSEL5: /* GPIO Function Select 5 */
        return s->gpfsel[5].bits;

    case GPSET0: /* GPIO Pin Output Set 0 */
        return s->gpset0;
    case GPSET1: /* GPIO Pin Output Set 1 */
        return s->gpset1;

    case GPCLR0: /* GPIO Pin Output Clear 0 */
        return s->gpclr0;
    case GPCLR1: /* GPIO Pin Output Clear 1 */
        return s->gpclr1;

    case GPLEV0: /* GPIO Pin Level 0 */
        return s->data & 0xFFFFFFFF;
    case GPLEV1: /* GPIO Pin Level 1 */
        return (s->data >> 32) & 0xFFFFFFFF;

    /* TODO Implement */
    case GPEDS0: /* GPIO Pin Event Detect Status 0 */
    case GPEDS1: /* GPIO Pin Event Detect Status 1 */
    case GPREN0: /* GPIO Pin Rising Edge Detect Enable 0 */
    case GPREN1: /* GPIO Pin Rising Edge Detect Enable 1 */
    case GPFEN0: /* GPIO Pin Falling Edge Detect Enable 0 */
    case GPFEN1: /* GPIO Pin Falling Edge Detect Enable 1 */
    case GPHEN0: /* GPIO Pin High Detect Enable 0 */
    case GPHEN1: /* GPIO Pin High Detect Enable 1 */
    case GPLEN0: /* GPIO Pin Low Detect Enable 0 */
    case GPLEN1: /* GPIO Pin Low Detect Enable 1 */
    case GPAREN0: /* GPIO Pin Async. Rising Edge Detect 0 */
    case GPAREN1: /* GPIO Pin Async. Rising Edge Detect 1 */
    case GPAFEN0: /* GPIO Pin Async. Falling Edge Detect 0 */
    case GPAFEN1: /* GPIO Pin Async. Falling Edge Detect 1 */
    case GPPUD: /* GPIO Pin Pull-up/down Enable */
    case GPPUDCLK0: /* GPIO Pin Pull-up/down Enable Clock 0 */
    case GPPUDCLK1: /* GPIO Pin Pull-up/down Enable Clock 1 */
    case TEST: /* Test */
        return 0;
    }

    return 0;
}

static void bcm2835_gpio_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    bcm2835_gpio_state *s = (bcm2835_gpio_state *)opaque;

    switch (offset) {
    case GPFSEL0: /* GPIO Function Select 0 */
        s->gpfsel[0].bits = value & 0x7FFFFFFF;
        break;
    case GPFSEL1: /* GPIO Function Select 1 */
        s->gpfsel[1].bits = value & 0x7FFFFFFF;
        break;
    case GPFSEL2: /* GPIO Function Select 2 */
        s->gpfsel[2].bits = value & 0x7FFFFFFF;
        break;
    case GPFSEL3: /* GPIO Function Select 3 */
        s->gpfsel[3].bits = value & 0x7FFFFFFF;
        break;
    case GPFSEL4: /* GPIO Function Select 4 */
        s->gpfsel[4].bits = value & 0x7FFFFFFF;
        break;
    case GPFSEL5: /* GPIO Function Select 5 */
        s->gpfsel[5].bits = value & 0x7FFFFFFF;
        break;

    case GPSET0: /* GPIO Pin Output Set 0 */
        s->gpset0 = (uint32_t)(value & 0xFFFFFFFF);
        s->data = (s->data | s->gpset0) & 0x003FFFFFFFFFFFFF;
        break;
    case GPSET1: /* GPIO Pin Output Set 1 */
        s->gpset1 = (uint32_t)(value & 0x003FFFFF);
        s->data = (s->data | ((uint64_t)s->gpset1 << 32)) & 0x003FFFFFFFFFFFFF;
        break;

    case GPCLR0: /* GPIO Pin Output Clear 0 */
        s->gpclr0 = (uint32_t)(value & 0xFFFFFFFF);
        s->data = (s->data & ~s->gpclr0) & 0x003FFFFFFFFFFFFF;
        break;
    case GPCLR1: /* GPIO Pin Output Clear 1 */
        s->gpclr1 = (uint32_t)(value & 0x003FFFFF);
        s->data = (s->data & ~((uint64_t)s->gpclr1 << 32)) & 0x003FFFFFFFFFFFFF;
        break;

    /* Manual is unclear about whether these are writable. */
    case GPLEV0: /* GPIO Pin Level 0 */
    case GPLEV1: /* GPIO Pin Level 1 */
        break;

    /* TODO Implement */
    case GPEDS0: /* GPIO Pin Event Detect Status 0 */
    case GPEDS1: /* GPIO Pin Event Detect Status 1 */
    case GPREN0: /* GPIO Pin Rising Edge Detect Enable 0 */
    case GPREN1: /* GPIO Pin Rising Edge Detect Enable 1 */
    case GPFEN0: /* GPIO Pin Falling Edge Detect Enable 0 */
    case GPFEN1: /* GPIO Pin Falling Edge Detect Enable 1 */
    case GPHEN0: /* GPIO Pin High Detect Enable 0 */
    case GPHEN1: /* GPIO Pin High Detect Enable 1 */
    case GPLEN0: /* GPIO Pin Low Detect Enable 0 */
    case GPLEN1: /* GPIO Pin Low Detect Enable 1 */
    case GPAREN0: /* GPIO Pin Async. Rising Edge Detect 0 */
    case GPAREN1: /* GPIO Pin Async. Rising Edge Detect 1 */
    case GPAFEN0: /* GPIO Pin Async. Falling Edge Detect 0 */
    case GPAFEN1: /* GPIO Pin Async. Falling Edge Detect 1 */
    case GPPUD: /* GPIO Pin Pull-up/down Enable */
    case GPPUDCLK0: /* GPIO Pin Pull-up/down Enable Clock 0 */
    case GPPUDCLK1: /* GPIO Pin Pull-up/down Enable Clock 1 */
    case TEST: /* Test */
        return;
    }
}

static const MemoryRegionOps bcm2835_gpio_ops = {
    .read = bcm2835_gpio_read,
    .write = bcm2835_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int bcm2835_gpio_init(SysBusDevice *sbd)
{
    int i;
    DeviceState *dev = DEVICE(sbd);
    bcm2835_gpio_state *s = BCM2835_GPIO(dev);

    for (i = 0; i < 4; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }

    memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_gpio_ops, s,
        TYPE_BCM2835_GPIO, 0xB1);
    sysbus_init_mmio(sbd, &s->iomem);

    // TODO Make GPIO lines, too.

    return 0;
}

static void bcm2835_gpio_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

    sdc->init = bcm2835_gpio_init;
}

static TypeInfo bcm2835_gpio_info = {
    .name          = TYPE_BCM2835_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(bcm2835_gpio_state),
    .class_init    = bcm2835_gpio_class_init,
};

static void bcm2835_gpio_register_types(void)
{
    type_register_static(&bcm2835_gpio_info);
}

type_init(bcm2835_gpio_register_types)
