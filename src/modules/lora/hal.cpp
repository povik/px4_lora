#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>

#include <px4_arch/micro_hal.h>

#include <stm32_gpio.h>
#include <stm32_exti.h>

#undef ASSERT

extern "C"{
#include "basicmac/lmic/lmic.h"
}

// Datasheet defins typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static unsigned long MAX_BUSY_TIME = 5000;

// -----------------------------------------------------------------------------
// I/O

px4_sem_t sem_wakeup;
px4_sem_t sem_irq;
px4_sem_t sem_irq_handled;
u4_t irq_time;

static int irq_handler(int irq, void *context, void *arg)
{
    if (sem_trywait(&sem_irq_handled) < 0)
        return 0; /* previous interrupt has not been handled yet, drop this one */
    irq_time = hal_ticks();
    px4_sem_post(&sem_wakeup);
    px4_sem_post(&sem_irq);
    return 0;
}

static void hal_io_init () {
    px4_sem_init(&sem_wakeup, 0, 0);
    px4_sem_init(&sem_irq, 0, 0);
    px4_sem_init(&sem_irq_handled, 0, 1);
    stm32_gpiosetevent(
        (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTI|GPIO_PIN10),
        true, false, false, irq_handler, NULL
    );
}

void check_message();
void hal_loop() {
    while (1) {
        os_runstep();

        if (sem_trywait(&sem_irq) == 0) {
            radio_irq_handler(0, irq_time);
            sem_post(&sem_irq_handled);
        }

        check_message();
    }
}

// rx = 0, tx = 1, off = -1
void hal_ant_switch (u1_t val) {
}

// set radio RST pin to given value (or keep floating!)
bool hal_pin_rst (u1_t val) {
    return false;
}

void hal_irqmask_set (int /* mask */) {
    // Not implemented
}

void hal_pin_busy_wait (void) {
    //if (lmic_pins.busy >= LMIC_UNUSED_PIN) {
        // TODO: We could probably keep some state so we know the chip
        // is in sleep, since otherwise the delay can be much shorter.
        // Also, all delays after commands (rather than waking up from
        // sleep) are measured from the *end* of the previous SPI
        // transaction, so we could wait shorter if we remember when
        // that was.
    px4_usleep(MAX_BUSY_TIME);
    //} else {
    //    unsigned long start = micros();
    //
    //    while((micros() - start) < MAX_BUSY_TIME && digitalRead(lmic_pins.busy)) /* wait */;
    //}
}

#if defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)
bool hal_dio3_controls_tcxo (void) {
    return false; // lmic_pins.tcxo == LMIC_CONTROLLED_BY_DIO3;
}
bool hal_dio2_controls_rxtx (void) {
    return false; // lmic_pins.tx == LMIC_CONTROLLED_BY_DIO2;
}
#endif // defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)

// -----------------------------------------------------------------------------
// SPI

static struct spi_dev_s *spi = NULL;

void hal_spi_init () {
    if (spi == NULL) {
        spi = px4_spibus_initialize(5);
    }

    if (spi == NULL) {
        printf("SPI bad.\n");
        exit(1);
    }
}

void hal_spi_select (int on) {
    if (on != 0) {
        SPI_LOCK(spi, true);
        SPI_SETFREQUENCY(spi, 2*1000*1000);
        SPI_SETBITS(spi, 8);
        SPI_SETMODE(spi, SPIDEV_MODE0);
        SPI_SELECT(spi, 0x10000000, true);
    } else {
        SPI_SELECT(spi, 0x10000000, false);
        SPI_LOCK(spi, false);
        px4_usleep(100);
    }
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    u1_t res = SPI_SEND(spi, out);
    return res;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    // Nothing to do
}

u4_t hal_ticks () {
    return hrt_absolute_time() / US_PER_OSTICK;
}

u8_t hal_xticks (void) {
    // TODO
    return hal_ticks();
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    if (delta > 0)
        px4_usleep(delta*US_PER_OSTICK);
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    //noInterrupts();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        //interrupts();
    }
}

u1_t hal_sleep (u1_t type, u4_t target_time) {
    if (type == HAL_SLEEP_FOREVER) {
        px4_sem_wait(&sem_wakeup);
        return 0;
    }

    s4_t delta = delta_time(target_time);
    if (delta < 10)
        return 0;

    timespec abstime;
    if (clock_gettime(CLOCK_REALTIME, &abstime) != 0) {
        hal_debug_str("clock_gettime() in hal_speed failed\n");
        hal_failed();
    }
    const unsigned billion = 1000 * 1000 * 1000;
    uint64_t nsecs = abstime.tv_nsec + ((uint64_t) delta)*1000*US_PER_OSTICK;
    abstime.tv_sec += nsecs / billion;
    nsecs -= (nsecs / billion) * billion;
    abstime.tv_nsec = nsecs;

    int ret;
    while ((ret = sem_timedwait(&sem_wakeup, &abstime)) == -1 && errno == EINTR);

    return 1;
}

void hal_watchcount (int /* cnt */) {
    // Not implemented
}

// -----------------------------------------------------------------------------
// DEBUG

#ifdef CFG_DEBUG
static void hal_debug_init() {
}


void hal_debug_str (const char* str) {
    printf("%s", str);
}

void hal_debug_led (int val) {
}
#endif // CFG_DEBUG

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
void hal_printf_init() {
}
#endif // defined(LMIC_PRINTF_TO)

void hal_spi_init();
void hal_init (void * /* bootarg */) {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
#ifdef CFG_DEBUG
    hal_debug_init();
#endif
}

void hal_failed () {
    // keep IRQs enabled, to allow e.g. USB to continue to run and allow
    // firmware uploads on boards with native USB.
    // exit(1);
}

void hal_reboot (void) {
    // TODO
    hal_failed();
}

u1_t hal_getBattLevel (void) {
    // Not implemented
    return 0;
}

void hal_setBattLevel (u1_t /* level */) {
    // Not implemented
}

void hal_fwinfo (hal_fwi* /* fwi */) {
    // Not implemented
}

u1_t* hal_joineui (void) {
    return nullptr;
}

u1_t* hal_deveui (void) {
    return nullptr;
}

u1_t* hal_nwkkey (void) {
    return nullptr;
}

u1_t* hal_appkey (void) {
    return nullptr;
}

u1_t* hal_serial (void) {
    return nullptr;
}

u4_t  hal_region (void) {
    return 0;
}

u4_t  hal_hwid (void) {
    return 0;
}

u4_t  hal_unique (void) {
    return 0;
}

u4_t hal_dnonce_next (void) {
    return os_getRndU2();
}
