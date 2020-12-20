#include <stdio.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/outgoing_lora_message.h>

#undef ASSERT
extern "C"{
#include "basicmac/lmic/lmic.h"
#include "board.h"
}

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getJoinEui (u1_t* /* buf */) { }
void os_getDevEui (u1_t* /* buf */) { }
void os_getNwkKey (u1_t* /* buf */) { }

// The region to use, this just uses the first one (can be changed if
// multiple regions are enabled).
u1_t os_getRegion (void) { return LMIC_regionCode(0); }

// Schedule TX every this many milliseconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60000;

void onLmicEvent (ev_t ev) {
    printf("%d: ", os_getTime());
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            printf("EV_SCAN_TIMEOUT\n");
            break;
        case EV_BEACON_FOUND:
            printf("EV_BEACON_FOUND\n");
            break;
        case EV_BEACON_MISSED:
            printf("EV_BEACON_MISSED\n");
            break;
        case EV_BEACON_TRACKED:
            printf("EV_BEACON_TRACKED\n");
            break;
        case EV_JOINING:
            printf("EV_JOINING\n");
            break;
        case EV_JOINED:
            printf("EV_JOINED\n");
            break;
        case EV_RFU1:
            printf("EV_RFU1\n");
            break;
        case EV_JOIN_FAILED:
            printf("EV_JOIN_FAILED\n");
            break;
        case EV_REJOIN_FAILED:
            printf("EV_REJOIN_FAILED\n");
            break;
        case EV_TXCOMPLETE:
            printf("EV_TXCOMPLETE (includes waiting for RX windows)\n");
            if (LMIC.txrxFlags & TXRX_ACK)
              printf("Received ack\n");
            if (LMIC.dataLen)
              printf("Received %d bytes of payload\n", LMIC.dataLen);
            break;
        case EV_LOST_TSYNC:
            printf("EV_LOST_TSYNC\n");
            break;
        case EV_RESET:
            printf("EV_RESET\n");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            printf("EV_RXCOMPLETE\n");
            break;
        case EV_LINK_DEAD:
            printf("EV_LINK_DEAD\n");
            break;
        case EV_LINK_ALIVE:
            printf("EV_LINK_ALIVE\n");
            break;
        case EV_SCAN_FOUND:
            printf("EV_SCAN_FOUND\n");
            break;
        case EV_TXSTART:
            printf("EV_TXSTART\n");
            break;
        case EV_TXDONE:
            printf("EV_TXDONE\n");
            break;
        case EV_DATARATE:
            printf("EV_DATARATE\n");
            break;
        case EV_START_SCAN:
            printf("EV_START_SCAN\n");
            break;
        case EV_ADR_BACKOFF:
            printf("EV_ADR_BACKOFF\n");
            break;
         default:
            printf("Unknown event: %d\n", ev);
            break;
    }
}

struct outgoing_lora_message_s outgoing;
px4_sem_t sem_message;
px4_sem_t sem_message_done;
extern px4_sem_t sem_wakeup;

extern "C"
int thread_poll(int argc, char **argv)
{
    int sub_fd = orb_subscribe(ORB_ID(outgoing_lora_message));
    px4_pollfd_struct_t fds[] = {
        { .fd = sub_fd, .events = POLLIN },
    };
    while (true) {
        sem_wait(&sem_message_done);
        while (true) {
            int poll_ret = px4_poll(fds, 1, 1000);
            if (poll_ret < 0) {
                return 0;
            }
            if (fds[0].revents & POLLIN)
                break;
        }
        orb_copy(ORB_ID(outgoing_lora_message), sub_fd, &outgoing);
        sem_post(&sem_message);
        sem_post(&sem_wakeup);
    }
}

int packet_counter;
int ordinary_dr;
int extraordinary_dr;
int extraordinary_dr_period;

void check_message() {
    if (sem_trywait(&sem_message) == 0) {
        if (!(LMIC.opmode & (OP_JOINING|OP_TXRXPEND))) {
            packet_counter++;

            if ((extraordinary_dr_period > 0) && \
                    (extraordinary_dr > 0) && \
                    (packet_counter % extraordinary_dr_period == 0))
                LMIC_setDrTxpow(extraordinary_dr, KEEP_TXPOWADJ);
            else
                LMIC_setDrTxpow(ordinary_dr, KEEP_TXPOWADJ);

            LMIC_setTxData2(1, outgoing.data, outgoing.len, 0);
        } else {
            printf("Dropping message. Bad mode.\n");
        }
        sem_post(&sem_message_done);
    }
}
void hal_loop();

// LoRaWAN NwkSKey, network session key
static u1_t NWKSKEY[16]; // = { 0x6D, 0x4C, 0x27, 0x9A, 0xD1, 0xF5, 0x3F, 0x24, 0x03, 0xA8, 0xFF, 0x98, 0x79, 0x6E, 0x3D, 0xC9 };

// LoRaWAN AppSKey, application session key
static u1_t APPSKEY[16]; // = { 0x02, 0xCC, 0x72, 0xDE, 0x1A, 0x38, 0x13, 0x62, 0x6B, 0xC4, 0x44, 0xCC, 0x3B, 0x04, 0x6D, 0xCE };

// LoRaWAN end-device address (DevAddr)
static u4_t DEVADDR; // = 0x26011483;

// These are defined by the LoRaWAN specification
enum {
    EU_DR_SF12 = 0,
    EU_DR_SF11 = 1,
    EU_DR_SF10 = 2,
    EU_DR_SF9 = 3,
    EU_DR_SF8 = 4,
    EU_DR_SF7 = 5,
    EU_DR_SF7_BW250 = 6,
    EU_DR_FSK = 7,
};

extern "C"
int thread(int argc, char **argv)
{
    // LMIC init
    os_init(nullptr);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7_BW250)); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(EU_DR_FSK,  EU_DR_FSK));      // g2-band

    // TTN uses SF9 at 869.525Mhz for its RX2 window (frequency is
    // default LoRaWAN, SF is different, but set them both to be
    // explicit).
    LMIC.dn2Freq = 869525000;
    LMIC.dn2Dr = EU_DR_SF9;

    // Set data rate for uplink
    LMIC_setDrTxpow(ordinary_dr, KEEP_TXPOWADJ);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    printf("Off to the loop!\n");

    hal_loop();
    return 0;
}

static bool parse_hex(const char *argname, const char *s, uint8_t *t, int hexlen)
{
    for (int i = 0; i < hexlen; i++) {
        int n = -1;
        char c = s[i];
        if (c >= '0' && c <= '9')
            n = c - '0';
        else if (c >= 'a' && c <= 'f')
            n = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F')
            n = c - 'A' + 10;
        else if (c == '\0')
            fprintf(stderr, "%s: argument short\n", argname);
        else
            fprintf(stderr, "%s: unexpected character: '%c'\n", argname, c);
        if (n == -1)
            return false;

        if (i % 2 == 0)
            *t = n << 4;
        else
            *t++ |= n;
    }
    if (s[hexlen] != '\0') {
        fprintf(stderr, "%s: argument too long\n", argname);
        return false;
    }
    return true;
}

static void usage() {
    fprintf(stderr, "usage: lora -n NET_SESS_KEY -a APP_SESS_KEY -d DEV_ADDR -s SF -x HIGHER_SF -p HIGHER_SF_PERIOD\n");
}

extern "C"
int
lora_main(int argc, char *argv[])
{
    int opt;
    int optind = 1;
    const char* optarg;
    int setargs = 0;

    packet_counter = 0;
    extraordinary_dr = -1;
    extraordinary_dr_period = -1;

    while ((opt = px4_getopt(argc, argv, "n:a:d:s:x:p:", &optind, &optarg)) != EOF) {
        switch (opt) {
        case 'n':
            if (!parse_hex("network session key", optarg, NWKSKEY, sizeof(NWKSKEY)*2))
                return 1;
            setargs |= 0b0001;
            break;
        case 'a':
            if (!parse_hex("app session key", optarg, APPSKEY, sizeof(APPSKEY)*2))
                return 1;
            setargs |= 0b0010;
            break;
        case 'd':
            {
                u1_t b[4];
                if (!parse_hex("device address", optarg, b, sizeof(b)*2))
                    return 1;
                DEVADDR = ((uint32_t) b[0]) << 24 \
                        | ((uint32_t) b[1]) << 16 \
                        | ((uint32_t) b[2]) << 8 \
                        | ((uint32_t) b[3]);
            };
            setargs |= 0b0100;
            break;
        case 's':
            {
                int m = atoi(optarg);
                if (m < 6 || m > 12) {
                    usage();
                    return 1;
                }
                ordinary_dr = EU_DR_SF12 + (12 - m);
            };
            setargs |= 0b1000;
            break;
        case 'x':
            {
                int m = atoi(optarg);
                if (m < 6 || m > 12) {
                    usage();
                    return 1;
                }
                extraordinary_dr = EU_DR_SF12 + (12 - m);
            };
            break;
        case 'p':
            extraordinary_dr_period = atoi(optarg);
            if (extraordinary_dr_period < 1) {
                usage();
                return 1;
            }
            break;
        default:
            usage();
            return 1;
        }
    }
    if (setargs != 0b1111) {
        usage();
        return 1;
    }

    px4_sem_init(&sem_message, 0, 0);
    px4_sem_init(&sem_message_done, 0, 1);
    px4_task_spawn_cmd("lora",
        SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5,
        2000, thread, (char *const *)nullptr);
    px4_task_spawn_cmd("lora_poll",
        SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5,
        2000, thread_poll, (char *const *)nullptr);

    return 0;
}
