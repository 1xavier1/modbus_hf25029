/* Copyright 2024. All Rights Reserved. */
/* Hardware Abstraction Layer Implementation */

#include "hardware.h"
#include "register_map.h"
#include "ad7606.h"
#include "gpio_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <poll.h>
#include <errno.h>
#include <sys/stat.h>

// ========================================================================
// Forward Declarations
// ========================================================================

static void rs485_close_ports(void);

// ========================================================================
// Internal State
// ========================================================================

static int g_initialized = 0;
static ad7606_dev_t g_ad7606;
static int g_ad7606_fd = -1;

// DI state
static uint32_t g_di_status = 0;
static uint32_t g_di_latch = 0;
static uint32_t g_di_last_status = 0;

// DO state
static uint16_t g_do_status = 0;

// RS485 buffers (5 channels, 3 bytes each per V1.2)
static uint8_t g_rs485_rx[5][3];
static uint8_t g_rs485_len[5] = {0};
static uint8_t g_rs485_stat[5] = {0};

// RS232 buffers (3 channels, 256 bytes each)
static uint8_t g_rs232_rx[3][256];
static uint16_t g_rs232_rx_len[3] = {0};
static uint16_t g_rs232_stat[3] = {0};
static uint8_t g_rs232_err[3] = {0};

static uint8_t g_rs232_tx[3][256];
static uint16_t g_rs232_tx_len[3] = {0};

// Serial port file descriptors
static int g_rs485_fd[5] = {-1, -1, -1, -1, -1};
static int g_rs232_fd[3] = {-1, -1, -1};

// Device configuration
static uint8_t g_slave_addr = 1;
static uint8_t g_baudrate = 6;  // 115200
static uint8_t g_parity = 0;    // 0=None, 1=Odd, 2=Even

// Network configuration - IP address (PDU address 3-6)
static uint8_t g_ip_addr[4] = {10, 0, 0, 5};  // Default: 10.0.0.5

// Subnet mask (PDU address 7-10)
static uint8_t g_netmask[4] = {255, 255, 255, 0};  // Default: 255.255.255.0

// RS232/RS485 parameters
static uint8_t g_rs232_param[3][4] = {
    {6, 8, 1, 0},  // 115200, 8N1
    {6, 8, 1, 0},
    {6, 8, 1, 0}
};

static uint8_t g_rs485_param[5][4] = {
    {3, 8, 1, 0},  // 9600, 8N1
    {3, 8, 1, 0},
    {3, 8, 1, 0},
    {3, 8, 1, 0},
    {3, 8, 1, 0}
};

// RS485 protocol (V1.2)
static uint8_t g_rs485_proto[5] = {1, 1, 1, 1, 1};

// RS485 forwarding control (V1.2)
static uint8_t g_rs485_ctrl[5] = {0, 0, 0, 0, 0};

// GPIO file descriptors for DI
static int g_di_gpio_fd[DI_GPIO_COUNT] = {-1};

// ========================================================================
// Persistent Storage
// ========================================================================

#define PERSIST_DIR "/etc/modbus_slave"
#define PERSIST_FILE PERSIST_DIR "/holding_regs.dat"
#define PERSIST_MAGIC 0x4853
#define PERSIST_VERSION 1

static uint8_t g_persist_dirty[128];  // 1024 bits for dirty flags
static pthread_mutex_t g_persist_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_persist_initialized = 0;

static inline void persist_mark_dirty(uint16_t addr) {
    if (addr >= 1024) return;
    g_persist_dirty[addr / 8] |= (1 << (addr % 8));
}

static inline int persist_is_dirty(uint16_t addr) {
    if (addr >= 1024) return 0;
    return g_persist_dirty[addr / 8] & (1 << (addr % 8));
}

// CRC32 calculation
static uint32_t crc32_calc(const uint8_t *data, int len) {
    uint32_t crc = 0xFFFFFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// Ensure persist directory exists
static int persist_ensure_dir(void) {
    struct stat st;
    if (stat(PERSIST_DIR, &st) == 0) {
        if (S_ISDIR(st.st_mode)) return 0;
        return -1;
    }
    return mkdir(PERSIST_DIR, 0755);
}

// Flush dirty registers to file
void hardware_persist_flush(void) {
    if (!g_persist_initialized) return;

    pthread_mutex_lock(&g_persist_mutex);

    // Count dirty registers
    int dirty_count = 0;
    for (int i = 0; i < 128; i++) {
        if (g_persist_dirty[i / 8] & (1 << (i % 8))) {
            dirty_count++;
        }
    }

    if (dirty_count == 0) {
        pthread_mutex_unlock(&g_persist_mutex);
        return;
    }

    if (persist_ensure_dir() != 0) {
        pthread_mutex_unlock(&g_persist_mutex);
        return;
    }

    // Write to temp file first, then rename for atomicity
    char tmp_file[256];
    snprintf(tmp_file, sizeof(tmp_file), "%s.tmp", PERSIST_FILE);

    FILE *fp = fopen(tmp_file, "wb");
    if (!fp) {
        pthread_mutex_unlock(&g_persist_mutex);
        return;
    }

    // Write header
    uint16_t magic = PERSIST_MAGIC;
    uint16_t version = PERSIST_VERSION;
    fwrite(&magic, 2, 1, fp);
    fwrite(&version, 2, 1, fp);

    // Skip checksum for now (will fill in later)
    long checksum_pos = ftell(fp);
    uint32_t checksum = 0;
    fwrite(&checksum, 4, 1, fp);

    // Write dirty count
    fwrite(&dirty_count, 2, 1, fp);

    // Collect dirty register values
    uint16_t addr, value;
    for (int i = 0; i < 1024; i++) {
        if (persist_is_dirty(i)) {
            addr = (uint16_t)i;
            // Get value based on address
            if (i == REG_DEVICE_ADDR) value = g_slave_addr;
            else if (i == REG_BAUDRATE) value = g_baudrate;
            else if (i == REG_PARITY) value = g_parity;
            else if (i >= REG_IP_BASE && i < REG_IP_BASE + REG_IP_COUNT) {
                value = g_ip_addr[i - REG_IP_BASE];
            }
            else if (i >= REG_NETMASK_BASE && i < REG_NETMASK_BASE + REG_NETMASK_COUNT) {
                value = g_netmask[i - REG_NETMASK_BASE];
            }
            else if (i >= REG_RS232_PARAM_BASE && i < REG_RS232_PARAM_BASE + REG_RS232_PARAM_COUNT) {
                int ch = (i - REG_RS232_PARAM_BASE) / REG_RS232_PARAM_PER_CH;
                int idx = (i - REG_RS232_PARAM_BASE) % REG_RS232_PARAM_PER_CH;
                value = g_rs232_param[ch][idx];
            }
            else if (i >= REG_RS485_CTRL_BASE && i < REG_RS485_CTRL_BASE + REG_RS485_CTRL_COUNT) {
                value = g_rs485_ctrl[i - REG_RS485_CTRL_BASE];
            }
            else if (i >= REG_RS485_PARAM_BASE && i < REG_RS485_PARAM_BASE + REG_RS485_PARAM_COUNT) {
                int ch = (i - REG_RS485_PARAM_BASE) / REG_RS485_PARAM_PER_CH;
                int idx = (i - REG_RS485_PARAM_BASE) % REG_RS485_PARAM_PER_CH;
                value = g_rs485_param[ch][idx];
            }
            else if (i >= REG_RS485_PROTO_BASE && i < REG_RS485_PROTO_BASE + REG_RS485_PROTO_COUNT) {
                value = g_rs485_proto[i - REG_RS485_PROTO_BASE];
            }
            else continue;

            fwrite(&addr, 2, 1, fp);
            fwrite(&value, 2, 1, fp);
        }
    }

    // Calculate checksum of data section
    long data_start = checksum_pos + 6;  // after magic, version, checksum
    fseek(fp, data_start, SEEK_SET);
    uint8_t *data_buf = (uint8_t*)malloc(dirty_count * 4);
    fread(data_buf, dirty_count * 4, 1, fp);
    checksum = crc32_calc(data_buf, dirty_count * 4);
    free(data_buf);

    // Write checksum
    fseek(fp, checksum_pos, SEEK_SET);
    fwrite(&checksum, 4, 1, fp);

    fclose(fp);

    // Atomic rename
    rename(tmp_file, PERSIST_FILE);

    // Clear dirty flags
    memset(g_persist_dirty, 0, sizeof(g_persist_dirty));

    pthread_mutex_unlock(&g_persist_mutex);
    printf("[HW] Persisted %d registers\n", dirty_count);
}

void hardware_persist_init(void) {
    memset(g_persist_dirty, 0, sizeof(g_persist_dirty));
    g_persist_initialized = 1;
}

// Load registers from persistent storage
static void persist_load_all(void) {
    if (persist_ensure_dir() != 0) return;

    FILE *fp = fopen(PERSIST_FILE, "rb");
    if (!fp) return;

    uint16_t magic, version;
    if (fread(&magic, 2, 1, fp) != 1 || magic != PERSIST_MAGIC) {
        fclose(fp);
        return;
    }
    if (fread(&version, 2, 1, fp) != 1 || version != PERSIST_VERSION) {
        fclose(fp);
        return;
    }

    uint32_t stored_checksum;
    fread(&stored_checksum, 4, 1, fp);

    uint16_t dirty_count;
    fread(&dirty_count, 2, 1, fp);

    // Read all entries first
    struct { uint16_t addr; uint16_t value; } entries[128];
    int count = 0;
    for (int i = 0; i < dirty_count && i < 128; i++) {
        if (fread(&entries[i].addr, 2, 1, fp) != 1) break;
        if (fread(&entries[i].value, 2, 1, fp) != 1) break;
        count++;
    }

    // Verify checksum
    long data_pos = ftell(fp);
    fseek(fp, data_pos, SEEK_SET);
    uint8_t *buf = (uint8_t*)malloc(count * 4);
    fread(buf, count * 4, 1, fp);
    uint32_t calc_checksum = crc32_calc(buf, count * 4);
    free(buf);

    if (calc_checksum != stored_checksum) {
        printf("[HW] Persist checksum mismatch, ignoring saved data\n");
        fclose(fp);
        return;
    }

    // Apply values
    for (int i = 0; i < count; i++) {
        uint16_t addr = entries[i].addr;
        uint16_t value = entries[i].value;

        if (addr == REG_DEVICE_ADDR) {
            g_slave_addr = (uint8_t)value;
        } else if (addr == REG_BAUDRATE) {
            g_baudrate = (uint8_t)value;
        } else if (addr == REG_PARITY) {
            g_parity = (uint8_t)value;
        } else if (addr >= REG_IP_BASE && addr < REG_IP_BASE + REG_IP_COUNT) {
            g_ip_addr[addr - REG_IP_BASE] = (uint8_t)value;
        } else if (addr >= REG_NETMASK_BASE && addr < REG_NETMASK_BASE + REG_NETMASK_COUNT) {
            g_netmask[addr - REG_NETMASK_BASE] = (uint8_t)value;
        } else if (addr >= REG_RS232_PARAM_BASE && addr < REG_RS232_PARAM_BASE + REG_RS232_PARAM_COUNT) {
            int ch = (addr - REG_RS232_PARAM_BASE) / REG_RS232_PARAM_PER_CH;
            int idx = (addr - REG_RS232_PARAM_BASE) % REG_RS232_PARAM_PER_CH;
            g_rs232_param[ch][idx] = (uint8_t)value;
        } else if (addr >= REG_RS485_CTRL_BASE && addr < REG_RS485_CTRL_BASE + REG_RS485_CTRL_COUNT) {
            g_rs485_ctrl[addr - REG_RS485_CTRL_BASE] = (uint8_t)value;
        } else if (addr >= REG_RS485_PARAM_BASE && addr < REG_RS485_PARAM_BASE + REG_RS485_PARAM_COUNT) {
            int ch = (addr - REG_RS485_PARAM_BASE) / REG_RS485_PARAM_PER_CH;
            int idx = (addr - REG_RS485_PARAM_BASE) % REG_RS485_PARAM_PER_CH;
            g_rs485_param[ch][idx] = (uint8_t)value;
        } else if (addr >= REG_RS485_PROTO_BASE && addr < REG_RS485_PROTO_BASE + REG_RS485_PROTO_COUNT) {
            g_rs485_proto[addr - REG_RS485_PROTO_BASE] = (uint8_t)value;
        }
    }

    fclose(fp);
    printf("[HW] Loaded %d registers from persistent storage\n", count);
}

// ========================================================================
// GPIO Access via sysfs
// ========================================================================

static int gpio_export(int gpio) {
    char buf[64];
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, strlen(buf));
    close(fd);
    return 0;
}

static int gpio_direction(int gpio, const char *dir) {
    char buf[128];
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(buf, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, dir, strlen(dir));
    close(fd);
    return 0;
}

static int gpio_read_value(int gpio) {
    char buf[64];
    char val;
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
    fd = open(buf, O_RDONLY);
    if (fd < 0) return -1;
    read(fd, &val, 1);
    close(fd);
    return (val == '1') ? 1 : 0;
}

static int gpio_write_value(int gpio, int value) {
    char buf[64];
    int fd;

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
    fd = open(buf, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, value ? "1" : "0", 1);
    close(fd);
    return 0;
}

// ========================================================================
// DI Configuration (from gpio_config)
// ========================================================================

static const int g_di_gpio_map[DI_GPIO_COUNT] = {
    34, 129, 130, 131, 132, 297, 326, 35,
    136, 137, 138, 139, 140, 141, 142, 143,
    144, 145, 146, 147, 148, 149
};

// ========================================================================
// Configuration File Loading
// ========================================================================

static int load_network_config(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        printf("[HW] Config file %s not found, using defaults\n", filename);
        return -1;
    }

    char line[128];
    int loaded = 0;

    printf("[HW] Loading network config from %s\n", filename);

    while (fgets(line, sizeof(line), fp)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n' || line[0] == ' ')
            continue;

        // Parse ip_addr=10.0.0.5
        if (strncmp(line, "ip_addr=", 8) == 0) {
            uint8_t a, b, c, d;
            if (sscanf(line + 8, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d) == 4) {
                g_ip_addr[0] = a;
                g_ip_addr[1] = b;
                g_ip_addr[2] = c;
                g_ip_addr[3] = d;
                printf("[HW]   IP: %d.%d.%d.%d\n", a, b, c, d);
                loaded++;
            }
        }
        // Parse netmask=255.255.255.0
        else if (strncmp(line, "netmask=", 8) == 0) {
            uint8_t a, b, c, d;
            if (sscanf(line + 8, "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d) == 4) {
                g_netmask[0] = a;
                g_netmask[1] = b;
                g_netmask[2] = c;
                g_netmask[3] = d;
                printf("[HW]   Netmask: %d.%d.%d.%d\n", a, b, c, d);
                loaded++;
            }
        }
        // Parse slave_addr=1
        else if (strncmp(line, "slave_addr=", 11) == 0) {
            uint8_t addr;
            if (sscanf(line + 11, "%hhu", &addr) == 1) {
                g_slave_addr = addr;
                printf("[HW]   Slave Addr: %d\n", addr);
                loaded++;
            }
        }
        // Parse baudrate=6
        else if (strncmp(line, "baudrate=", 9) == 0) {
            uint8_t rate;
            if (sscanf(line + 9, "%hhu", &rate) == 1) {
                g_baudrate = rate;
                printf("[HW]   Baudrate: %d\n", rate);
                loaded++;
            }
        }
    }

    fclose(fp);
    printf("[HW] Loaded %d config values\n", loaded);
    return 0;
}

// ========================================================================
// Initialization
// ========================================================================

int hardware_init(void) {
    if (g_initialized) return 0;

    printf("[HW] Initializing hardware...\n");

    // Initialize persistent storage
    hardware_persist_init();

    // Load network config from file (falls back to defaults if not found)
    load_network_config("/etc/modbus_slave/network.conf");

    // Load persisted registers (overrides network.conf defaults)
    persist_load_all();

    // Initialize AD7606
    memset(&g_ad7606, 0, sizeof(g_ad7606));
    g_ad7606.gpio_reset = AD7606_GPIO_RESET;
    g_ad7606.gpio_convst = AD7606_GPIO_CONVST;
    g_ad7606.gpio_cs = AD7606_GPIO_CS;
    g_ad7606.gpio_busy = AD7606_GPIO_BUSY;
    g_ad7606.gpio_miso = AD7606_GPIO_MISO;

    if (ad7606_init(&g_ad7606) != 0) {
        printf("[HW] AD7606 init failed, using simulated data\n");
    } else {
        printf("[HW] AD7606 initialized\n");
    }

    // Export and configure DI GPIOs
    for (int i = 0; i < DI_GPIO_COUNT; i++) {
        int gpio = g_di_gpio_map[i];
        gpio_export(gpio);
        gpio_direction(gpio, "in");
        g_di_gpio_fd[i] = open("/sys/class/gpio/gpio%d/value", O_RDONLY);
        // Store the fd for later use
        char path[64];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
        g_di_gpio_fd[i] = open(path, O_RDONLY);
    }

    printf("[HW] Hardware initialized\n");
    g_initialized = 1;
    return 0;
}

int hardware_load_config(const char *config_file) {
    if (!config_file) {
        return load_network_config("/etc/modbus_slave/network.conf");
    }
    return load_network_config(config_file);
}

void hardware_release(void) {
    if (!g_initialized) return;

    // Close DI GPIO fds
    for (int i = 0; i < DI_GPIO_COUNT; i++) {
        if (g_di_gpio_fd[i] >= 0) {
            close(g_di_gpio_fd[i]);
            g_di_gpio_fd[i] = -1;
        }
    }

    // Close RS485 ports
    rs485_close_ports();

    // Close RS232 ports
    for (int i = 0; i < 3; i++) {
        if (g_rs232_fd[i] >= 0) {
            close(g_rs232_fd[i]);
            g_rs232_fd[i] = -1;
        }
    }

    // Release AD7606
    ad7606_release(&g_ad7606);

    g_initialized = 0;
    printf("[HW] Hardware released\n");
}

// ========================================================================
// AD7606 ADC
// ========================================================================

uint16_t hardware_get_pot_gain(uint8_t channel) {
    if (channel >= 5) return 0;

    // Try to read from AD7606
    if (g_ad7606.initialized) {
        return ad7606_get_raw(&g_ad7606, channel);
    }

    // Fallback: return 0 if no hardware
    return 0;
}

uint16_t hardware_get_pot_percent(uint8_t channel) {
    uint16_t val = hardware_get_pot_gain(channel);
    return (val * 100) / 4095;
}

void hardware_get_all_adc(uint16_t *samples) {
    if (!samples) return;

    if (g_ad7606.initialized) {
        for (int i = 0; i < 5; i++) {
            samples[i] = ad7606_get_raw(&g_ad7606, i);
        }
    } else {
        memset(samples, 0, 5 * sizeof(uint16_t));
    }
}

// ========================================================================
// Digital Input
// ========================================================================

uint32_t hardware_get_di_status(void) {
    uint32_t status = 0;

    for (int i = 0; i < DI_GPIO_COUNT; i++) {
        if (g_di_gpio_fd[i] >= 0) {
            char val;
            lseek(g_di_gpio_fd[i], 0, SEEK_SET);
            if (read(g_di_gpio_fd[i], &val, 1) == 1) {
                if (val == '1') {
                    status |= (1 << i);
                }
            }
        }
    }

    // Update latch on change
    uint32_t changed = status ^ g_di_last_status;
    g_di_latch |= changed;
    g_di_last_status = status;
    g_di_status = status;

    return status;
}

bool hardware_get_di(uint8_t channel) {
    if (channel >= 22) return false;
    uint32_t status = hardware_get_di_status();
    return (status >> channel) & 1;
}

uint32_t hardware_get_di_latch(void) {
    return g_di_latch;
}

void hardware_clear_di_latch(uint8_t channel) {
    if (channel >= 22) return;
    g_di_latch &= ~(1 << channel);
}

void hardware_clear_all_di_latch(void) {
    g_di_latch = 0;
}

// ========================================================================
// Digital Output
// ========================================================================

uint16_t hardware_get_do_status(void) {
    return g_do_status;
}

bool hardware_get_do(uint8_t channel) {
    if (channel >= 12) return false;
    return (g_do_status >> channel) & 1;
}

void hardware_set_do(uint8_t channel, bool value) {
    if (channel >= 12) return;

    if (value) {
        g_do_status |= (1 << channel);
    } else {
        g_do_status &= ~(1 << channel);
    }

    // Note: For real GPIO write, need to implement actual GPIO write here
    // gpio_write(DO_GPIO_BASE + channel, value);
}

// ========================================================================
// RS485 Data
// ========================================================================

uint8_t hardware_get_rs485_len(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_rs485_len[channel];
}

int hardware_get_rs485_data(uint8_t channel, uint8_t *buf, int max_len) {
    if (channel >= 5 || buf == NULL) return -1;

    int copy_len = g_rs485_len[channel];
    if (copy_len > max_len) copy_len = max_len;

    memcpy(buf, g_rs485_rx[channel], copy_len);

    // Clear data ready flag after reading
    g_rs485_stat[channel] &= ~0x01;

    return copy_len;
}

uint8_t hardware_get_rs485_stat(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_rs485_stat[channel];
}

void hardware_clear_rs485_stat(uint8_t channel, uint8_t bits) {
    if (channel >= 5) return;
    g_rs485_stat[channel] &= ~bits;
}

// ========================================================================
// RS232 Data
// ========================================================================

uint16_t hardware_get_rs232_len(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_rx_len[channel];
}

int hardware_get_rs232_data(uint8_t channel, uint8_t *buf, int max_len) {
    if (channel >= 3 || buf == NULL) return -1;

    int copy_len = g_rs232_rx_len[channel];
    if (copy_len > max_len) copy_len = max_len;

    memcpy(buf, g_rs232_rx[channel], copy_len);

    // Clear buffer not empty flag
    g_rs232_stat[channel] &= ~0x01;

    return copy_len;
}

uint16_t hardware_get_rs232_stat(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_stat[channel];
}

uint8_t hardware_get_rs232_err(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_err[channel];
}

void hardware_put_rs232_data(uint8_t channel, const uint8_t *buf, int len) {
    if (channel >= 3 || buf == NULL || len <= 0) return;
    if (len > 256) len = 256;

    memcpy(g_rs232_rx[channel], buf, len);
    g_rs232_rx_len[channel] = len;
    g_rs232_stat[channel] |= 0x01;  // Buffer not empty
}

void hardware_clear_rs232_rx(uint8_t channel) {
    if (channel >= 3) return;
    memset(g_rs232_rx[channel], 0, 256);
    g_rs232_rx_len[channel] = 0;
    g_rs232_stat[channel] &= ~0x01;
}

// ========================================================================
// Device Configuration
// ========================================================================

uint8_t hardware_get_slave_addr(void) {
    return g_slave_addr;
}

void hardware_set_slave_addr(uint8_t addr) {
    if (addr < 1 || addr > 247) return;
    g_slave_addr = addr;
    persist_mark_dirty(REG_DEVICE_ADDR);
}

uint8_t hardware_get_baudrate(void) {
    return g_baudrate;
}

void hardware_set_baudrate(uint8_t baudrate) {
    if (baudrate > 6) baudrate = 6;
    g_baudrate = baudrate;
    persist_mark_dirty(REG_BAUDRATE);
}

uint8_t hardware_get_parity(void) {
    return g_parity;
}

void hardware_set_parity(uint8_t parity) {
    if (parity > 2) parity = 0;
    g_parity = parity;
    persist_mark_dirty(REG_PARITY);
}

// ========================================================================
// Network Configuration
// ========================================================================

void hardware_get_ip_addr(uint8_t *ip) {
    if (ip) {
        memcpy(ip, g_ip_addr, 4);
    }
}

void hardware_set_ip_addr(uint8_t *ip) {
    if (ip) {
        memcpy(g_ip_addr, ip, 4);
        persist_mark_dirty(REG_IP_BASE + 0);
        persist_mark_dirty(REG_IP_BASE + 1);
        persist_mark_dirty(REG_IP_BASE + 2);
        persist_mark_dirty(REG_IP_BASE + 3);
    }
}

void hardware_get_netmask(uint8_t *netmask) {
    if (netmask) {
        memcpy(netmask, g_netmask, 4);
    }
}

void hardware_set_netmask(uint8_t *netmask) {
    if (netmask) {
        memcpy(g_netmask, netmask, 4);
        persist_mark_dirty(REG_NETMASK_BASE + 0);
        persist_mark_dirty(REG_NETMASK_BASE + 1);
        persist_mark_dirty(REG_NETMASK_BASE + 2);
        persist_mark_dirty(REG_NETMASK_BASE + 3);
    }
}

// ========================================================================
// RS232/RS485 Parameters
// ========================================================================

void hardware_get_rs232_param(uint8_t channel,
                              uint8_t *baudrate_code,
                              uint8_t *databits,
                              uint8_t *stopbits,
                              uint8_t *parity) {
    if (channel >= 3) return;
    if (baudrate_code) *baudrate_code = g_rs232_param[channel][0];
    if (databits) *databits = g_rs232_param[channel][1];
    if (stopbits) *stopbits = g_rs232_param[channel][2];
    if (parity) *parity = g_rs232_param[channel][3];
}

void hardware_set_rs232_param(uint8_t channel,
                              uint8_t baudrate_code,
                              uint8_t databits,
                              uint8_t stopbits,
                              uint8_t parity) {
    if (channel >= 3) return;
    g_rs232_param[channel][0] = baudrate_code;
    g_rs232_param[channel][1] = databits;
    g_rs232_param[channel][2] = stopbits;
    g_rs232_param[channel][3] = parity;
    persist_mark_dirty(REG_RS232_PARAM_BASE + channel * 4 + 0);
    persist_mark_dirty(REG_RS232_PARAM_BASE + channel * 4 + 1);
    persist_mark_dirty(REG_RS232_PARAM_BASE + channel * 4 + 2);
    persist_mark_dirty(REG_RS232_PARAM_BASE + channel * 4 + 3);
}

void hardware_get_rs485_param(uint8_t channel,
                              uint8_t *baudrate_code,
                              uint8_t *databits,
                              uint8_t *stopbits,
                              uint8_t *parity) {
    if (channel >= 5) return;
    if (baudrate_code) *baudrate_code = g_rs485_param[channel][0];
    if (databits) *databits = g_rs485_param[channel][1];
    if (stopbits) *stopbits = g_rs485_param[channel][2];
    if (parity) *parity = g_rs485_param[channel][3];
}

void hardware_set_rs485_param(uint8_t channel,
                              uint8_t baudrate_code,
                              uint8_t databits,
                              uint8_t stopbits,
                              uint8_t parity) {
    if (channel >= 5) return;
    g_rs485_param[channel][0] = baudrate_code;
    g_rs485_param[channel][1] = databits;
    g_rs485_param[channel][2] = stopbits;
    g_rs485_param[channel][3] = parity;
    persist_mark_dirty(REG_RS485_PARAM_BASE + channel * 4 + 0);
    persist_mark_dirty(REG_RS485_PARAM_BASE + channel * 4 + 1);
    persist_mark_dirty(REG_RS485_PARAM_BASE + channel * 4 + 2);
    persist_mark_dirty(REG_RS485_PARAM_BASE + channel * 4 + 3);
}

// ========================================================================
// RS485 Protocol
// ========================================================================

uint8_t hardware_get_rs485_proto(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_rs485_proto[channel];
}

void hardware_set_rs485_proto(uint8_t channel, uint8_t protocol) {
    if (channel >= 5) return;
    if (protocol < 1) protocol = 1;
    if (protocol > 6) protocol = 6;
    g_rs485_proto[channel] = protocol;
    persist_mark_dirty(REG_RS485_PROTO_BASE + channel);
}

uint8_t hardware_get_rs485_ctrl(uint8_t channel) {
    if (channel >= 5) return 0;
    return g_rs485_ctrl[channel];
}

void hardware_set_rs485_ctrl(uint8_t channel, uint8_t ctrl) {
    if (channel >= 5) return;
    g_rs485_ctrl[channel] = ctrl;
    persist_mark_dirty(REG_RS485_CTRL_BASE + channel);
}

// ========================================================================
// RS485 Protocol Parsing (V1.2)
// ========================================================================

// RS485 device paths for 5 channels
static const char* g_rs485_devs[5] = {
    DEV_RS485_CH1,  // /dev/ttyAS9
    DEV_RS485_CH2,  // /dev/ttyAS7
    DEV_RS485_CH3,  // /dev/ttyAS2
    DEV_RS485_CH4,  // /dev/ttyAS8
    DEV_RS485_CH5   // /dev/ttyAS4
};

// Forward declarations for serial functions (defined in serial.c)
extern int serial_open(const char *dev, int baudrate, char parity, int databits, int stopbits);
extern void serial_close(int fd);
extern int serial_read(int fd, uint8_t *buf, int len, int timeout_ms);

// Open all RS485 serial ports
static int rs485_open_ports(void) {
    for (int i = 0; i < 5; i++) {
        if (g_rs485_fd[i] < 0) {
            // Get baudrate code for this channel
            uint8_t baudcode = g_rs485_param[i][0];
            uint8_t databits = g_rs485_param[i][1];
            uint8_t stopbits = g_rs485_param[i][2];
            uint8_t parity = g_rs485_param[i][3];

            // Convert baudrate code to actual value
            int baud = 115200;
            switch (baudcode) {
                case 0: baud = 1200; break;
                case 1: baud = 2400; break;
                case 2: baud = 4800; break;
                case 3: baud = 9600; break;
                case 4: baud = 19200; break;
                case 5: baud = 38400; break;
                case 6: baud = 115200; break;
            }

            // Convert parity code to char
            char par = 'N';
            switch (parity) {
                case 1: par = 'O'; break;
                case 2: par = 'E'; break;
            }

            g_rs485_fd[i] = serial_open(g_rs485_devs[i], baud, par, databits, stopbits);
            if (g_rs485_fd[i] >= 0) {
                printf("[RS485] CH%d opened on %s at %d\n", i+1, g_rs485_devs[i], baud);
            } else {
                printf("[RS485] CH%d failed to open %s\n", i+1, g_rs485_devs[i]);
            }
        }
    }
    return 0;
}

// Close all RS485 serial ports
static void rs485_close_ports(void) {
    for (int i = 0; i < 5; i++) {
        if (g_rs485_fd[i] >= 0) {
            serial_close(g_rs485_fd[i]);
            g_rs485_fd[i] = -1;
        }
    }
}

// Verify checksum based on protocol
// Returns 1 if checksum is valid, 0 if invalid
static int rs485_verify_checksum(uint8_t proto, const uint8_t *frame, int frame_len) {
    if (frame_len < 2) return 0;

    uint8_t calc_sum = 0;
    switch (proto) {
        case 1:
        case 2:
        case 5:
        case 6:
            // Checksum = (Byte2 + Byte3) & 0xFF
            if (frame_len < 4) return 0;
            calc_sum = (frame[2] + frame[3]) & 0xFF;
            return (calc_sum == frame[4]);
        case 3:
        case 4:
            // Checksum = (Byte2 + Byte3 + Byte4 + Byte5) & 0xFF
            if (frame_len < 6) return 0;
            calc_sum = (frame[2] + frame[3] + frame[4] + frame[5]) & 0xFF;
            return (calc_sum == frame[6]);
        default:
            return 0;
    }
}

// Parse RS485 frame and store to g_rs485_rx buffer
// Returns: 1 if data was stored, 0 if no valid data
static int rs485_parse_frame(uint8_t ch, uint8_t proto, const uint8_t *frame, int frame_len) {
    if (ch >= 5 || frame == NULL || frame_len < 2) return 0;

    // Verify frame header
    if (frame[0] != 0x5A || frame[1] != 0xA5) return 0;

    // Verify checksum
    if (!rs485_verify_checksum(proto, frame, frame_len)) {
        g_rs485_stat[ch] |= 0x02;  // CRC Error
        return 0;
    }

    // Clear previous errors
    g_rs485_stat[ch] &= ~0x06;

    // Parse based on protocol
    uint8_t reg0 = 0, reg1 = 0, reg2 = 0;

    switch (proto) {
        case 1:
        case 2:
        case 6:
            // Protocol 1/2/6: 5 bytes, Y axis (2 bytes)
            // Frame: 5A A5 YY_H YY_L CHK
            if (frame_len < 5) return 0;
            reg0 = frame[3];  // YY_L (low byte)
            reg1 = frame[2];  // YY_H (high byte)
            reg2 = frame[4];  // CHK
            break;

        case 3:
        case 4:
            // Protocol 3/4: 7 bytes, XYZ axis (3 bytes)
            // Frame: 5A A5 RSV X Y Z CHK
            if (frame_len < 7) return 0;
            reg0 = frame[3];  // X
            reg1 = frame[4];  // Y
            reg2 = frame[5];  // Z
            break;

        case 5:
            // Protocol 5: 5 bytes, Y axis signed (1 byte)
            // Frame: 5A A5 RSV YY CHK
            // YY range: -128 (0x80) to +127 (0x7F)
            // Sign-extend to 16-bit: 0x80 → 0xFF80, 0x7F → 0x007F
            if (frame_len < 5) return 0;
            reg0 = (int8_t)frame[3] < 0 ? 0xFF : 0x00;  // Sign extension
            reg1 = frame[3];  // Actual signed byte
            reg2 = 0;
            break;

        default:
            return 0;
    }

    // Store parsed data
    g_rs485_rx[ch][0] = reg0;
    g_rs485_rx[ch][1] = reg1;
    g_rs485_rx[ch][2] = reg2;
    g_rs485_len[ch] = 3;

    // Set data ready flag
    g_rs485_stat[ch] |= 0x01;  // Data Ready
    g_rs485_stat[ch] &= ~0x08; // Clear overflow

    return 1;
}

// Poll one RS485 channel for data
// Returns: 1 if data was received, 0 if not
static int rs485_poll_channel(uint8_t ch) {
    // Debug: write to a specific file - ALWAYS called
    int fd = open("/tmp/rs485_debug.txt", O_WRONLY|O_CREAT|O_APPEND, 0644);
    if (fd >= 0) {
        char dbg_msg[128];
        int dbg_len = snprintf(dbg_msg, sizeof(dbg_msg), "[RS485] poll CH%d called, g_rs485_fd=%d, g_rs485_ctrl=0x%02X\r\n",
                                ch+1, g_rs485_fd[ch], g_rs485_ctrl[ch]);
        write(fd, dbg_msg, dbg_len);
        close(fd);
    }

    if (ch >= 5 || g_rs485_fd[ch] < 0) {
        return 0;
    }

    // Force enable for testing - always
    g_rs485_ctrl[ch] = 1;

    if ((g_rs485_ctrl[ch] & 0x01) == 0) {
        return 0;
    }

    // Read available data (non-blocking, 10ms timeout)
    uint8_t buf[64];
    int n = serial_read(g_rs485_fd[ch], buf, sizeof(buf), 10);

    if (n <= 0) return 0;

    // Get protocol for this channel
    uint8_t proto = g_rs485_proto[ch];
    if (proto < 1 || proto > 6) proto = 1;

    // Expected frame length based on protocol
    int expected_len = (proto == 3 || proto == 4) ? 7 : 5;

    // Simple frame detection - wait for complete frame
    // We accumulate bytes until we have enough or timeout

    // For simplicity, just try to parse when we have enough bytes
    if (n >= expected_len) {
        // Try to find 5A A5 header and parse
        for (int i = 0; i <= n - expected_len; i++) {
            if (buf[i] == 0x5A && i + 1 < n && buf[i+1] == 0xA5) {
                if (rs485_parse_frame(ch, proto, &buf[i], n - i)) {
                    return 1;
                }
            }
        }
    }

    return 0;
}

// Poll all RS485 channels for data
// Called periodically (e.g., every 20ms) to check for new data
void hardware_rs485_poll_all(void) {
    static int init_done = 0;

    // Initialize serial ports on first call
    if (!init_done) {
        rs485_open_ports();
        init_done = 1;
    }

    for (int ch = 0; ch < 5; ch++) {
        rs485_poll_channel(ch);
    }
}

// ========================================================================
// RS232 Control
// ========================================================================

uint16_t hardware_get_rs232_tx_len(uint8_t channel) {
    if (channel >= 3) return 0;
    return g_rs232_tx_len[channel];
}

void hardware_set_rs232_tx_len(uint8_t channel, uint16_t len) {
    if (channel >= 3) return;
    if (len > 256) len = 256;
    g_rs232_tx_len[channel] = len;
}

int hardware_get_rs232_tx_data(uint8_t channel, uint8_t *buf, int max_len) {
    if (channel >= 3 || buf == NULL) return -1;

    int copy_len = g_rs232_tx_len[channel];
    if (copy_len > max_len) copy_len = max_len;

    memcpy(buf, g_rs232_tx[channel], copy_len);
    return copy_len;
}

void hardware_rs232_start_tx(uint8_t channel) {
    if (channel >= 3) return;
    g_rs232_stat[channel] |= 0x04;  // TX buffer empty
}

void hardware_clear_rs232_tx(uint8_t channel) {
    if (channel >= 3) return;
    memset(g_rs232_tx[channel], 0, 256);
    g_rs232_tx_len[channel] = 0;
}

// ========================================================================
// Debug
// ========================================================================

void hardware_print_state(void) {
    printf("\n========== Hardware State ==========\n");

    // DI Status
    uint32_t di = hardware_get_di_status();
    printf("DI Status:  0x%06X\n", di);

    // DO Status
    printf("DO Status:   0x%03X\n", g_do_status);

    // ADC Values
    uint16_t adc[5];
    hardware_get_all_adc(adc);
    printf("ADC:        ");
    for (int i = 0; i < 5; i++) {
        printf("CH%d=%d ", i+1, adc[i]);
    }
    printf("\n");

    // RS485
    printf("RS485:      ");
    for (int i = 0; i < 5; i++) {
        printf("CH%d=%dbytes,", i+1, g_rs485_len[i]);
    }
    printf("\n");

    // RS232
    printf("RS232 RX:   ");
    for (int i = 0; i < 3; i++) {
        printf("CH%d=%dbytes,", i+1, g_rs232_rx_len[i]);
    }
    printf("\n");

    printf("Slave Addr: %d, Baudrate: %d\n", g_slave_addr, g_baudrate);
    printf("===================================\n\n");
}
