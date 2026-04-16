/* Copyright 2024. All Rights Reserved. */
/* GPIO Configuration Loader Implementation */

#include "gpio_config.h"
#include "modbus_func.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Default DI configuration
static const gpio_channel_config_t default_di_channels[GPIO_DI_MAX_CHANNELS] = {
    {34,  "PB2",  "DI_CH01"},
    {129, "PE1",  "DI_CH02"},
    {130, "PE2",  "DI_CH03"},
    {131, "PE3",  "DI_CH04"},
    {132, "PE4",  "DI_CH05"},
    {297, "PJ9",  "DI_CH06"},
    {326, "PK6",  "DI_CH07"},
    {35,  "PB3",  "DI_CH08"},
    {136, "PE8",  "DI_CH09"},
    {137, "PE9",  "DI_CH10"},
    {138, "PE10", "DI_CH11"},
    {139, "PE11", "DI_CH12"},
    {140, "PE12", "DI_CH13"},
    {141, "PE13", "DI_CH14"},
    {142, "PE14", "DI_CH15"},
    {143, "PE15", "DI_CH16"},
    {144, "PF0",  "DI_CH17"},
    {145, "PF1",  "DI_CH18"},
    {146, "PF2",  "DI_CH19"},
    {147, "PF3",  "DI_CH20"},
    {148, "PF4",  "DI_CH21"},
    {149, "PF5",  "DI_CH22"}
};

// Default DO configuration
static const gpio_channel_config_t default_do_channels[GPIO_DO_MAX_CHANNELS] = {
    {35,  "PB3",  "DO_CH01"},
    {36,  "PB4",  "DO_CH02"},
    {37,  "PB5",  "DO_CH03"},
    {38,  "PB6",  "DO_CH04"},
    {39,  "PB7",  "DO_CH05"},
    {40,  "PB8",  "DO_CH06"},
    {41,  "PB9",  "DO_CH07"},
    {42,  "PB10", "DO_CH08"},
    {43,  "PB11", "DO_CH09"},
    {44,  "PB12", "DO_CH10"},
    {45,  "PB13", "DO_CH11"},
    {46,  "PB14", "DO_CH12"}
};

void gpio_config_load_default(gpio_config_t *config) {
    if (!config) return;

    // Load default DI channels
    config->di_count = GPIO_DI_MAX_CHANNELS;
    for (int i = 0; i < GPIO_DI_MAX_CHANNELS; i++) {
        config->di_channels[i].gpio_num = default_di_channels[i].gpio_num;
        config->di_channels[i].name = default_di_channels[i].name;
        config->di_channels[i].label = default_di_channels[i].label;
    }

    // Load default DO channels
    config->do_count = GPIO_DO_MAX_CHANNELS;
    for (int i = 0; i < GPIO_DO_MAX_CHANNELS; i++) {
        config->do_channels[i].gpio_num = default_do_channels[i].gpio_num;
        config->do_channels[i].name = default_do_channels[i].name;
        config->do_channels[i].label = default_do_channels[i].label;
    }
}

int gpio_config_load(const char *filename, gpio_config_t *config) {
    if (!filename || !config) return -1;

    FILE *fp = fopen(filename, "r");
    if (!fp) {
        LOG_WARN("Cannot open GPIO config file %s, using defaults", filename);
        gpio_config_load_default(config);
        return 0;
    }

    // Parse config file
    char line[256];
    int di_idx = 0, do_idx = 0;
    int in_di = 0, in_do = 0;

    LOG_INFO("Parsing GPIO config from %s...", filename);

    while (fgets(line, sizeof(line), fp)) {
        // Detect section
        if (strstr(line, "\"di\":")) {
            in_di = 1; in_do = 0;
            continue;
        }
        if (strstr(line, "\"do\":")) {
            in_di = 0; in_do = 1;
            continue;
        }

        // Parse channel entries
        if ((in_di || in_do) && strstr(line, "\"gpio\":")) {
            int gpio_num;
            char label[64];

            if (sscanf(line, " %*c %*s %*s %d", &gpio_num) == 1) {
                // Try to extract label
                char *label_start = strstr(line, "\"label\":");
                if (label_start) {
                    label_start = strchr(label_start, '"');
                    if (label_start) {
                        label_start++;
                        char *label_end = strchr(label_start, '"');
                        if (label_end) {
                            int len = label_end - label_start;
                            if (len < 64) {
                                strncpy(label, label_start, len);
                                label[len] = '\0';

                                if (in_di && di_idx < GPIO_DI_MAX_CHANNELS) {
                                    config->di_channels[di_idx].gpio_num = gpio_num;
                                    config->di_channels[di_idx].name = "GPIO";
                                    config->di_channels[di_idx].label = strdup(label);
                                    di_idx++;
                                } else if (in_do && do_idx < GPIO_DO_MAX_CHANNELS) {
                                    config->do_channels[do_idx].gpio_num = gpio_num;
                                    config->do_channels[do_idx].name = "GPIO";
                                    config->do_channels[do_idx].label = strdup(label);
                                    do_idx++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    fclose(fp);

    // Set channel counts
    config->di_count = di_idx;
    config->do_count = do_idx;

    // If no channels parsed, use defaults
    if (di_idx == 0 && do_idx == 0) {
        LOG_WARN("Failed to parse GPIO config, using defaults");
        gpio_config_load_default(config);
        return 0;
    }

    LOG_INFO("Loaded GPIO config: %d DI, %d DO channels", di_idx, do_idx);
    return 0;
}

void gpio_config_print(const gpio_config_t *config) {
    if (!config) return;

    printf("\n========== GPIO Configuration ==========\n");

    printf("DI Channels (%d):\n", config->di_count);
    for (int i = 0; i < config->di_count; i++) {
        printf("  %s: GPIO %d (%s)\n",
               config->di_channels[i].label ? config->di_channels[i].label : "?",
               config->di_channels[i].gpio_num,
               config->di_channels[i].name ? config->di_channels[i].name : "?");
    }

    printf("DO Channels (%d):\n", config->do_count);
    for (int i = 0; i < config->do_count; i++) {
        printf("  %s: GPIO %d (%s)\n",
               config->do_channels[i].label ? config->do_channels[i].label : "?",
               config->do_channels[i].gpio_num,
               config->do_channels[i].name ? config->do_channels[i].name : "?");
    }

    printf("========================================\n\n");
}
