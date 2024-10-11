#ifndef SPP_COUNTER_H
#define SPP_COUNTER_H

#define BTSTACK_FILE__ "spp_counter.c"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
 
#include "btstack.h"

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 50

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static int toggle = true;

static uint16_t rfcomm_channel_id;
static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;
int btstack_main(int argc, const char * argv[]);
int return_parameters(int argc);

#endif