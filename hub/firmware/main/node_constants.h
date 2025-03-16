#ifndef __NODES_H__
#define __NODES_H__

#define NODE_COUNT 6
#define NODE_MAX_SECONDS_SINCE_LAST_UPDATE 3 * 60
#define NODE_TEMPERATURE_24H_MIN -1 // Index of the node whose 24h min. temperature will be shown

#define NODE0_NAME "Kuchyn"
#define NODE0_ADDRESS 0xE7E7E7E7E7

#define NODE1_NAME "Loznice"
#define NODE1_ADDRESS 0xC2C2C2C2C2

#define NODE2_NAME "Pracovna"
#define NODE2_ADDRESS 0xC3

#define NODE3_NAME "Detsky pokoj"
#define NODE3_ADDRESS 0xC4

#define NODE4_NAME "Koupelna"
#define NODE4_ADDRESS 0xC5

#define NODE5_NAME "Lodzie"
#define NODE5_ADDRESS 0xC6

extern const char *NODE_NAMES[NODE_COUNT];

#endif
