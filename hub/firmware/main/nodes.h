#ifndef __NODES_H__
#define __NODES_H__

#define NODE_COUNT 6
#define NODE_TEMPERATURE_24H_MIN 5 // Index of the node whose 24h min. temperature will be shown

#define NODE0_NAME "Pracovna"
#define NODE0_ADDRESS 0xE7E7E7E7E7

#define NODE1_NAME "Loznice"
#define NODE1_ADDRESS 0xC2C2C2C2C2

#define NODE2_NAME "Kuchyn"
#define NODE2_ADDRESS 0xC3

#define NODE3_NAME "Koupelna"
#define NODE3_ADDRESS 0xC4

#define NODE4_NAME "Lodzie"
#define NODE4_ADDRESS 0xC5

#define NODE5_NAME "Sever"
#define NODE5_ADDRESS 0xC6

extern const char *NODE_NAMES[NODE_COUNT];

#endif
