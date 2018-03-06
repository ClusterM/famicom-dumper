#include "defines.h"

#define CLUJTAG_CONCAT(a, b)            a ## b
#define CLUJTAG_OUTPORT(name)           CLUJTAG_CONCAT(PORT, name)
#define CLUJTAG_INPORT(name)            CLUJTAG_CONCAT(PIN, name)
#define CLUJTAG_DDRPORT(name)           CLUJTAG_CONCAT(DDR, name)

#define PORT CLUJTAG_OUTPORT(JTAG_PORT)
#define PORT_DDR CLUJTAG_DDRPORT(JTAG_PORT)
#define PORT_PIN CLUJTAG_INPORT(JTAG_PORT)
#define PORT_LED CLUJTAG_OUTPORT(LED_PORT)
#define PORT_LED_DDR CLUJTAG_DDRPORT(LED_PORT)

void jtag_setup();
void jtag_shutdown();
int jtag_execute(int count, uint8_t* data);
