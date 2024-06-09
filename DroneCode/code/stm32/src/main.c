#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "IAALAB03"
#include "debug.h"
#include "commander.h"
#include "pm.h"
#include "log.h"


#include "cpx_internal_router.h"


void receive_app_msg(const CPXPacket_t* cpxRx) {

  DEBUG_PRINT("CL FREQ = %d\n", *(int *)(cpxRx->data));
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  vTaskDelay(M2T(300));
  
  cpxRegisterAppMessageHandler(receive_app_msg);

  while(1) {
    
    vTaskDelay(M2T(100));    
  }
}
