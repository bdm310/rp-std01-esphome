#include "app.h"
#include "hal.h"
#include "stc15.h"

void main(void) {
  hal_init();
  app_init();
  for (;;)
    app_loop();
}
