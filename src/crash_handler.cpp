#include "mbed.h"
#include "platform/mbed_error.h"

// Only include stats if available in bare-metal
#if MBED_CONF_PLATFORM_HEAP_STATS_ENABLED
#include "platform/mbed_stats.h"
#endif

// Override the default error hook function
void mbed_error_hook(const mbed_error_ctx *error_context) {
  printf("\n=== FIRMWARE CRASH DETECTED ===\n");
  printf("Error Status: 0x%08X\n", error_context->error_status);
  printf("Error Value: 0x%08lX\n", error_context->error_value);
  printf("Error Address: 0x%08lX\n", error_context->error_address);

  // Print filename and line number if available
#ifdef MBED_CONF_PLATFORM_MAX_ERROR_FILENAME_LEN
  if (error_context->error_filename[0] != '\0') {
    printf("File: %s\n", error_context->error_filename);
    printf("Line: %lu\n", error_context->error_line_number);
  }
#endif

  // Print thread information
  printf("Thread ID: 0x%08lX\n", error_context->thread_id);
  printf("Thread Entry Address: 0x%08lX\n", error_context->thread_entry_address);
  printf("Thread Stack Size: %lu\n", error_context->thread_stack_size);
  printf("Thread Stack Memory: 0x%08lX\n", error_context->thread_stack_mem);
  printf("Thread Current SP: 0x%08lX\n", error_context->thread_current_sp);

  // Print memory stats (if available in bare-metal)
#if MBED_CONF_PLATFORM_HEAP_STATS_ENABLED
  printf("\n=== MEMORY STATISTICS ===\n");
  mbed_stats_heap_t heap_stats;
  mbed_stats_heap_get(&heap_stats);
  printf("Heap Size: %lu bytes\n", heap_stats.reserved_size);
  printf("Heap Used: %lu bytes\n", heap_stats.current_size);
  printf("Heap Max Used: %lu bytes\n", heap_stats.max_size);
  printf("Heap Alloc Count: %lu\n", heap_stats.alloc_cnt);
  printf("Heap Alloc Fail Count: %lu\n", heap_stats.alloc_fail_cnt);
#endif

  printf("\n=== END CRASH REPORT ===\n");
  printf("System will reboot in 3 seconds...\n");

  // Flush output to ensure the crash report is printed
  fflush(stdout);

  // Wait 3 seconds to allow the crash report to be read/logged
  wait_us(3000000);  // 3 seconds

  printf("Rebooting now...\n");
  fflush(stdout);

  // Trigger system reset
  NVIC_SystemReset();
}
