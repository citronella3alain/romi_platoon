typedef enum {
  OFF,
  DRIVING,
} robot_state_t;

static void darkness_sensor();

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder);

uint32_t mean(uint32_t* arr, uint32_t arr_size);

robot_state_t controller(robot_state_t state, uint32_t* dist_mm_ptr, uint32_t num_measurements, uint32_t* data, uint8_t* instruction);
