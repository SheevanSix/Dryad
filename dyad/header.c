#include "unit.h"

// Header unit per NTS-1 mkII, modulo oscillatore.
// I parametri non elencati restano a zero = k_unit_param_type_none.
const __unit_header unit_header_t unit_header = {
    .header_size = sizeof(unit_header_t),
    .target      = UNIT_TARGET_PLATFORM | k_unit_module_osc,
    .api         = UNIT_API_VERSION,
    .dev_id      = 0x44594144U,   // "DYAD"
    .unit_id     = 0x00000001U,
    .version     = 0x00010000U,   // 1.0.0
    .name        = "DYAD",
    .num_presets = 0,
    .num_params  = 6,
    .params = {
        // min, max, center, default, type, frac, frac_mode, reserved, name
        {0, 100, 0, 40, k_unit_param_type_percent, 0, 0, 0, {"Feedback"}},   // Knob A
        {0, 100, 0, 35, k_unit_param_type_percent, 0, 0, 0, {"SubDetune"}},  // Knob B
        {0, 100, 0, 35, k_unit_param_type_percent, 0, 0, 0, {"DriftDepth"}},
        {0, 100, 0, 50, k_unit_param_type_percent, 0, 0, 0, {"DriftTime"}},
        {0, 100, 0, 20, k_unit_param_type_percent, 0, 0, 0, {"Fifth"}},
        {0, 100, 0, 40, k_unit_param_type_percent, 0, 0, 0, {"Drive"}},
    },
};
