/*
 * DYAD - oscillatore drone generativo per Korg NTS-1 mkII (logue SDK v2)
 *
 * Pensato per due unita' in feedback incrociato (Out A -> In B, Out B -> In A).
 *
 *  - Audio In (mono o stereo) -> DC blocker -> LP -> AGC -> tanh -> modulatore di fase
 *  - Se l'input non e' disponibile (puntatore nullo) usa l'auto-feedback interno
 *  - Oscillatori: root + 2 sub-armonici detunati (f/2 +/- d) + quinta (3/2)
 *  - Drift generativo: 3 random-walk lenti con periodi non commensurabili
 *  - Il suono NON dipende dal gate: note_off / gate_off / all_note_off ignorati
 *  - Protezione: tanh razionale veloce + AGC a inseguitore d'inviluppo
 */

#include "unit.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr int   kNumParams = 6;

enum Param : uint8_t {
  P_FEEDBACK = 0,   // Knob A
  P_SUB_DETUNE,     // Knob B
  P_DRIFT_DEPTH,
  P_DRIFT_TIME,
  P_FIFTH,
  P_DRIVE
};

const int32_t kDefaults[kNumParams] = {40, 35, 35, 50, 20, 40};

// ---------------------------------------------------------------- utilita'

inline float fast_tanh(float x) {
  // Approssimazione razionale, saturata a +/-1 oltre |x| = 3
  if (x > 3.f) x = 3.f;
  if (x < -3.f) x = -3.f;
  const float x2 = x * x;
  return x * (27.f + x2) / (27.f + 9.f * x2);
}

// sin(2*pi*t), t in giri (qualsiasi valore reale). Errore ~0.1%
inline float fast_sin_turns(float t) {
  t -= floorf(t + 0.5f);                 // [-0.5, 0.5)
  float y = 8.f * t - 16.f * t * fabsf(t);
  return 0.225f * (y * fabsf(y) - y) + y;
}

struct Dyad {
  float sr = 48000.f;
  float inv_sr = 1.f / 48000.f;
  uint8_t in_ch = 0;
  uint8_t out_ch = 1;

  int32_t params[kNumParams];

  // parametri smussati (0..1)
  float fb = 0.f, sub = 0.f, depth = 0.f, fifth = 0.f, drive = 0.f;

  // pitch (semitoni MIDI)
  float note_target = 36.f, note_cur = 36.f, bend = 0.f;

  // fasi (giri)
  float ph_main = 0.f, ph_s1 = 0.25f, ph_s2 = 0.6f, ph_fifth = 0.8f;

  // incrementi (aggiornati per blocco)
  float inc_main = 0.f, inc_s1 = 0.f, inc_s2 = 0.f, inc_fifth = 0.f;
  float idx_turns = 0.f;      // indice di modulazione in giri

  // catena d'ingresso
  float dc_x = 0.f, dc_y = 0.f, lp = 0.f, env = 0.f, last_out = 0.f;
  float lp_a = 0.3f, env_atk = 0.f, env_rel = 0.f, smooth_a = 0.f;

  // drift
  float dr_val[3] = {0.f, 0.f, 0.f};
  float dr_tgt[3] = {0.f, 0.f, 0.f};
  float dr_timer[3] = {0.f, 0.f, 0.f};
  uint32_t rng = 0x1234567u;

  float rand_bipolar() {
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return (static_cast<float>(rng & 0xFFFFFFu) / 8388608.f) - 1.f;
  }
};

Dyad g;

inline float p01(int id) { return static_cast<float>(g.params[id]) * 0.01f; }

void update_control(uint32_t frames) {
  const float dt = static_cast<float>(frames) * g.inv_sr;

  // --- drift generativo: periodo 2..60 s, fattori irrazionali tra le sorgenti
  const float period = 2.f * powf(30.f, p01(P_DRIFT_TIME));
  const float a = 1.f - expf(-dt / (period * 0.5f));
  static const float kRatio[3] = {1.f, 1.618034f, 2.414214f};
  for (int k = 0; k < 3; ++k) {
    g.dr_timer[k] -= dt;
    if (g.dr_timer[k] <= 0.f) {
      g.dr_tgt[k] = g.rand_bipolar();
      g.dr_timer[k] = period * kRatio[k] * (0.7f + 0.3f * (g.rand_bipolar() + 1.f));
    }
    g.dr_val[k] += (g.dr_tgt[k] - g.dr_val[k]) * a;
  }

  // --- glide di pitch (~80 ms)
  g.note_cur += (g.note_target - g.note_cur) * (1.f - expf(-dt / 0.08f));

  const float depth = g.depth;
  const float semis = g.note_cur + g.bend + depth * 0.15f * g.dr_val[0];
  float f0 = 440.f * exp2f((semis - 69.f) / 12.f);
  if (f0 > 8000.f) f0 = 8000.f;

  // --- sub-armonici: f/2 con detune simmetrico (0..60 cent) + drift
  const float cents = g.sub * 60.f + depth * 10.f * g.dr_val[1];
  const float r_up = exp2f(cents / 1200.f);
  const float r_dn = 1.f / r_up;

  g.inc_main  = f0 * g.inv_sr;
  g.inc_s1    = f0 * 0.5f * r_up * g.inv_sr;
  g.inc_s2    = f0 * 0.5f * r_dn * g.inv_sr;
  g.inc_fifth = f0 * 1.5f * exp2f(-depth * 8.f * g.dr_val[1] / 1200.f) * g.inv_sr;

  // --- indice PM: max 6 rad, modulato lentamente dal drift #3
  float idx = g.fb * (1.f + 0.4f * depth * g.dr_val[2]);
  if (idx < 0.f) idx = 0.f;
  g.idx_turns = idx * 6.f / kTwoPi;
}

}  // namespace

// ================================================================ unit API v2

extern "C" {

__unit_callback int8_t unit_init(const unit_runtime_desc_t* desc) {
  if (!desc) return k_unit_err_undef;
  if (desc->target != unit_header.target) return k_unit_err_target;
  if (!UNIT_API_IS_COMPAT(desc->api)) return k_unit_err_api_version;
  if (desc->samplerate != 48000) return k_unit_err_samplerate;
  if (desc->output_channels < 1 || desc->output_channels > 2) return k_unit_err_geometry;

  g = Dyad();
  g.sr = static_cast<float>(desc->samplerate);
  g.inv_sr = 1.f / g.sr;
  g.in_ch = desc->input_channels;
  g.out_ch = desc->output_channels;

  g.env_atk  = 1.f - expf(-1.f / (0.005f * g.sr));
  g.env_rel  = 1.f - expf(-1.f / (0.400f * g.sr));
  g.smooth_a = 1.f - expf(-1.f / (0.020f * g.sr));
  g.lp_a     = 1.f - expf(-kTwoPi * 3000.f / g.sr);   // LP ~3 kHz sul modulatore

  for (int i = 0; i < kNumParams; ++i) g.params[i] = kDefaults[i];
  g.fb = p01(P_FEEDBACK); g.sub = p01(P_SUB_DETUNE); g.depth = p01(P_DRIFT_DEPTH);
  g.fifth = p01(P_FIFTH); g.drive = p01(P_DRIVE);

  // fasi iniziali e timer di drift sfalsati
  for (int k = 0; k < 3; ++k) { g.dr_tgt[k] = g.rand_bipolar(); g.dr_timer[k] = 1.f + k; }
  update_control(64);
  return k_unit_err_none;
}

__unit_callback void unit_teardown() {}
__unit_callback void unit_reset() { g.dc_x = g.dc_y = g.lp = g.env = g.last_out = 0.f; }
__unit_callback void unit_resume() {}
__unit_callback void unit_suspend() {}

__unit_callback void unit_render(const float* in, float* out, uint32_t frames) {
  update_control(frames);

  const bool ext = (in != nullptr) && (g.in_ch > 0);
  const uint8_t ic = g.in_ch;
  const uint8_t oc = g.out_ch;

  const float t_fb = p01(P_FEEDBACK), t_sub = p01(P_SUB_DETUNE);
  const float t_depth = p01(P_DRIFT_DEPTH), t_fifth = p01(P_FIFTH), t_drive = p01(P_DRIVE);
  const float sa = g.smooth_a;

  for (uint32_t i = 0; i < frames; ++i) {
    g.fb    += (t_fb    - g.fb)    * sa;
    g.sub   += (t_sub   - g.sub)   * sa;
    g.depth += (t_depth - g.depth) * sa;
    g.fifth += (t_fifth - g.fifth) * sa;
    g.drive += (t_drive - g.drive) * sa;

    // ---- modulatore: Audio In (o auto-feedback)
    float m;
    if (ext) {
      m = (ic >= 2) ? 0.5f * (in[i * ic] + in[i * ic + 1]) : in[i * ic];
    } else {
      m = g.last_out * 0.6f;
    }

    // DC blocker + passa-basso
    const float dcy = m - g.dc_x + 0.995f * g.dc_y;
    g.dc_x = m; g.dc_y = dcy;
    g.lp += g.lp_a * (dcy - g.lp);

    // inseguitore d'inviluppo -> AGC (riduce il gain del loop se il livello sale)
    const float r = fabsf(g.lp);
    g.env += (r > g.env ? g.env_atk : g.env_rel) * (r - g.env);
    const float agc = 1.f / (1.f + 4.f * g.env);
    const float mod = fast_tanh(g.lp * agc * 2.f);       // limitato a +/-1

    const float mt = g.idx_turns * mod;

    // ---- oscillatori (phase modulation)
    const float s_main = fast_sin_turns(g.ph_main + mt);
    const float s1     = fast_sin_turns(g.ph_s1 + 0.5f * mt);
    const float s2     = fast_sin_turns(g.ph_s2 - 0.5f * mt);
    const float s5     = fast_sin_turns(g.ph_fifth + 0.3f * mt);

    g.ph_main  += g.inc_main;  if (g.ph_main  >= 1.f) g.ph_main  -= 1.f;
    g.ph_s1    += g.inc_s1;    if (g.ph_s1    >= 1.f) g.ph_s1    -= 1.f;
    g.ph_s2    += g.inc_s2;    if (g.ph_s2    >= 1.f) g.ph_s2    -= 1.f;
    g.ph_fifth += g.inc_fifth; if (g.ph_fifth >= 1.f) g.ph_fifth -= 1.f;

    // ---- mix + saturazione di uscita
    const float mix = 0.50f * s_main + 0.30f * (s1 + s2) + 0.40f * g.fifth * s5;
    float y = fast_tanh(mix * (1.f + 3.f * g.drive)) * 0.8f;
    if (!(y == y)) { y = 0.f; g.lp = g.env = g.dc_x = g.dc_y = 0.f; }   // anti-NaN
    g.last_out = y;

    if (oc == 1) {
      out[i] = y;
    } else {
      const float w = 0.15f * (s1 - s2);
      out[i * 2]     = y + w;
      out[i * 2 + 1] = y - w;
    }
  }
}

__unit_callback void unit_set_param_value(uint8_t id, int32_t value) {
  if (id < kNumParams) g.params[id] = value;
}
__unit_callback int32_t unit_get_param_value(uint8_t id) {
  return (id < kNumParams) ? g.params[id] : 0;
}
__unit_callback const char* unit_get_param_str_value(uint8_t, int32_t) { return nullptr; }
__unit_callback const uint8_t* unit_get_param_bmp_value(uint8_t, int32_t) { return nullptr; }

__unit_callback void unit_set_tempo(uint32_t) {}
__unit_callback void unit_tempo_4ppqn_tick(uint32_t) {}

// Drone: il pitch segue l'ultima nota, ma NESSUN evento di spegnimento azzera il suono.
__unit_callback void unit_note_on(uint8_t note, uint8_t) { g.note_target = static_cast<float>(note); }
__unit_callback void unit_note_off(uint8_t) {}
__unit_callback void unit_gate_on(uint8_t) {}
__unit_callback void unit_gate_off() {}
__unit_callback void unit_all_note_off() {}

__unit_callback void unit_pitch_bend(uint16_t bend) {
  g.bend = (static_cast<float>(bend) - 8192.f) * (2.f / 8192.f);   // +/-2 semitoni
}
__unit_callback void unit_channel_pressure(uint8_t) {}
__unit_callback void unit_aftertouch(uint8_t, uint8_t) {}

__unit_callback void unit_load_preset(uint8_t) {}
__unit_callback uint8_t unit_get_preset_index() { return 0; }
__unit_callback const char* unit_get_preset_name(uint8_t) { return nullptr; }

}  // extern "C"
