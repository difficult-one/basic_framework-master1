#include "power_feedback.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static void check(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}

static ChassisPowerFeedback meter(uint32_t time, float power = 40.0f)
{
    ChassisPowerFeedback f = {};
    f.power_w = power;
    f.power_source = CHASSIS_POWER_SOURCE_METER_CAN1;
    f.power_sequence = 1;
    f.power_sample_ms = time;
    f.power_timeout_ms = 100;
    f.buffer_timeout_ms = 300;
    return f;
}

int main()
{
    // A valid meter must not trigger model fallback just because referee is absent.
    PowerFeedbackController independent;
    auto f = meter(1000);
    auto s = independent.update(f, 1000, 160.0f, 80.0f, 0.0f);
    check(s.power_valid && !s.buffer_valid, "meter validity independent of buffer");
    check(std::fabs(s.effective_limit_w - 78.4f) < 0.001f, "no duplicate model fallback");

    PowerFeedbackController zero;
    f = meter(0, 0);
    f.buffer_valid = 1;
    f.buffer_energy_j = 0;
    f.buffer_sequence = 1;
    s = zero.update(f, 0, 20, 80, 0);
    check(s.power_valid && s.buffer_valid, "zero is valid data");
    check(s.buffer_attenuation <= 0.35f, "zero buffer protects immediately");

    PowerFeedbackController fresh;
    f = meter(0);
    check(fresh.update(f, 99, 20, 80, 0).power_valid, "fresh sample accepted");
    check(!fresh.update(f, 100, 20, 80, 0).power_valid, "sample expires at timeout");
    f = meter(UINT32_MAX - 20);
    check(fresh.update(f, 10, 20, 80, 0).power_valid, "timestamp wrap preserves freshness");
    f.power_w = std::numeric_limits<float>::quiet_NaN();
    check(!fresh.update(f, 11, 20, 80, 0).power_valid, "NaN power rejected");

    PowerFeedbackController samples;
    f = meter(100, 80);
    samples.update(f, 100, 20, 80, 0);
    f.power_sequence++;
    f.power_sample_ms = 120;
    s = samples.update(f, 120, 20, 80, 0);
    check(s.model_correction > 1.0f, "new measurement corrects underestimated model");
    const float correction = s.model_correction;
    for (uint32_t t = 121; t < 150; ++t)
        s = samples.update(f, t, 20, 80, 0);
    check(s.model_correction == correction, "old frame is not relearned every motor tick");
    s = samples.update(f, 250, 20, 80, 0);
    check(s.model_correction == correction, "dropout retains conservative correction");

    PowerFeedbackController recovery;
    f = meter(100, 160);
    auto low = recovery.update(f, 100, 20, 80, 0);
    f.power_source = CHASSIS_POWER_SOURCE_REFEREE;
    f.power_w = 20;
    f.power_sample_ms = 101;
    auto high = recovery.update(f, 101, 20, 80, 0);
    check(high.effective_limit_w <= low.effective_limit_w + 0.081f,
          "switching source cannot abruptly release power");

    PowerFeedbackController regeneration;
    f = meter(100, -10);
    s = regeneration.update(f, 100, 20, 80, 0);
    check(s.power_valid && s.measured_power_w == -10, "signed regeneration is valid");
    check(s.model_correction == 1, "regeneration does not reduce model correction");

    PowerFeedbackController fallback;
    f = {};
    s = fallback.update(f, 0, 160, 80, 0);
    check(!s.power_valid && s.power_source == CHASSIS_POWER_SOURCE_NONE,
          "no measurements selects model fallback");
    check(std::fabs(s.effective_limit_w - 39.2f) < 0.001f,
          "model over limit conservatively attenuates fallback");

    // Equal elapsed time and constant physical input should give equal correction
    // even when the sensor reports every 10ms versus every 20ms.
    PowerFeedbackController fast, slow;
    auto fast_frame = meter(100, 80);
    auto slow_frame = fast_frame;
    fast.update(fast_frame, 100, 20, 80, 0);
    slow.update(slow_frame, 100, 20, 80, 0);
    for (uint32_t t = 110; t <= 300; t += 10)
    {
        ++fast_frame.power_sequence;
        fast_frame.power_sample_ms = t;
        fast.update(fast_frame, t, 20, 80, 0);
        if (t % 20 == 0)
        {
            ++slow_frame.power_sequence;
            slow_frame.power_sample_ms = t;
            slow.update(slow_frame, t, 20, 80, 0);
        }
    }
    check(std::fabs(fast.status().model_correction - slow.status().model_correction) < 0.0001f,
          "model smoothing follows sample time rather than task call count");

    PowerFeedbackController buffer;
    f = meter(100, 20);
    f.buffer_valid = 1;
    f.buffer_sequence = 1;
    f.buffer_sample_ms = 100;
    f.buffer_energy_j = 35;
    buffer.update(f, 100, 20, 80, 0);
    ++f.buffer_sequence;
    f.buffer_sample_ms = 120;
    s = buffer.update(f, 120, 20, 80, 0);
    const float buffer_attenuation = s.buffer_attenuation;
    for (uint32_t t = 121; t < 150; ++t)
        s = buffer.update(f, t, 20, 80, 0);
    check(s.buffer_attenuation == buffer_attenuation, "duplicate buffer frame is not integrated");
    s = buffer.update(f, 420, 20, 80, 0);
    check(!s.buffer_valid && s.buffer_attenuation == buffer_attenuation,
          "stale referee cannot imply recovered buffer energy");

    PowerFeedbackController limits;
    f = meter(100, 20);
    limits.update(f, 100, 20, 80, 0);
    s = limits.update(f, 101, 20, 40, 0);
    check(s.effective_limit_w <= 39.2f, "lower referee limit tightens immediately");
    const float tightened = s.effective_limit_w;
    s = limits.update(f, 102, 20, 80, 0);
    check(s.effective_limit_w <= tightened + 0.081f, "higher limit releases gradually");
    s = limits.update(f, 1102, 20, 80, 0);
    check(s.effective_limit_w <= tightened + 1.681f, "long task pause caps recovery step");

    PowerFeedbackController idle;
    f = meter(100, 10);
    s = idle.update(f, 100, 0, 80, 10);
    check(s.effective_limit_w <= 68.4f,
          "chassis supply idle load is reserved before allocating motor power");
    s = idle.update(f, 200, 0, 80, 10);
    check(!s.power_valid && s.effective_limit_w <= 68.4f,
          "model fallback also reserves the chassis idle load");
    s = idle.update(f, 201, 0, 5, 10);
    check(s.effective_limit_w == 0, "idle above total limit leaves no motor budget");

    std::puts("PASS: power feedback regression tests");
}
