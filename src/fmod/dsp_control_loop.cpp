#include "fh6/fmod/dsp_control_loop.hpp"
#include "fh6/fmod/radio_discovery.hpp"
#include "fh6/audio_source_manager.hpp"
#include "fh6/log.hpp"

#include <chrono>
#include <cstring>

namespace fh6::fmod_bridge {

namespace {
using namespace std::chrono_literals;
constexpr auto kTick           = 20ms;
constexpr auto kDiscoveryRetry = 5s;
constexpr int kDiscoveryTries  = 120; // 10-minute budget; the radio system
                                      // isn't wired up until well into launch.

// Ticks of no read_callback progress (while the source is producing PCM)
// before we conclude the DSP is attached to a dead channel. 1s @ 20ms.
constexpr int kStaleTickThreshold = 50;

// SoundName of the placeholder sample our DSP overwrites. Matches the carrier
// shipped by the radio-mod media overlay; if absent, we fall back to the
// first chain-valid instance so a stale overlay doesn't silently break audio.
constexpr const char* kTargetSoundName = "HZ6_R9_PeterBroderick_EyesClosedandTraveling";
} // namespace

ControlLoop::ControlLoop(DSPBridge& bridge, const PEImage& img, float configured_gain)
    : bridge_{bridge}, img_{img}, configured_gain_{configured_gain},
      thread_{[this](const std::stop_token& tok) { run(tok); }} {}

ControlLoop::~ControlLoop() {
    thread_.request_stop();
    if (thread_.joinable()) thread_.join();
}

void ControlLoop::run(const std::stop_token& tok) {
    log::info("[ctrl] control loop started");

    DiscoveryResult disc;
    for (int attempt = 0; attempt < kDiscoveryTries && !tok.stop_requested(); ++attempt) {
        disc = discover_radio_instances(img_);
        if (!disc.instances.empty()) break;
        for (auto t = std::chrono::steady_clock::now() + kDiscoveryRetry;
             std::chrono::steady_clock::now() < t && !tok.stop_requested();)
            std::this_thread::sleep_for(kTick);
    }

    if (disc.instances.empty()) {
        log::warn("[ctrl] discovery timed out; control loop exiting");
        return;
    }

    const RadioInstance* chosen = select_instance(disc, /*require_live=*/false);
    if (!chosen) {
        log::warn("[ctrl] discovery returned no usable instance; control loop exiting");
        return;
    }
    if (chosen->sound_name != kTargetSoundName) {
        log::warn(R"([ctrl] no instance matches target "{}"; falling back to "{}")",
                  kTargetSoundName, chosen->sound_name);
    }

    void* fmod_system = resolve_fmod_system(img_, chosen->radio_stream);
    if (!fmod_system) {
        log::warn("[ctrl] FMOD SystemI resolution failed");
        return;
    }
    std::vector<RadioInstance> targets;
    for (const auto& i : disc.instances) {
        if (i.sound_name == kTargetSoundName) targets.push_back(i);
    }
    if (targets.empty()) targets.push_back(*chosen);

    bridge_.sync_instances(targets, fmod_system);
    meta_.set_target(chosen->sample_props_body);
    log::info("[ctrl] targeted {} RadioStreamFmod instance(s) starting @0x{:X}",
              targets.size(), reinterpret_cast<uintptr_t>(chosen->radio_stream));

    // The radio HUD reads from the SampleProperties slots at a much lower
    // rate than the audio mixer. 4 Hz is more than enough and keeps the
    // memory writes off the hot path.
    constexpr int kMetaEveryNTicks = 12; // ~240 ms at the 20 ms tick rate.
    int meta_tick                  = 0;

    auto next = std::chrono::steady_clock::now();
    while (!tok.stop_requested()) {
        next += kTick;
        bridge_.manager().pump_once();

        if (++meta_tick >= kMetaEveryNTicks) {
            meta_tick = 0;
            push_metadata();

            // Check for newly spawned emitters (e.g. driving into festival range, switching views)
            auto latest_disc = discover_radio_instances(img_);
            std::vector<RadioInstance> new_targets;
            const RadioInstance* best = select_instance(latest_disc, true);
            if (best) {
                new_targets.push_back(*best);
            } else {
                best = select_instance(latest_disc, false);
                if (best) new_targets.push_back(*best);
            }
            if (!new_targets.empty()) {
                void* fs = resolve_fmod_system(img_, new_targets[0].radio_stream);
                if (fs) bridge_.sync_instances(new_targets, fs);
                meta_.set_target(new_targets[0].sample_props_body);
            }
        }

        auto* active          = bridge_.manager().active();

        const float target = [this, active] {
            if (!active) return 0.0f;
            switch (active->playback_state()) {
                case PlaybackState::playing:
                case PlaybackState::buffering:
                    return configured_gain_.load(std::memory_order_acquire);
                default: return 0.0f;
            }
        }();
        // 1-pole low-pass at ~100 ms so play/pause fades smoothly.
        const float cur = bridge_.gain();
        float next_g    = cur + (target - cur) * 0.1f;
        if (std::abs(next_g - cur) < 1e-4f) next_g = target;
        bridge_.set_gain(next_g);

        std::this_thread::sleep_until(next);
    }
    log::info("[ctrl] control loop exiting");
}

const RadioInstance* ControlLoop::select_instance(const DiscoveryResult& disc,
                                                  bool require_live) const noexcept {
    const RadioInstance* fallback = nullptr;
    for (auto& i : disc.instances) {
        if (require_live && !bridge_.channel_handle_alive(i.radio_stream)) continue;
        if (i.sound_name == kTargetSoundName) return &i;
        if (!fallback) fallback = &i;
    }
    return fallback;
}

void ControlLoop::push_metadata() noexcept {
    auto* a = bridge_.manager().active();
    if (!a) {
        meta_.update("FH6 Universal Radio", "Idle");
        return;
    }
    TrackInfo info;
    try {
        info = a->current_track();
    } catch (...) {
        return;
    }
    std::string title  = !info.title.empty() ? info.title : std::string{a->display_name()};
    std::string artist = info.artist;
    if (artist.empty()) {
        switch (a->playback_state()) {
            case PlaybackState::playing:   artist = "Playing"; break;
            case PlaybackState::buffering: artist = "Buffering"; break;
            case PlaybackState::paused:    artist = "Paused"; break;
            case PlaybackState::stopped:   artist = "Stopped"; break;
        }
    }
    meta_.update(title, artist);
}

} // namespace fh6::fmod_bridge
