#include "tracking_incidents.hpp"

#include "lh/version.hpp"
#include "platform/platform.hpp"

#include <chrono>
#include <fstream>
#include <utility>
#include <nlohmann/json.hpp>

namespace lh::openvr {
namespace {
using json = nlohmann::json;
constexpr double kBurst = 5.0, kHistory = 6.0, kTail = 5.0;
constexpr size_t kMaxHistory = 4096;

json Encode(const TrackingIncidents::Row& row) {
  json j{{"t", row.time}};
  if (auto* s = std::get_if<TrackingIncidents::Sample>(&row.value)) {
    const auto& p = s->pose;
    j.update(json{{"type", "pose"}, {"serial", s->serial},
                  {"state", ToString(p.state)}, {"valid", p.valid}, {"connected", s->connected},
                  {"result", int(p.result)}, {"poseTime", p.hostTime}, {"position", p.worldFromImu.position},
                  {"orientation", p.worldFromImu.orientation}, {"velocity", p.velocity},
                  {"angularVelocity", p.angularVelocity}, {"filterTime", p.filterTime},
                  {"filterPosition", p.filterPosition}, {"primaryBase", p.primaryBase},
                  {"updates", p.updates}, {"roomRevision", s->roomRevision}, {"coverage", json::array()}});
    if (s->battery) j["batteryPercent"] = *s->battery;
    for (const auto& c : p.coverage)
      j["coverage"].push_back({{"base", c.base}, {"frames", c.frames}, {"meanSensors", c.sensors}});
  } else if (auto* radio = std::get_if<TrackingIncidents::Radio>(&row.value)) {
    const auto& r = radio->receiver;
    j.update(json{{"type", "radio"}, {"serial", radio->serial}, {"receiver", r.serial},
                  {"kind", r.kind}, {"linked", r.linked}, {"framesPerSec", r.framesPerSec},
                  {"kBytesPerSec", r.kBytesPerSec}});
    if (r.kind == "openhouse")
      j.update(json{{"lossPercent", r.lossPercent}, {"dropsPerSec", r.dropsPerSec},
                    {"rssiDongle", r.rssiDongle}, {"rssiDevice", r.rssiDevice}});
  } else if (auto* e = std::get_if<SessionEvent>(&row.value)) {
    j.update(json{{"type", "event"}, {"eventTime", e->hostTime}, {"serial", e->serial}, {"message", e->text}});
  } else {
    const auto& m = std::get<TrackingIncidents::Marker>(row.value);
    j.update(json{{"type", "trigger"}, {"serial", m.serial}, {"reason", m.reason}});
  }
  return j;
}
}  // namespace

TrackingIncidents::TrackingIncidents(const std::vector<std::string>& serials) {
  for (const auto& serial : serials) if (!serial.empty()) tracks_.try_emplace(serial);
}

void TrackingIncidents::Add(Row row) {
  while (!history_.empty() && (row.time - history_.front().time > kHistory || history_.size() >= kMaxHistory))
    history_.pop_front();
  if (clip_ && clip_->rows.size() < 2 * kMaxHistory) clip_->rows.push_back(row);
  history_.push_back(std::move(row));
}

void TrackingIncidents::Observe(double now, const DeviceUpdate& u, uint64_t roomRevision) {
  const auto it = tracks_.find(u.serial);
  if (it == tracks_.end() || u.role != DeviceRole::Tracker) return;
  seenTarget_ = true;
  auto& tr = it->second;
  if (!u.enabled) { tr = {}; return; }
  const auto& p = u.pose;
  const bool changed = !tr.initialized || tr.state != p.state;
  if (!changed && now - tr.lastSample < 0.01) return;

  const char* reason = nullptr;
  if (tr.initialized && tr.state == TrackingState::Tracking && p.state != TrackingState::Tracking) {
    if (now - tr.lastLoss <= kBurst) reason = "repeated_tracking_loss";
    tr.lastLoss = now;
  }
  // Position error after accounting for motion at the previous velocity.
  const double dt = p.hostTime - tr.poseTime;
  if (!changed && p.state == TrackingState::Tracking && p.valid && dt > 0 && dt <= 0.04) {
    double squaredError = 0;
    for (size_t i = 0; i < 3; ++i) {
      const double error = p.worldFromImu.position[i] - tr.position[i] - dt * tr.velocity[i];
      squaredError += error * error;
    }
    if (squaredError > 0.04 * 0.04) reason = "position_jump";
  }
  tr.initialized = true;
  tr.state = p.state;
  tr.position = p.worldFromImu.position;
  tr.velocity = p.velocity;
  tr.poseTime = p.hostTime;
  tr.lastSample = now;
  Add({now, Sample{u.serial, p, u.connected, u.batteryPercent, roomRevision}});
  if (reason) {
    if (!clip_) clip_ = Clip{now, std::move(history_)};
    Add({now, Marker{u.serial, reason}});
  }
}

void TrackingIncidents::Observe(double now, const Session::Receiver& r) {
  if (tracks_.count(r.connected)) Add({now, Radio{r.connected, r}});
}

void TrackingIncidents::Observe(double now, const SessionEvent& e) {
  if (tracks_.count(e.serial) || (e.serial.empty() && seenTarget_))
    Add({now, e});
}

std::optional<TrackingIncidents::Clip> TrackingIncidents::Finish(double now, bool shutdown) {
  if (!clip_ || (!shutdown && now - clip_->trigger < kTail)) return {};
  return std::exchange(clip_, std::nullopt);
}

std::filesystem::path SaveTrackingIncident(const std::filesystem::path& dir, const TrackingIncidents::Clip& clip) {
  namespace fs = std::filesystem;
  fs::create_directories(dir);
  fs::path dest;
  auto oldest = fs::file_time_type::max();
  for (int slot = 1; slot <= 16; ++slot) {
    const auto file = dir / ("incident-" + std::to_string(slot) + ".jsonl");
    if (!fs::exists(file)) { dest = file; break; }
    auto stamp = fs::last_write_time(file);
    if (dest.empty() || stamp < oldest) { dest = file; oldest = stamp; }
  }
  const auto tmp = dir / ("incident-" + std::to_string(platform::ProcessId()) + ".tmp");
  try {
    std::ofstream out;
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out.open(tmp, std::ios::binary);
    const auto saved = std::chrono::system_clock::now().time_since_epoch();
    out << json{{"type", "incident"}, {"version", kVersion}, {"triggerTime", clip.trigger},
                {"savedUnixMs", std::chrono::duration_cast<std::chrono::milliseconds>(saved).count()}}.dump() << '\n';
    for (const auto& row : clip.rows)
      out << Encode(row).dump(-1, ' ', false, json::error_handler_t::replace) << '\n';
    out.close();
    platform::ReplaceFile(tmp, dest);
  } catch (...) {
    std::error_code ignored;
    fs::remove(tmp, ignored);
    throw;
  }
  return dest;
}

}  // namespace lh::openvr
