#pragma once

#include "lh/session.hpp"

#include <array>
#include <deque>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <variant>

namespace lh::openvr {

// Selected devices share a timeline. Each device counts its own losses.
class TrackingIncidents {
 public:
  explicit TrackingIncidents(const std::vector<std::string>& serials);
  bool Enabled() const { return !tracks_.empty(); }
  struct Sample {
    std::string serial;
    DevicePose pose;
    bool connected;
    std::optional<int> battery;
    uint64_t roomRevision;
  };
  struct Radio { std::string serial; Session::Receiver receiver; };
  struct Marker { std::string serial; std::string reason; };
  struct Row {
    double time;
    std::variant<Sample, Radio, SessionEvent, Marker> value;
  };
  struct Clip {
    double trigger;
    std::deque<Row> rows;
  };

  void Observe(double now, const DeviceUpdate& u, uint64_t roomRevision);
  void Observe(double now, const Session::Receiver& receiver);
  void Observe(double now, const SessionEvent& event);
  std::optional<Clip> Finish(double now, bool shutdown = false);

 private:
  void Add(Row row);
  struct Track {
    bool initialized = false;
    TrackingState state = TrackingState::Uninitialized;
    double lastLoss = -1e30, lastSample = -1e30;
    double poseTime = 0;
    std::array<double, 3> position{}, velocity{};
  };
  std::map<std::string, Track> tracks_;
  bool seenTarget_ = false;
  std::deque<Row> history_;
  std::optional<Clip> clip_;
};

// Called on the writer thread. Returns the completed path, throws on a write error.
std::filesystem::path SaveTrackingIncident(const std::filesystem::path& dir, const TrackingIncidents::Clip& clip);

}  // namespace lh::openvr
