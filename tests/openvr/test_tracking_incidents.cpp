#include <gtest/gtest.h>
#include <fstream>
#include <nlohmann/json.hpp>

#include "openvr/tracking_incidents.hpp"
#include "platform/platform.hpp"

namespace {
using namespace lh;
using openvr::TrackingIncidents;

const std::vector<std::string> serials{"LHR-A", "LHR-B"};

DeviceUpdate Update(size_t device, TrackingState state) {
  DeviceUpdate u;
  u.serial = serials[device];
  u.pose.state = state;
  u.connected = true;
  u.pose.valid = state == TrackingState::Tracking || state == TrackingState::Coasting;
  return u;
}

TEST(TrackingIncidents, SeparateLossesAndStartupDoNotTrigger) {
  TrackingIncidents rec(serials);
  for (size_t i = 0; i < 2; ++i) {
    rec.Observe(0, Update(i, TrackingState::Lost), 1);
    rec.Observe(1, Update(i, TrackingState::Tracking), 1);
    rec.Observe(2, Update(i, TrackingState::Coasting), 1);
    rec.Observe(3, Update(i, TrackingState::Lost), 1);
    rec.Observe(4, Update(i, TrackingState::Tracking), 1);
    rec.Observe(7.1, Update(i, TrackingState::Lost), 1);
  }
  EXPECT_FALSE(rec.Finish(20));
}

TEST(TrackingIncidents, SavesBothDevicesEventsRadioAndRecovery) {
  TrackingIncidents rec(serials);
  auto other = Update(0, TrackingState::Tracking);
  other.serial = "LHR-OTHER";
  rec.Observe(0, other, 1);
  rec.Observe(0, Update(0, TrackingState::Tracking), 1);
  rec.Observe(0.1, Update(1, TrackingState::Tracking), 1);
  rec.Observe(1, Update(0, TrackingState::Coasting), 1);
  rec.Observe(2, Update(0, TrackingState::Tracking), 1);
  Session::Receiver r;
  r.connected = serials[0];
  rec.Observe(3, r);
  SessionEvent event;
  event.serial = r.connected;
  event.text = "filter reset";
  rec.Observe(4, event);
  rec.Observe(6, Update(0, TrackingState::Lost), 1);
  rec.Observe(7, Update(0, TrackingState::Tracking), 1);
  EXPECT_FALSE(rec.Finish(10.9));
  auto clip = rec.Finish(11);
  ASSERT_TRUE(clip);
  EXPECT_EQ(clip->trigger, 6);
  const auto dir = std::filesystem::temp_directory_path() / ("tracking-incident-test-" + std::to_string(platform::ProcessId()));
  const auto file = openvr::SaveTrackingIncident(dir, *clip);
  std::ifstream in(file);
  std::vector<nlohmann::json> rows;
  for (std::string line; std::getline(in, line);) rows.push_back(nlohmann::json::parse(line));
  ASSERT_EQ(rows.size(), 10u);
  EXPECT_EQ(rows[1]["t"], 0);
  EXPECT_EQ(rows[2]["serial"], "LHR-B");
  EXPECT_EQ(rows[5]["type"], "radio");
  EXPECT_EQ(rows[6]["message"], "filter reset");
  EXPECT_EQ(rows[8]["reason"], "repeated_tracking_loss");
  EXPECT_EQ(rows.back()["state"], "tracking");
  in.close();
  std::filesystem::remove(file);
  std::filesystem::remove(dir);
  EXPECT_FALSE(rec.Finish(20));
}

TEST(TrackingIncidents, MotionCompensatedJumpCanTriggerWhileTracking) {
  TrackingIncidents rec(serials);
  auto u = Update(1, TrackingState::Tracking);
  u.pose.velocity[0] = 4;
  rec.Observe(0, u, 1);
  u.pose.hostTime = 0.02;
  u.pose.worldFromImu.position[0] = 0.08;
  rec.Observe(0.02, u, 1);
  EXPECT_FALSE(rec.Finish(0.02, true));
  u.pose.hostTime = 0.04;
  u.pose.worldFromImu.position[0] = 0.3;
  rec.Observe(0.04, u, 1);
  auto clip = rec.Finish(0.04, true);
  ASSERT_TRUE(clip);
  EXPECT_EQ(std::get<TrackingIncidents::Marker>(clip->rows.back().value).reason, "position_jump");
}
}  // namespace
