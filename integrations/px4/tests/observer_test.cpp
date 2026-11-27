// Offline tests against real generated SensorGps/protobuf types; no SITL launch.
#include "GpsAcquisitionObserver.hpp"
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
using namespace aerotwin_gps;
int main(int argc, char **argv)
{
 assert(argc == 3);
 const std::string mode = argv[1];
 if (mode == "queue") {
  auto q = std::make_unique<Queue>();
  Record r{}, out{};
  for (uint64_t i=0; i<Queue::Capacity; ++i) { assert(q->push(r)); }
  assert(!q->push(r));
  for (uint64_t i=0; i<Queue::Capacity; ++i) { assert(q->pop(out)); }
  assert(!q->pop(out)); assert(q->push(r)); assert(q->pop(out));
  std::atomic<uint64_t> successes{0}, losses{0};
  std::vector<std::thread> producers;
  for (unsigned j=0; j<4; ++j) { producers.emplace_back([&] {
   for (unsigned i=0; i<4096; ++i) { if(q->push(r)) { ++successes; } else { ++losses; } }
  }); }
  for(auto &t:producers) { t.join(); }
  uint64_t drained=0; while(q->pop(out)) { ++drained; }
  assert(drained == successes && drained <= Queue::Capacity);
  assert(successes + losses == 16384 && losses > 0);
  std::cout << "queue wrap/full and four-producer accounting PASS\n";
  return 0;
 }
 Observer observer;
 if (mode == "disabled") {
  unsetenv("AEROTWIN_GPS_OBSERVER"); assert(observer.start("/test"));
  assert(!observer.enabled() && !observer.begin().active); observer.stop(); return 0;
 }
 if (mode == "invalid") { assert(!observer.start("/test")); return 0; }
 assert(observer.start("/test"));
 gz::msgs::NavSat msg;
 msg.set_frame_id("gps");
 msg.mutable_header()->mutable_stamp()->set_sec(1);
 auto seq = msg.mutable_header()->add_data(); seq->set_key("seq"); seq->add_value("0");
 observer.finish(observer.begin(), msg, 1000000, nullptr, false, -1);
 sensor_gps_s gps{};
 gps.timestamp = gps.timestamp_sample = 1033000;
 gps.latitude_deg = 47.123456789123; gps.longitude_deg = 8.987654321987;
 gps.altitude_msl_m = gps.altitude_ellipsoid_m = 400.25;
 gps.vel_n_m_s = -0.0f; gps.vel_e_m_s = 0.1234567f; gps.vel_d_m_s = -1.25f;
 gps.device_id = 42; gps.fix_type = 3; gps.satellites_used = 12; gps.vel_ned_valid = true;
 const uint32_t negative_zero = 0x80000000U;
 memcpy(&gps.vel_n_m_s, &negative_zero, sizeof(negative_zero));
 unsigned char original[sizeof(gps)]; memcpy(original, &gps, sizeof(gps));
 unsigned count = mode == "overflow" ? 50000 : 3;
 for(unsigned i=1; i<count; ++i) {
  seq->set_value(0, std::to_string(i));
  msg.mutable_header()->mutable_stamp()->set_nsec(int32_t(i*33000000ULL % 1000000000ULL));
  msg.mutable_header()->mutable_stamp()->set_sec(1 + i*33000000ULL / 1000000000ULL);
  if(mode == "malformed") { seq->set_value(0, "18446744073709551616"); }
  observer.finish(observer.begin(), msg, gps.timestamp, &gps, true, 0);
 }
 assert(memcmp(original, &gps, sizeof(gps)) == 0);
 observer.stop(); assert(!observer.enabled());
 std::ofstream marker(argv[2]); marker << "actual SensorGps bytes and assigned timestamps unchanged\n";
 return 0;
}
