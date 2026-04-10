#include <cstdint>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>

struct Event {
  uint64_t t;
  uint16_t x, y;
  uint8_t p;
};

using event_camera_codecs::EventPacket;

class MyProcessor : public event_camera_codecs::EventProcessor {
public:
  MyProcessor() { events.reserve(10000); }
  std::vector<Event> events;
  inline void eventCD(uint64_t t, uint16_t ex, uint16_t ey,
                      uint8_t polarity) override {
    events.push_back({t, ex, ey, polarity});
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void finished() override {
  }; // called after no more events decoded in this packet
  void rawData(const char *, size_t) override {}; // passthrough of raw data
};

class EventCameraNode : public rclcpp::Node {
private:
  MyProcessor processor_;
  event_camera_codecs::DecoderFactory<EventPacket, MyProcessor> decoder_factory;

  rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr sub_;
  std::uint64_t t_last_;
  std::uint64_t last_seqno_{0};

public:
  EventCameraNode() : Node("event_camera_node") {
    auto qos = rclcpp::QoS(10).best_effort();

    this->sub_ = create_subscription<event_camera_msgs::msg::EventPacket>(
        "/event_camera/events", qos,
        std::bind(&EventCameraNode::callback<false>, this,
                  std::placeholders::_1));
    this->t_last_ = 0;
  }

private:
  template <bool is_master>
  void
  callback(const event_camera_msgs::msg::EventPacket::ConstSharedPtr &msg) {
    if (last_seqno_ != 0 && msg->seq != last_seqno_ + 1) {
      RCLCPP_WARN(get_logger(), "seqno jump from %lu -> %lu", last_seqno_, msg->seq);
    }
    last_seqno_ = msg->seq;
    auto decoder = this->decoder_factory.getInstance(*msg);
    if (!decoder) {
      RCLCPP_WARN(get_logger(), "unknown encoding");
      return;
    }
    decoder->decode(*msg, &this->processor_);

    std::ranges::for_each(this->processor_.events, [this](const auto &event) {
      if (this->t_last_ != 0 && event.t < this->t_last_) {
        const uint64_t d = this->t_last_ - event.t;
        RCLCPP_WARN(get_logger(),
                    "timestamps are not monotonously increasing, (t_last=%lu, "
                    "t=%lu, d=%lu)",
                    t_last_, event.t, d);
      }
      this->t_last_ = event.t;
    });

    this->processor_.events.clear();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventCameraNode>());
  rclcpp::shutdown();
  return 0;
}
