#include <memory>
#include <vector>

#include <provant_simulator_controller/controller.hpp>
#include <provant_simulator_controller/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
class DeterministicController final : public provant::IController
{
protected:
  bool config() override
  {
    return true;
  }

  std::vector<double> execute(
    const std::vector<double> & states,
    const std::vector<double> & references) override
  {
    last_states_ = states;
    last_references_ = references;

    const double state = states.empty() ? 0.0 : states.front();
    const double reference = references.empty() ? 0.0 : references.front();
    last_error_ = {reference - state};
    return last_error_;
  }

  void reset() override
  {
    last_states_.clear();
    last_references_.clear();
    last_error_.clear();
  }

  std::vector<double> getErrorVector() const override
  {
    return last_error_;
  }

  std::vector<double> getReferences() const override
  {
    return last_references_;
  }

  std::vector<double> getStateVector() const override
  {
    return last_states_;
  }

private:
  std::vector<double> last_states_;
  std::vector<double> last_references_;
  std::vector<double> last_error_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<provant::Controller>(
    rclcpp::NodeOptions{},
    std::make_unique<DeterministicController>());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
