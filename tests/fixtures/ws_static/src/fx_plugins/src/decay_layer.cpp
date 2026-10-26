#include "pluginlib/class_list_macros.hpp"

namespace fx_plugins
{
class DecayLayer
{
public:
  void onInitialize() {}
};
}  // namespace fx_plugins

PLUGINLIB_EXPORT_CLASS(fx_plugins::DecayLayer, fx_core::Layer)
