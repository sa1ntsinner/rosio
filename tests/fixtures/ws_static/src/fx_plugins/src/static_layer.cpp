#include "pluginlib/class_list_macros.hpp"

namespace fx_plugins
{
class StaticLayer
{
public:
  void onInitialize() {}
};
}  // namespace fx_plugins

// Wrapped across two lines on purpose: a single line regex misses this one.
PLUGINLIB_EXPORT_CLASS(
  fx_plugins::StaticLayer, fx_core::Layer)
