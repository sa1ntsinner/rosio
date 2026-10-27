namespace deck
{
template <typename T>
T Deck::speed_cap()
{
  return declare_parameter<T>("speed_cap");
}
}  // namespace deck
