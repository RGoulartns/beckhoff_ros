#include "beckhoff/transport.hpp"
#include <stdexcept>
namespace beckhoff {
std::unique_ptr<Transport> makeAdsTransport(const AdsOptions&) {
    throw std::runtime_error("ADS transport disabled; build with BECKHOFF_PLC_WITH_ADS=ON or inject a Transport");
}
}
