// STATIC: distance from a source robot, estimated hop by hop over the range-and-bearing network.
// The RAB range is FCPP's `nbr_dist()`, so the library gradient works unchanged.
#include "fcpp_argos.hpp"

namespace fcpp::coordination::gradient {

MAIN() {
    bool source = node.storage(tags::source{});
    real_t d = abf_distance(CALL, source);
    node.storage(tags::led{}) = source ? color(WHITE) : color::hsva(std::min<real_t>(d, 6) * 45, 1, 1);
    node.storage(tags::label{}) = d < INF ? std::to_string(int(d * 100)) + "cm" : "?";
}
FUN_EXPORT main_t = common::export_list<abf_distance_t>;

}

using CGradient = TFcppController<fcpp::coordination::gradient::main, fcpp::coordination::gradient::main_t>;
REGISTER_CONTROLLER(CGradient, "fcpp_gradient_controller")
