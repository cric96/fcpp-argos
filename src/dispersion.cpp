// MOVING: robots start packed together and spread out until nobody is closer than 0.6 m.
// FCPP's library elastic force on nbr_vec, made push-only by giving zero strength to farther neighbours
// (a spring that also pulls would make the dense swarm collapse instead).
#include "fcpp_argos.hpp"

namespace fcpp::coordination::dispersion {

MAIN() {
    vec<2> force = neighbour_elastic_force(CALL, 0.6, mux(node.nbr_dist() < 0.6, 1.0, 0.0));
    node.storage(tags::velocity{}) = force + 0.05 * node.storage(tags::obstacle{});
    node.storage(tags::led{}) = norm(force) < 0.02 ? color(GREEN) : color(ORANGE); // green = settled
    node.storage(tags::label{}) = std::to_string(count_hood(CALL) - 1);                 // neighbours in range
}
FUN_EXPORT main_t = common::export_list<neighbour_elastic_force_t>;

}

using CDispersion = TFcppController<fcpp::coordination::dispersion::main, fcpp::coordination::dispersion::main_t>;
REGISTER_CONTROLLER(CDispersion, "fcpp_dispersion_controller")
