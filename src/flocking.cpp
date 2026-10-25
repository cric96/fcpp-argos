// ADVANCED: Reynolds flocking as an aggregate program.
//   separation: push away from neighbours closer than 0.6 m   (nbr_vec)
//   cohesion:   move towards the neighbourhood centre         (mean of nbr_vec)
//   alignment:  head where the neighbours head                 (mean of nbr(heading), headings from the compass)
// LED hue = heading, so a single colour means the flock is aligned.
#include "fcpp_argos.hpp"

namespace fcpp::coordination::flocking {

MAIN() {
    vec<2> heading = node.storage(tags::heading{});
    vec<2> separation = sum_hood(CALL, map_hood([](vec<2> v) {
        real_t d = norm(v);
        return d > 0 and d < 0.6 ? -v / (d * d) : vec<2>{};
    }, node.nbr_vec()), vec<2>{});
    vec<2> cohesion = mean_hood(CALL, node.nbr_vec(), vec<2>{});
    vec<2> alignment = mean_hood(CALL, nbr(CALL, heading));
    vec<2> steer = alignment + cohesion + 0.5 * separation + 0.5 * node.storage(tags::obstacle{});
    node.storage(tags::velocity{}) = norm(steer) > 0 ? steer / norm(steer) * 0.08 : vec<2>{}; // cruise at 8 cm/s
    node.storage(tags::led{}) = color::hsva(std::atan2(heading[1], heading[0]) * 180 / M_PI + 180, 1, 1);
}
FUN_EXPORT main_t = common::export_list<vec<2>>;

}

using CFlocking = TFcppController<fcpp::coordination::flocking::main, fcpp::coordination::flocking::main_t>;
REGISTER_CONTROLLER(CFlocking, "fcpp_flocking_controller")
