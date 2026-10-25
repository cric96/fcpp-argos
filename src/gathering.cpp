// MOVING: the swarm elects a leader and gathers around it.
// Robots cannot see the leader: each one computes its vector to the leader as
// "vector to my parent (nbr_vec) + my parent's vector to the leader" along the distance gradient.
#include "fcpp_argos.hpp"

namespace fcpp::coordination::gathering {

MAIN() {
    bool leader = wave_election(CALL) == node.uid;
    real_t dist = abf_distance(CALL, leader);
    device_t parent = get<1>(min_hood(CALL, make_tuple(nbr(CALL, dist), nbr_uid(CALL))));
    vec<2> to_leader = nbr(CALL, vec<2>{}, [&](field<vec<2>> x) {
        // computed by everybody: a hood operation only sees neighbours that executed it too
        vec<2> via_parent = sum_hood(CALL, mux(nbr_uid(CALL) == parent, x + node.nbr_vec(), vec<2>{}), vec<2>{});
        return leader ? vec<2>{} : via_parent;
    });
    // keep 0.35 m from everybody (spring that only pushes)
    vec<2> separation = sum_hood(CALL, map_hood([](vec<2> v) {
        real_t d = norm(v);
        return d > 0 and d < 0.35 ? v * (1 - 0.35 / d) : vec<2>{};
    }, node.nbr_vec()), vec<2>{});
    node.storage(tags::velocity{}) = leader ? vec<2>{} : 0.3 * to_leader + 2 * separation + 0.05 * node.storage(tags::obstacle{});
    node.storage(tags::led{}) = leader ? color(RED) : color::hsva(std::min<real_t>(dist, 3) * 90 + 60, 1, 1);
    node.storage(tags::label{}) = dist < INF ? std::to_string(int(norm(to_leader) * 100)) + "cm" : "?";
}
FUN_EXPORT main_t = common::export_list<wave_election_t<>, abf_distance_t, real_t, vec<2>>;

}

using CGathering = TFcppController<fcpp::coordination::gathering::main, fcpp::coordination::gathering::main_t>;
REGISTER_CONTROLLER(CGathering, "fcpp_gathering_controller")
