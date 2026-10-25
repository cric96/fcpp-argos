// STATIC: the swarm elects a leader (lowest uid), counts itself towards it and spreads the count back.
// Classic gradient-collection-broadcast pipeline, without any designated source.
#include "fcpp_argos.hpp"

namespace fcpp::coordination::election {

MAIN() {
    bool leader = wave_election(CALL) == node.uid;
    hops_t hops = abf_hops(CALL, leader);
    int count = sp_collection(CALL, hops, 1, 0, [](int x, int y) { return x + y; }); // complete at the leader
    int total = broadcast(CALL, hops, count);                                         // leader's value, everywhere
    node.storage(tags::led{}) = leader ? color(RED) : color::hsva(60 + 30 * std::min<hops_t>(hops, 7), 1, 1);
    node.storage(tags::label{}) = std::to_string(hops) + "/" + std::to_string(total);
}
FUN_EXPORT main_t = common::export_list<wave_election_t<>, abf_hops_t, sp_collection_t<hops_t, int>, broadcast_t<hops_t, int>>;

}

using CElection = TFcppController<fcpp::coordination::election::main, fcpp::coordination::election::main_t>;
REGISTER_CONTROLLER(CElection, "fcpp_election_controller")
