#!/bin/bash
# Runs every experiment headless for 30 simulated seconds, then checks that in the election
# experiment all 40 robots agree on the swarm size. Run after building: scripts/smoke.sh
set -euo pipefail
cd "$(dirname "$0")/.."

for f in experiments/*.argos; do
    name=$(basename "$f" .argos)
    sed 's/length="0"/length="30"/' "$f" > "build/$name.argos"
    if ! argos3 -z -n -c "build/$name.argos" > "build/$name.log" 2>&1 || grep -q "exceeds rab_data_size" "build/$name.log"; then
        cat "build/$name.log"
        echo "FAIL $name"
        exit 1
    fi
    echo "ok   $name"
done

agree=$(grep -c "final: [0-9]*/40$" build/election.log || true)
[ "$agree" -eq 40 ] || { grep "final:" build/election.log; echo "FAIL election: only $agree/40 robots count 40"; exit 1; }
echo "ok   election consensus (40/40)"
