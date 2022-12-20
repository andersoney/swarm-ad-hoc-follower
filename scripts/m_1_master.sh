# Run all experiments for m = 1

DEBUGECHO="" 

for hol in holo nonholo; do
  for alg in SQF TRVF; do
    $DEBUGECHO bash experiment.sh scripts/NoCoordAlt/$hol/conf$alg\_m1.sh;
  done
done
