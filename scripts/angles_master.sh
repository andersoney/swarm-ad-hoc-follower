# Run all experiments for angle of $\gamma_{ad}$-area

DEBUGECHO="" 

for hol in holo nonholo; do
  for alg in SQF TRVF; do
    for numTotal in 100 200; do
      for m in 1 5 9; do
        $DEBUGECHO bash experiment.sh scripts/NoCoordAlt/$hol/conf$alg\_angle\_$numTotal\_$m.sh;
      done
    done
  done
done
