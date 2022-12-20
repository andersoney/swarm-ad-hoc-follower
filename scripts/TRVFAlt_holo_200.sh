hol=holo
for arg in `seq 20 20 200 | shuf`; do
  for j in `seq 1 9 | shuf`; do
    bash experiment.sh scripts/TRVFAlt/$hol/confSQF\_prob$arg\_$j.sh
  done
done
