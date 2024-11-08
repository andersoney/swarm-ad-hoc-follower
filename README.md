# swarm-ad-hoc-follower

An algorithm for congestion control of robotic swarms in common target area problem when the robots do not know the algorithm executed by the others. 

# Relationship between the files

![File relations](fileRelationship.jpeg)

## Notes

The Ad Hoc Follower (AHF) is set by giving neighbourhoodAngle in config.ini different of zero. 

The Ad Hoc Mixed Teams (AHMT) -- or Mixed Teams (MT) -- are set by neighbourhoodAngle = 0 in config.ini.

## Dependency
Stage 
https://github.com/rtv/Stage

## Compiling
```sh
make
```

## Running an example
See test.sh.

## Running an example of experiments in batch
See experiment.sh.


