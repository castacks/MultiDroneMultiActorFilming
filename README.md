# Multi-Agent Sensing

This package provides centralized implementations of Randomized Sequential
Partitions (RSP) (see references) that are suitable for numerical simulations as
well as a number of other algorithms for multi-agent planning via submodular
maximization such as sequential planning and auction algorithms.

Additionally, we implemented two application scenarios
* A simple coverage problem. Agent centers are distributed across the unit
  square. Actions are disks near the agent centers.
* A mutual information based target tracking problem. Agents plan on (short)
  receding horizons via Monte Carlo tree search.

## Installation

The easiest way would be to use the provided Dockerfile and scripts.

To build
* `cd docker-scripts`
* `sudo ./build`

To get a shell
* `sudo ./start`

To get a jupyter notebook
* `sudo ./run_jupyter`


## Example code

This package includes simple sensor coverage problems as well as receding
horizon target tracking and visual coverage problems.

For receding-horizon problems, press "enter" to advance the simulation.

* `example/coverage/coverage_test.jl`: This simulates run a static coverage
  problem and saves and image of the result.
  A group agents distributed on a rectangle can each choose to cover area under
  one of several nearby disks.
  The goal is to maximize the covered area.
* `example/target_tracking/multi_robot_example.jl`:
This example visualizes a group of robots "tracking" moving targets by
maintaining (common/centralized) Baye's filters based on noisy observations of
target distance.
The background is colored by probabilility density of target locations, averaged
across targets
* `example/target_tracking/multi_robot_coverage_example.jl`:
This example visualizes a group of robots seeking to "cover" moving targets by
keeping them inside the sensing range of any one robot

## References

If you use this package in published work, pleace consider citing either of the
following:

For the coverage scenario and the initial implementation of RSP:
```
@inproceedings{corah2018cdc,
  author = {Corah, Micah and Michael, Nathan},
  title={Distributed Submodular Maximization on Partition Matroids for Planning
         on Large Sensor Networks},
  booktitle={Proc. of the {IEEE} Conf. on Decision and Control},
  year = {2018},
  month = dec,
  address = {Miami, FL},
}
```

Please cite the thesis for the target tracking scenario and everything else:
```
@phdthesis{corah2020phd,
  author = {Corah, Micah},
  title = {Sensor Planning for Large Numbers of Robots},
  school = {Carnegie Mellon University},
  year = {2020}
}
```
