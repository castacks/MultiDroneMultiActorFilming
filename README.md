# Multi-Drone Multi-Actor Filming

[Multi-Robot Planning for Filming Groups of Moving Actors Leveraging Submodularity and Pixel Density](https://arxiv.org/abs/2404.03103)

This package provides implementations of multi-robot planners for
filming groups of moving actors (see reference details).
This repo is based on the
[MultiAgentSensing](https://github.com/mcorah/MultiAgentSensing) package
which provides implementations and examples of submodular maximization
algorithms.

Our approach solves challenging view planning problems like the one below where
groups may frequently split, join, and reassemble, and we do so via a
combination of submodular maximization, value iteration, and a view reward based
on pixel densities over actors surfaces:

<img src="media/filming_example.png" alt="filming example" width="80%"/>

## Installation

The easiest way to run this code is to use the provided Dockerfile and scripts.
To do this one must first install [docker](https://www.docker.com/)

To build
* `cd docker-scripts`
* `sudo ./build`

To get a shell
* `sudo ./start`

Once in a shell, one can run/interact with the multi-agent model MDMA in the Julia interpreter.

For example:
* `julia`
* `julia> using MDMA`

To run all the experiments and generate all outputs
* `julia> conf = ExperimentsConfig("./experiments")`
* `julia> run_all_experiments(conf)`

ExperimentsConfig can also be provided a list of experiments to run (in the case you do not want to run everything).
For example, to run only the `cluster` experiment you can do
* `julia> conf = ExperimentsConfig("./experiments", ["cluster"])`

The repo comes with a set of solutions already in the correct locations. You can also try
to render all the image outputs, and compute solution evaluations without recomputing solutions.
* `julia> blender_render_all_experiments(conf)`
* `julia> evaluate_all_experiments(conf)`


TODO: Explain how to navigate the experiment output.

## Example code

TODO: Add a smaller example

## References

If you use this package in published work, pleace consider citing either of the
following:

```
@inproceedings{hughes2024cdc,
  title = {Multi-Robot Planning for Filming Groups of Moving Actors Leveraging
           Submodularity and Pixel Density},
  booktitle={Proc. of the {IEEE} Conf. on Decision and Control},
  author = {Hughes, Skyler and Rebecca Martin and Corah, Micah and Scherer, Sebastian},
  year = {2024},
  month=dec
}
```
