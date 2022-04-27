# Collective control of modular soft robots via embodied Spiking Neural Cellular Automata
This repository hosts the code and the supplementary materials for the paper "Collective control of modular soft robots via embodied Spiking Neural Cellular Automata", accepted at the ["Workshop on From Cells to Societies: Collective Learning across Scales"](https://sites.google.com/view/collective-learning) at ICLR 2022.

## Abstract
Voxel-based Soft Robots (VSRs) are a form of modular soft robots, composed of several deformable cubes, i.e., voxels.
Each VSR is thus an ensemble of simple agents, namely the voxels, which must cooperate to give rise to the overall VSR behavior.
Within this paradigm, collective intelligence plays a key role in enabling the emerge of coordination, as each voxel is independently controlled, exploiting only the local sensory information together with some knowledge passed from its direct neighbors (distributed or collective control).
In this work, we propose a novel form of collective control, influenced by Neural Cellular Automata (NCA) and based on the bio-inspired Spiking Neural Networks: the embodied Spiking NCA (SNCA).
We experiment with different variants of SNCA, and find them to be competitive with the state-of-the-art distributed controllers for the task of locomotion.
In addition, our findings show significant improvement with respect to the baseline in terms of adaptability to unforeseen environmental changes, which could be a determining factor for physical practicability of VSRs.

## Content
The content of this repository is organized into two main packages: 
- `hmsrobots`, taken from [2D highly modular soft robots](https://github.com/ericmedvet/2dhmsr), where all the elements required to perform the simulation of VSRs are included; and
- `evolution`, which contains all the components used to perform the evolutionary optimization, i.e., the *evolution*, of VSRs. This also includes a dependency to [JGEA](https://github.com/ericmedvet/jgea), a general evolutionary algorithm (EA) framework written in Java, used for actually performing the optimization part. The jar for JGEA is already included in the `lib` folder.

## Suggested usage
### Visualizing a VSR
To visualize a VSR performing locomotion you need to run the `Starter` class, which is included in the `hmsrobots` package.
This will start a simulation and will display a video of a three VSRs - a biped, a worm, and comb - moving downhill.
Note that in this case we are using non-optimized VSRs, hence most of their successful movement is caused by the inclination of the terrain.

### Performing an optimization
Evolutionary optimizations are performed by running the `LocomotionEvolution` class contained in `evolution`.
This will start the evolution of a distributed controller for VSRs with the default parameters (listed at the beginning of the method `run()`).
One can freely play with the parameters, which are mostly self-explanatory, either changing the source code or by using the corresponding key words from command line.

## Bibliography
Nadizar, Medvet, Nichele, Pontes-Filho. "Collective control of modular soft robots via embodied Spiking Neural Cellular Automata", Workshop on From Cells to Societies: Collective Learning across Scales (Cells2Societies@ICLR), 2022
```
@article{nadizar2022collective,
  title={Collective control of modular soft robots via embodied Spiking Neural Cellular Automata},
  author={Nadizar, Giorgia and Medvet, Eric and Nichele, Stefano and Pontes-Filho, Sidney},
  journal={arXiv preprint arXiv:2204.02099},
  year={2022}
}
```
