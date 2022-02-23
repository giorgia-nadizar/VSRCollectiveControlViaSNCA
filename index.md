# Abstract
Voxel-based Soft Robots (VSRs) are a form of modular soft robots, composed of several deformable cubes, i.e., voxels.
Each VSR is thus an ensemble of simple agents, namely the voxels, which must cooperate to give rise to the overall VSR behavior.
Within this paradigm, collective intelligence plays a key role in enabling the emerge of coordination, as each voxel is independently controlled, exploiting only the local sensory information together with some knowledge passed from its direct neighbors (distributed or collective control).
In this work, we propose a novel form of collective control, influenced by Neural Cellular Automata (NCA) and based on the bio-inspired Spiking Neural Networks: the embodied Spiking NCA (SNCA).
We experiment with different variants of SNCA, and find them to be competitive with the state-of-the-art distributed controllers for the task of locomotion.
In addition, our findings show significant improvement with respect to the baseline in terms of adaptability to unforeseen environmental changes, which could be a determining factor for physical practicability of VSRs.

# Videos
The following videos display optimized VSRs[^note] performing locomotion on a flat terrain for 30 simulated seconds.
In each video the first VSR is controlled with a U<del>D</del>-SNCA, the second with a UD-NCA, and the third with a <del>U</del>D-NCA.
We show three morphologies: biped, comb, and worm, as considered in the paper.

The biped exhibits interesting behaviors with all three types of controllers.
![Bipeds performing locomotion on flat terrain](gifs/biped.gif)

The same applies to the comb.
![Combs performing locomotion on flat terrain](gifs/comb.gif)

Conversely, the worm displays high frequency dynamics with UD-NCA and <del>U</del>D-NCA with lead to fast, yet reality-gap prone gaits.
When controlled with U<del>D</del>-SNCA this morphology is less effective, due to its low structural complexity.
![Worms performing locomotion on flat terrain](gifs/worm.gif)

[^note]: For every combination of VSR morphology and controller we performed 10 indipendent optimizations. Here we display the best result of each.
