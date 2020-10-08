## Modeling Rocky Scenery using Implicit Blocks

<img src="https://aparis69.github.io/public_html/imgs/blocks_representative.jpg"
     alt="3D terrains - representative image"
     style="float: left; margin: 5px;" />

Source code for some of the results shown in the paper "Modeling Rocky Scenery using Implicit Blocks" published in TVC in 2020 
and presented at Computer Graphics International 2020. This is aimed at researchers, students or profesionnals who may want to reproduce **some** of the results described in the paper.
[Click here for more information about the project](https://aparis69.github.io/public_html/projects/paris2020_Blocks.html).

### Important notes
* This code is **not** the one which produced the scenes seen in the paper. Everything has been *recoded* on my side to make sure it is free to use. The original code from the paper is dependent on internal libraries of my team. Hence, the results as well as the timings may differ from the ones in the paper.
* This is **research** code provided without any warranty. However, if you have any problem you can still send me an email or create an issue.

### Testing
There is no dependency. Running the program will output 4 .obj files which can then be visualized in another application (Blender, MeshLab). Tests have been made on:
* Visual Studio 2019: double click on the solution in ./VS2019/ and Ctrl + F5 to run
* Ubuntu 16.04: cd ./G++/ && make && ./Out/RockFracturing

In you can't compile or run the code, the resulting obj files are available in the Objs/ folder in the repo.

### Citation
You can use this code in any way you want, however please credit the original article:
```
@article{Paris2020Blocks,
  	author = {Paris, Axel and Peytavie, Adrien and Gu{\'e}rin, Eric and Dischler, J-M and Galin, Eric},
  	title = {Modeling Rocky Scenery using Implicit Blocks},
  	journal = {The Visual Computer},
  	volume = {36},
  	number = {10},
  	year = {2020},
  	pages = {1432--2315},
}
```	

### Missing
There are still some things missing from the paper implementation. They might be added in the future if someone is interested. What is not in the code:
* Gradient-based warping operator (soon)
* Implicit replication operator
* Periodic and aperiodic tiling
* Some result scenes
