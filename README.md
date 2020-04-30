## Terrain Amplification with Implicit 3D Features

<img src="https://aparis69.github.io/public_html/imgs/implicitTerrain_representative.jpg"
     alt="3D terrains - representative image"
     style="float: left; margin: 5px;" />

Source code for some of the results shown in the paper "Terrain Amplification with Implicit 3D Features" published in ACM-TOG in 2019 
and presented at Siggraph Asia 2019. This is aimed at researchers, students or profesionnals who may want to reproduce **some** of the results described in the paper.
[Click here for more information about the project](https://aparis69.github.io/public_html/projects/paris2019_3D.html).

### Important notes
* This code is **not** the one which produced the scenes seen in the paper. Everything has been *recoded* on my side to make sure it is free to use. The original code from the paper is dependent on internal libraries of my team. Hence, the results as well as the timings may differ from the ones in the paper.
* This is **research** code provided without any warranty. However, if you have any problem you can still send me an email or create an issue.

### Testing
There is no dependency. Running the program will output 3 .obj files which can then be visualized in another application (Blender, MeshLab). Tests have been made on:
* Visual Studio 2017: double click on the solution in ./VS2017/ and Ctrl + F5 to run
* Visual Studio 2019: double click on the solution in ./VS2019/ and Ctrl + F5 to run
* Ubuntu 16.04: cd ./G++/ && make && make run

In you can't compile or run the code, the resulting obj files are available in the Objs/ folder in the repo.

### Citation
You can use this code in any way you want, however please credit the original article:
```
@article{Paris2019T3D,
	author = {Paris, Axel and Galin, Eric and Peytavie, Adrien and Gu{\'e}rin, Eric and Gain, James},
	title = {Terrain Amplification with Implicit 3D Features},
	journal = {ACM Trans. Graph.},
	volume = {38},
	number = {5},
	year = {2019},
	pages = {147:1--147:15},
}
```	

### Missing
There are still some things missing from the paper implementation. They might be added in the future if someone is interested. What is not in the code:
* Optimized Marching-cubes
* Hoodoos shape-grammar growth process
