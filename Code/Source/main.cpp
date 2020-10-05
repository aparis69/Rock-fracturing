/*
	This is an example implementation of some of the results described in the paper.
	Three scenes (in three different files) can be run to export an .obj file.

	To minimize dependencies, no realtime viewer is provided, and all input heightfields are
	defined analytically with noise primitives. To compute the final isosurface, a public
	domain marching-cube implementation is used.
	
	If you have any questions or problem to compile the code, you can contact me at:
	axel(dot)paris(at)liris(dot)cnrs(dot)fr
*/

#include "blocks.h"
#include <fstream>

static void ComputeAndExportTile(FractureType t, const char* filename)
{
	FractureType type = t;
	const float tileSize = 20.0f;
	Box tile = Box(Vector3(0), tileSize / 2.0f);

	// (1) Sample a cubic tile
	PointSet3 samples = PoissonSamplingCube(tile, 0.5f, 10000);

	// (2) Generate fracture distribution
	auto fractures = GenerateFractures(type, tile, 3);

	// (3) Clustering
	auto clusters = ComputeBlockClusters(samples, fractures);

	// (4) Mesh extraction
	auto meshes = ComputeBlockMeshes(clusters);

	//
	// From there, we can either play with the generated meshes, or extract
	// Implicit primitives as we do in the paper. In this code, we only
	// Export the generated meshes so that you can visualize them in another Application.
	//

	// Export all blocks in the same .obj file
	std::ofstream out;
	out.open(filename);
	if (out.is_open() == false)
		return;
	out << "o " << "Obj" << std::endl;
	int meshOffset = 0;
	for (int i = 0; i < meshes.size(); i++)
	{
		for (int j = 0; j < meshes[i].triangles.size(); j++)
		{
			Vector3 n = meshes[i].triangles[j].Normal();
			for (int k = 0; k < 3; k++)
			{
				Vector3 p = meshes[i].triangles[j].Point(k);
				out << "v " << p.x << " " << p.y << " " << p.z << '\n';
				out << "vn " << n.x << " " << n.y << " " << n.z << '\n';
			}
			out << "f " << (j * 3) + meshOffset + 1 << "//" << (j * 3) + meshOffset + 1
				<< " " << (j * 3) + meshOffset + 2 << "//" << (j * 3) + meshOffset + 2
				<< " " << (j * 3) + meshOffset + 3 << "//" << (j * 3) + meshOffset + 3
				<< '\n';
		}
		meshOffset += meshes[i].triangles.size() * 3;
	}
	out.close();
}

/*!
\brief Running this program will export some
meshes similar to the ones seen in the paper. Each scene
is in its own file and contains all the algorithms necessary
to reproduce it.
*/
int main()
{
	srand(time(NULL));
	ComputeAndExportTile(FractureType::Equidimensional, "tile_equidimensional.obj");
	ComputeAndExportTile(FractureType::Rhombohedral,	"tile_rhombohedral.obj");
	ComputeAndExportTile(FractureType::Polyhedral,		"tile_polyhedral.obj");
	ComputeAndExportTile(FractureType::Tabular,			"tile_tabular.obj");
	return 0;
}
