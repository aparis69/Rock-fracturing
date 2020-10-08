/*
	This is an example implementation of some of the results described in the paper.
	Four scenes (in Four different files) can be run to export an .obj file.

	To minimize dependencies, no realtime viewer is provided. To compute the final isosurface, 
	a public domain marching-cube implementation is used.
	
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
	PointSet3 samples = PoissonSamplingBox(tile, 0.5f, 10000);

	// (2) Generate fracture distribution
	auto fractures = GenerateFractures(type, tile, 3);

	// (3) Clustering
	auto clusters = ComputeBlockClusters(samples, fractures);

	// (4) Implicit primitive extraction
	auto sdf = ComputeBlockSDF(clusters);

	// (4) Mesh extraction
	MC::mcMesh mesh = PolygonizeSDF(tile.Extended(Vector3(2.0f)), sdf);

	// Export in .obj file
	std::ofstream out;
	out.open(filename);
	if (out.is_open() == false)
		return;
	out << "g " << "Obj" << std::endl;
	for (size_t i = 0; i < mesh.vertices.size(); i++)
		out << "v " << mesh.vertices.at(i).x << " " << mesh.vertices.at(i).y << " " << mesh.vertices.at(i).z << '\n';
	for (size_t i = 0; i < mesh.vertices.size(); i++)
		out << "vn " << mesh.normals.at(i).x << " " << mesh.normals.at(i).y << " " << mesh.normals.at(i).z << '\n';
	for (size_t i = 0; i < mesh.indices.size(); i += 3)
	{
		out << "f " << mesh.indices.at(i) + 1 << "//" << mesh.indices.at(i) + 1
			<< " " << mesh.indices.at(i + 1) + 1 << "//" << mesh.indices.at(i + 1) + 1
			<< " " << mesh.indices.at(i + 2) + 1 << "//" << mesh.indices.at(i + 2) + 1
			<< '\n';
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
	srand((unsigned int)(time(NULL)));
	
	ComputeAndExportTile(FractureType::Equidimensional, "tile_equidimensional.obj");
	std::cout << "Equidimensional tile done" << std::endl;

	ComputeAndExportTile(FractureType::Rhombohedral,	"tile_rhombohedral.obj");
	std::cout << "Rhombohedral tile done" << std::endl;

	ComputeAndExportTile(FractureType::Polyhedral,		"tile_polyhedral.obj");
	std::cout << "Polyhedral tile done" << std::endl;

	ComputeAndExportTile(FractureType::Tabular,			"tile_tabular.obj");
	std::cout << "Tabular tile done" << std::endl;

	return 0;
}
