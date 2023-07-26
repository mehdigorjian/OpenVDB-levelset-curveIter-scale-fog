#include <cstddef>
#include <string>
#include <vector>

#include "Viewer.h"
#include "loop_subdiv.h"

#include <igl/readOBJ.h>
#include <igl/readSTL.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/writeOBJ.h>

#include <openvdb/io/File.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/ChangeBackground.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/LevelSetFilter.h>
#include <openvdb/tools/LevelSetRebuild.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>

// file path: ../data/obj_hw1.obj

void subdiv_to_levelset(const char *filename, float voxelSize, float halfWidth)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    // call igl::readOBJ to load the .obj file
    if (!igl::readOBJ(filename, V, F))
    {
        std::cerr << "Failed to load " << filename << std::endl;
        return;
    }

    // convert Eigen::MatrixXd to std::vector<openvdb::Vec3s>
    std::vector<openvdb::Vec3s> points;
    for (int i = 0; i < V.rows(); ++i)
    {
        points.push_back(openvdb::Vec3s(V(i, 0), V(i, 1), V(i, 2)));
    }

    // convert Eigen::MatrixXi to std::vector<openvdb::Vec3I>
    std::vector<openvdb::Vec3I> triangles;
    for (int i = 0; i < F.rows(); ++i)
    {
        triangles.push_back(openvdb::Vec3I(F(i, 0), F(i, 1), F(i, 2)));
    }

    // define transform which maps from voxel space to world space
    auto transform = openvdb::math::Transform::createLinearTransform(voxelSize);

    // convert the mesh to a level-set
    auto grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(*transform, points, triangles, halfWidth);

    grid->setName("subdiv_to_levelset");
    openvdb::io::File file("../data/subdiv_to_levelset.vdb");
    openvdb::GridPtrVec grids;
    grids.push_back(grid);
    file.write(grids);
    file.close();
    std::cout << "subdivision to level-set .vdb file created successfully!" << std::endl;
}

void mean_curve_iter(const char *filename, int num_of_iter)
{
    // std::string filename;
    // std::cout << "Enter '.vdb' file path: ";
    // std::cin >> filename;

    // const char *filename = "../data/subdiv_to_levelset.vdb";
    openvdb::FloatGrid::Ptr floatGrid;

    openvdb::io::File file(filename);
    if (!file.open())
    {
        std::cerr << "Error opening file " << filename << std::endl;
        return;
    }

    // loop over all grids
    for (openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter)
    {
        // getting the grid pointer
        openvdb::GridBase::Ptr baseGrid = file.readGrid(nameIter.gridName());

        if (baseGrid->isType<openvdb::FloatGrid>())
        {
            floatGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
        }
    }
    file.close();

    // creating level set filter
    openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*floatGrid);

    // allpying 20 iterations mean curvature
    for (int i = 0; i < num_of_iter; ++i)
    {
        filter.meanCurvature();
    }

    floatGrid->setName("mean_curve_iter");
    openvdb::io::File outFile("../data/mean_curve_iter.vdb");
    openvdb::GridPtrVec grids;
    grids.push_back(floatGrid);
    outFile.write(grids);
    outFile.close();
    std::cout << "mean curvature filtered .vdb file created successfully!" << std::endl;
}

void z_scale(const char *filename, float scale)
{
    openvdb::io::File file(filename);

    if (!file.open())
    {
        std::cerr << "Error opening file " << filename << std::endl;
        return;
    }

    openvdb::GridBase::Ptr baseGrid;
    for (openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter)
    {
        baseGrid = file.readGrid(nameIter.gridName());
        break;
    }

    file.close();

    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);

    // create a transform scales by 1.3 in the z-direction
    openvdb::math::Mat4d matrix = openvdb::math::Mat4d::identity();
    matrix.preScale(openvdb::math::Vec3d(1.0, 1.0, 1.3));
    openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(matrix);
    grid->setTransform(transform);

    // rebuild the level-set
    openvdb::FloatGrid::Ptr grid1 = openvdb::gridPtrCast<openvdb::FloatGrid>(grid);
    openvdb::GridBase::Ptr baseLevelSetGrid = openvdb::tools::levelSetRebuild(*grid1, /*isovalue=*/0.0, /*halfWidth=*/3.0);
    openvdb::FloatGrid::Ptr levelSetGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseLevelSetGrid);

    levelSetGrid->setName("z_scale");
    openvdb::io::File outFile("../data/z_scale.vdb");
    openvdb::GridPtrVec grids;
    grids.push_back(levelSetGrid);
    outFile.write(grids);
    outFile.close();
    std::cout << "z-scale .vdb file created successfully!" << std::endl;
}

void fog_from_level_set(const char *filename, float active_value)
{
    // const char *filename = "../data/subdiv_to_levelset.vdb";
    openvdb::io::File file(filename);

    if (!file.open())
    {
        std::cerr << "Error opening file " << filename << std::endl;
        return;
    }

    // Read the first grid from the file
    openvdb::GridBase::Ptr baseGrid;
    for (openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter)
    {
        baseGrid = file.readGrid(nameIter.gridName());
        break;
    }

    file.close();

    // Cast the grid to a FloatGrid
    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);

    // convert the level-set to fog volume
    float maxVal = std::numeric_limits<float>::lowest();
    for (auto iter = grid->cbeginValueOn(); iter; ++iter)
    {
        if (*iter > maxVal)
            maxVal = *iter;
    }

    openvdb::tools::sdfToFogVolume(*grid, maxVal);

    const float outside = grid->background();
    const float width = 2.0 * outside;

    // update all grid's active values, related to the voxels on the narrow band.
    for (openvdb::FloatGrid::ValueOnIter iter = grid->beginValueOn(); iter; ++iter)
    {
        float dist = iter.getValue() * active_value;
        iter.setValue((outside - dist) / width);
    }
    // visit all grid's inactive tile and voxel values and update the values related to the interior region.
    for (openvdb::FloatGrid::ValueOffIter iter = grid->beginValueOff(); iter; ++iter)
    {
        if (iter.getValue() < 0.0)
        {
            iter.setValue(1.0);
            iter.setValueOff();
        }
    }

    // set exterior voxels to 0.
    openvdb::tools::changeBackground(grid->tree(), 0.0);

    // get the number of active tiles
    size_t activeTileCount = grid->tree().activeTileCount();
    std::cout << "Active tile count: " << activeTileCount << std::endl;

    // save the fog volume grid to a file
    grid->setName("fog_from_level_set");
    openvdb::io::File outFile("../data/fog_from_level_set.vdb");
    openvdb::GridPtrVec grids;
    grids.push_back(grid);
    outFile.write(grids);
    outFile.close();
    std::cout << "fog_from_level_set .vdb file created successfully!" << std::endl;
}

int main(int argc, char *argv[])
{
    openvdb::initialize();
    // Convert the subdivided model into an OpenVDB float grid level set with voxel size 0.01 and half width 3.0
    subdiv_to_levelset("../data/subdiv_obj_hw1.obj", 0.01f, 3.0f);

    // Apply 20 iterations of mean curvature filtering to the level set
    mean_curve_iter("../data/subdiv_to_levelset.vdb", 20);

    // Scale the level set by a factor of 1.3 in the z direction only, yielding a level set that still has half width 3.0
    z_scale("../data/subdiv_to_levelset.vdb", 1.3);

    // Create a fog volume from this level set
    // For each voxel in the fog volume, multiply each active value by 3, clamping values to lie between 0 and 1
    // This should be an in-place operation
    // Print the number of active tiles in the fog volume
    fog_from_level_set("../data/subdiv_to_levelset.vdb", 3.0);

    return 0;
}
