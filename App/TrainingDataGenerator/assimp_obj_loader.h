#pragma once

#include <iostream>

#include <TooN/TooN.h>


#include <assimp/vector3.h>
#include <assimp/types.h>
#include <assimp/cimport.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>

#include <set>
#include <map>
#include <vector>

//#include <vector_types.h>
//#include <vector_functions.h>

#include <fstream>
#include <boost/filesystem.hpp>

#include <TooN/se3.h>

//#include "../SceneNet_wnid_to_13labels.h"

using namespace std;

class AssimpObjLoader{

public:
    explicit AssimpObjLoader(const std::string& objfileName, uint &instance_num,
                    std::string _wnid = "");
    
    

    std::string get_obj_file_path()
    {
        return objectFileName;
    }

    std::string get_wnid()
    {
        return wnid;
    }

    int get_all_vertices_size()
    {
        return all_vertices_array_size;
    }

    int get_all_normals_size()
    {
        return all_normals_array_size;
    }

    std::shared_ptr<float> get_all_vertices()
    {
        return all_vertices;
    }

    std::shared_ptr<float> get_all_normals()
    {
        return all_normals;
    }

    std::vector<int> get_numVertes_submeshes()
    {
        return number_of_vertices_x3_shape;
    }

    std::vector<std::shared_ptr<float>> get_shape_vertices()
    {
        return shape_vertices;
    }
    std::vector<std::shared_ptr<unsigned int>> get_shape_vertices_label(){ return shape_vertice_label; }
    std::vector<std::shared_ptr<unsigned int>> get_shape_vertices_instance(){ return shape_vertice_instance; }

    int get_numMeshes()
    {
        return total_meshes;
    }

    std::vector<std::string>getMeshNames()
    {
        return meshNames;
    }

    std::vector<std::vector<float> > getPeripheralVertices()
    {
        return peripheral_vertices;
    }

    std::vector<std::vector<int> > get_face2vertex_floor()
    {
        return face2vertex_floor;
    }
    std::vector<std::vector<float> > get_index2vertex_floor()
    {
        return index2vertex_floor;
    }

    std::vector<float> get_min_max_3d_bounding_box()
    {
        std::vector<float>min_max_bb;

        min_max_bb.push_back(min_x);
        min_max_bb.push_back(max_x);

        min_max_bb.push_back(min_y);
        min_max_bb.push_back(max_y);

        min_max_bb.push_back(min_z);
        min_max_bb.push_back(max_z);

        return min_max_bb;
    }


    ~AssimpObjLoader(){}
public:
    std::shared_ptr<float> all_normals;
    std::shared_ptr<float>  all_vertices;

    int total_meshes;
    int total_faces;

    int total_vertices_floor;
    int total_faces_floor;

    int all_vertices_array_size;
    int all_normals_array_size;

    std::vector<std::shared_ptr<float>>shape_vertices;//(total_meshes,NULL);
    std::vector<std::shared_ptr<float>>shape_normals;//(total_meshes,NULL);
    std::vector<std::shared_ptr<unsigned int>> shape_vertice_label, shape_vertice_instance;

    std::vector<int>number_of_vertices_x3_shape;//(m_scene->mNumMeshes,0);


    std::vector<std::string>meshNames;

    std::vector< std::vector<int> >vertex2face_floor;//(total_vertices_floor);
    std::vector< std::vector<int> > face2vertex_floor;//(total_faces_floor);
    std::vector< std::vector<float> > index2vertex_floor;

//    std::vector< std::vector<flot> > vertices_floor;
    std::vector< std::vector<float> > peripheral_vertices;

    std::string wnid, objectFileName;

    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;

    std::vector<float> mvAllVertices;
};