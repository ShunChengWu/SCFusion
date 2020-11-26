#include "assimp_obj_loader.h"
#include "../../ORUtils/Logging.h"

AssimpObjLoader::AssimpObjLoader(const std::string& objfileName, uint &instance_num,
                         std::string _wnid)
{
    wnid = _wnid;

    objectFileName = objfileName;

    Assimp::Importer importer;

    const aiScene *m_scene =importer.ReadFile( objfileName,
                                               aiProcess_Triangulate     |
                                               aiProcess_JoinIdenticalVertices  |
                                               aiProcess_FindInvalidData |
                                               aiProcess_CalcTangentSpace
    );
    assert(m_scene->mNumMaterials > 0);

    /// Get the contents of the obj file into the arrays
    /// 1. Find the number of total vertices.
    /// 2. Find the number of submeshes.
    /// Use this to initialise the arrays.
    min_x =  1E10;
    max_x = -1E10;

    min_y =  1E10;
    max_y = -1E10;

    min_z =  1E10;
    max_z = -1E10;

    total_vertices_floor = 0;
    total_faces_floor    = 0;

    total_faces    = 0;
    total_meshes   = 0;

    for(int i =0; i < (int)m_scene->mNumMeshes; i++)
    {
        aiMesh* mesh = m_scene->mMeshes[i];
//        auto materialIndex = mesh->mMaterialIndex;
//        SCLOG(VERBOSE) << "Mesh:"<<mesh->mName.C_Str();

        if (wnid.length() > 1 && !mesh->HasNormals())
        {
            continue;
        }

        std::string current_mesh_name(mesh->mName.C_Str());

        for(char & s : current_mesh_name)
        {
            s = std::tolower(s);
        }



//        if ( current_mesh_name.find("floor") != std::string::npos)
        {
            total_vertices_floor += (int)mesh->mNumVertices;
            total_faces_floor    += (int)mesh->mNumFaces;
        }

        // calculate valid faces
        for(unsigned int f = 0; f < mesh->mNumFaces; f++) {
            if(mesh->mFaces[f].mNumIndices==3)
                total_faces++;
        }
        //total_faces    += (int)mesh->mNumFaces;

        total_meshes++;
    }
//    SCLOG(VERBOSE) << "Number of Meshes loaded = " << total_meshes;


    vertex2face_floor  = std::vector< std::vector<int> >(total_vertices_floor);

    index2vertex_floor = std::vector< std::vector<float> > (total_vertices_floor);
    face2vertex_floor  = std::vector< std::vector<int> > (total_faces_floor);

    shape_vertices = std::vector<std::shared_ptr<float>>(total_meshes,nullptr);
    shape_normals  = std::vector<std::shared_ptr<float>>(total_meshes,nullptr);
    shape_vertice_label = std::vector<std::shared_ptr<unsigned int>>(total_meshes, nullptr);
    shape_vertice_instance = std::vector<std::shared_ptr<unsigned int>>(total_meshes, nullptr);
    number_of_vertices_x3_shape = std::vector<int>(total_meshes);

    all_vertices.reset(new float[total_faces*3*3]);
    all_normals.reset(new float[total_faces*3*3]);

    all_vertices_array_size = total_faces*3*3;
    all_normals_array_size  = total_faces*3*3;

    int vface = 0;
//    int face_no =0;
    int mesh_no = 0;

    /// offset for vertices
    unsigned int offset = 0;

    if(!_wnid.empty()) instance_num++;
    for(int i =0; i < (int)m_scene->mNumMeshes; i++)
    {
        aiMesh* mesh = m_scene->mMeshes[i];
        uint instance = _wnid.empty()?instance_num++:instance_num;

//        SCLOG(VERBOSE) << "mesh["<<i<<"]";
//        SCLOG(VERBOSE) << "Vertices: " << mesh->mNumVertices;
        auto* vertices_f3 = reinterpret_cast<aiVector3D*>( mesh->mVertices );
        //aiVector3D* normals_f3  = reinterpret_cast<aiVector3D*>( mesh->mNormals );
        for(size_t v=0;v<mesh->mNumVertices;++v){
            mvAllVertices.push_back(vertices_f3[v].x);
            mvAllVertices.push_back(vertices_f3[v].y);
            mvAllVertices.push_back(vertices_f3[v].z);
        }

        int count = 0;

        if (wnid.length() > 1 && !mesh->HasNormals())
        {
            continue;
        }

//        SCLOG(VERBOSE) << "mesh name = "<< mesh->mName.C_Str();
        std::string current_mesh_name(mesh->mName.C_Str());
        unsigned int label = 0;
        if(wnid.empty())
        {
            auto materialIndex = mesh->mMaterialIndex;
            aiMaterial *material = m_scene->mMaterials[materialIndex];
            aiString name;
            material->Get(AI_MATKEY_NAME, name);
            std::string strMat(name.C_Str());
            std::string lookup;
            auto found = strMat.find_first_of('.');
            if (found!=std::string::npos) {
                lookup = strMat.substr(0,found);
            } else {
                lookup = strMat;
            }
            std::transform(lookup.begin(), lookup.end(), lookup.begin(), ::tolower);

//            if(name_to_wnid.find(lookup) != name_to_wnid.end()) {
////                    printf("mesh[%d] has materialIndex [%d] which is [%s] that has wnid[%s] which is class[%d] \n",
////                           i, materialIndex, lookup.c_str(), name_to_wnid.at(lookup).c_str(),
////                           wnid_to_NYU13classid[name_to_wnid.at(lookup)]);
//                label = wnid_to_NYU13classid[name_to_wnid.at(lookup)];
//            } else
//                SCLOG(VERBOSE) << "mesh[" << i<<"] has mateirialIndex ["<<materialIndex<<"] which is [" << lookup << "] that cannot find corresponding wnid!";

        } else {
//            label = wnid_to_NYU13classid[wnid];
        }


        for(char & s : current_mesh_name)
        {
            s = std::tolower(s);
        }

        meshNames.push_back(current_mesh_name);


        shape_vertices[mesh_no].reset(new float[mesh->mNumFaces*3*3]);
        shape_vertice_label[mesh_no].reset(new unsigned int[mesh->mNumFaces]);
        shape_vertice_instance[mesh_no].reset(new unsigned int[mesh->mNumFaces]);
        shape_normals[mesh_no].reset( new float[mesh->mNumFaces*3*3] );

        for(unsigned int f = 0; f < mesh->mNumFaces; f++)
        {
            aiFace face = mesh->mFaces[f];
            shape_vertice_label[mesh_no].get()[f] = label;
            shape_vertice_instance[mesh_no].get()[f] = instance;

//            assert(face.mNumIndices==3);
            if(face.mNumIndices!=3){
//                SCLOG(DEBUG) << "meshid["<<m_scene->mNumMeshes<<"],face_id[" << f <<"] has non 3 indices! (" << face.mNumIndices << ")";
                continue;
            }
            unsigned int v1 = face.mIndices[0];
            unsigned int v2 = face.mIndices[1];
            unsigned int v3 = face.mIndices[2];

            auto numberValidation = [] (float value) ->bool {
                if(std::isnan(value)) return false;
                return !std::isinf(value);
            };

            if(!numberValidation(vertices_f3[v1].x)) continue;
            if(!numberValidation(vertices_f3[v1].y)) continue;
            if(!numberValidation(vertices_f3[v1].z)) continue;
            if(!numberValidation(vertices_f3[v2].x)) continue;
            if(!numberValidation(vertices_f3[v2].y)) continue;
            if(!numberValidation(vertices_f3[v2].z)) continue;
            if(!numberValidation(vertices_f3[v3].x)) continue;
            if(!numberValidation(vertices_f3[v3].y)) continue;
            if(!numberValidation(vertices_f3[v3].z)) continue;


            TooN::Vector<3>vertex_v1 = TooN::makeVector(vertices_f3[v1].x,
                                                        vertices_f3[v1].y,
                                                        vertices_f3[v1].z);

            TooN::Vector<3>vertex_v2 =  TooN::makeVector(vertices_f3[v2].x,
                                                         vertices_f3[v2].y,
                                                         vertices_f3[v2].z);


            TooN::Vector<3>vertex_v3 = TooN::makeVector(vertices_f3[v3].x,
                                                        vertices_f3[v3].y,
                                                        vertices_f3[v3].z);

            /// Compute the geometric normal

            TooN::Vector<3>v1v2 = vertex_v1 - vertex_v2;
            TooN::Vector<3>v3v2 = vertex_v3 - vertex_v2;

            TooN::Vector<3>normal_cross_product = v1v2 ^ v3v2;

            auto norm = (float) std::sqrt(normal_cross_product[0]*normal_cross_product[0] +
                                          normal_cross_product[1]*normal_cross_product[1] +
                                          normal_cross_product[2]*normal_cross_product[2]);

            normal_cross_product = normal_cross_product / norm;

            /// Populate the first vertex

            shape_vertices[mesh_no].get()[count+0] = vertex_v1[0];
            shape_vertices[mesh_no].get()[count+1] = vertex_v1[1];
            shape_vertices[mesh_no].get()[count+2] = vertex_v1[2];

            all_vertices.get()[vface+0] = vertices_f3[v1].x;
            all_vertices.get()[vface+1] = vertices_f3[v1].y;
            all_vertices.get()[vface+2] = vertices_f3[v1].z;

            /*
            norm = sqrt(normals_f3[v1].x*normals_f3[v1].x +
                        normals_f3[v1].y*normals_f3[v1].y +
                        normals_f3[v1].z*normals_f3[v1].z);
                        */

            all_normals.get()[vface+0] =  normal_cross_product[0];//(normals_f3[v1].x)/norm;
            all_normals.get()[vface+1] =  normal_cross_product[1];//(normals_f3[v1].y)/norm;
            all_normals.get()[vface+2] =  normal_cross_product[1];//(normals_f3[v1].z)/norm;
            vface+=3;

            count+=3;

            /// Populate the second vertex

            shape_vertices[mesh_no].get()[count+0] = vertex_v2[0];
            shape_vertices[mesh_no].get()[count+1] = vertex_v2[1];
            shape_vertices[mesh_no].get()[count+2] = vertex_v2[2];

            all_vertices.get()[vface+0] = vertices_f3[v2].x;
            all_vertices.get()[vface+1] = vertices_f3[v2].y;
            all_vertices.get()[vface+2] = vertices_f3[v2].z;

            /*
            norm = sqrt(normals_f3[v2].x*normals_f3[v2].x +
                        normals_f3[v2].y*normals_f3[v2].y +
                        normals_f3[v2].z*normals_f3[v2].z);
                        */

            all_normals.get()[vface+0]  =  normal_cross_product[0];//normals_f3[v2].x/norm;
            all_normals.get()[vface+1]  =  normal_cross_product[1];//normals_f3[v2].y/norm;
            all_normals.get()[vface+2]  =  normal_cross_product[2];//normals_f3[v2].z/norm;
            vface+=3;


            count+=3;


            /// Populate the third vertex
            shape_vertices[mesh_no].get()[count+0] = vertex_v3[0];
            shape_vertices[mesh_no].get()[count+1] = vertex_v3[1];
            shape_vertices[mesh_no].get()[count+2] = vertex_v3[2];

            all_vertices.get()[vface+0] = vertices_f3[v3].x;
            all_vertices.get()[vface+1] = vertices_f3[v3].y;
            all_vertices.get()[vface+2] = vertices_f3[v3].z;

            /*
            norm = sqrt(normals_f3[v3].x*normals_f3[v3].x +
                        normals_f3[v3].y*normals_f3[v3].y +
                        normals_f3[v3].z*normals_f3[v3].z);
                        */

            all_normals.get()[vface+0] =  normal_cross_product[0];//normals_f3[v3].x/norm;
            all_normals.get()[vface+1] =  normal_cross_product[1];//normals_f3[v3].y/norm;
            all_normals.get()[vface+2] =  normal_cross_product[2];//normals_f3[v3].z/norm;
            vface+=3;

            count+=3;


            std::vector<float>vertexX;
            vertexX.push_back(vertex_v1[0]);
            vertexX.push_back(vertex_v2[0]);
            vertexX.push_back(vertex_v3[0]);


            std::vector<float>vertexY;
            vertexY.push_back(vertex_v1[1]);
            vertexY.push_back(vertex_v2[1]);
            vertexY.push_back(vertex_v3[1]);


            std::vector<float>vertexZ;
            vertexZ.push_back(vertex_v1[2]);
            vertexZ.push_back(vertex_v2[2]);
            vertexZ.push_back(vertex_v3[2]);

            std::sort(vertexX.begin(),vertexX.end());
            std::sort(vertexY.begin(),vertexY.end());
            std::sort(vertexZ.begin(),vertexZ.end());

            if ( min_x > vertexX.at(0) )
                min_x = vertexX.at(0);
            else if ( max_x < vertexX.at(2) )
                max_x = vertexX.at(2);

            if ( min_y > vertexY.at(0) ) {
                min_y = vertexY.at(0);
            } else if ( max_y < vertexY.at(2) ) {
                max_y = vertexY.at(2);
            }

            if ( min_z > vertexZ.at(0) )
                min_z = vertexZ.at(0);
            else if ( max_z < vertexZ.at(2) )
                max_z = vertexZ.at(2);
        }
//        SCLOG(DEBUG) << "vertices count: " <<  count;
//        SCLOG(DEBUG) << "mesh->mNumVertices: " << mesh->mNumVertices;

        offset = offset + mesh->mNumVertices;

        number_of_vertices_x3_shape[mesh_no] = mesh->mNumFaces*3*3;
        mesh_no++;
    }
}