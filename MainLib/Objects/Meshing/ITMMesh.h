// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Scene/ITMVoxelBlockHash.h"
#include "../../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib
{
	class ITMMesh
	{
	public:
		struct Triangle {
		    Vector4f p0, p1, p2;

		};
		struct Normal {
            Vector4f n0, n1, n2;
		};

		struct Color {
		    Vector4f c0, c1, c2;
		};

		MemoryDeviceType memoryType;

		uint noTotalTriangles;
		static const uint noMaxTriangles_default = SDF_LOCAL_BLOCK_NUM * 32 * 16;
//        static const uint noMaxTriangles_default = SDF_LOCAL_BLOCK_NUM * 32;
		uint noMaxTriangles;
		bool hasNormal;
		bool hasColor;

		ORUtils::MemoryBlock<Triangle> *triangles;
		ORUtils::MemoryBlock<Normal> *normals;
		ORUtils::MemoryBlock<Color> *colors;

		explicit ITMMesh(MemoryDeviceType memoryType, bool computeNormal = true, bool labelColor = true, uint maxTriangles = noMaxTriangles_default)
		{
			this->memoryType = memoryType;
			this->noTotalTriangles = 0;
			this->noMaxTriangles = maxTriangles;
			this->hasNormal = computeNormal;
			this->hasColor = labelColor;

			triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
			if(this->hasNormal)
			    normals = new ORUtils::MemoryBlock<Normal>(noMaxTriangles, memoryType);
			else
			    normals = nullptr;
			if(this->hasColor)
			    colors = new ORUtils::MemoryBlock<Color>(noMaxTriangles, memoryType);
			else
			    colors = nullptr;
		}

		//TODO: Add Print Normal
		void WriteOBJ(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shoulDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "w+");
			if (f != NULL)
			{
				for (uint i = 0; i < noTotalTriangles; i++)
				{
					fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
				}

				for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shoulDelete) delete cpu_triangles;
		}

		void WriteSTL(const char *fileName)
		{
			ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
			if (memoryType == MEMORYDEVICE_CUDA)
			{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shoulDelete = true;
			}
			else cpu_triangles = triangles;

			Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

			FILE *f = fopen(fileName, "wb+");

			if (f != NULL) {
				for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

				fwrite(&noTotalTriangles, sizeof(int), 1, f);

				float zero = 0.0f; short attribute = 0;
				for (uint i = 0; i < noTotalTriangles; i++)
				{
					fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f); 
					fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

					fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
					fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

					fwrite(&attribute, sizeof(short), 1, f);

					//fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
					//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
				}

				//for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
				fclose(f);
			}

			if (shoulDelete) delete cpu_triangles;
		}



        void WritePLY(const char *fileName)
        {
		    bool write_binary = true;

            ORUtils::MemoryBlock<Triangle> *cpu_triangles = nullptr;
            ORUtils::MemoryBlock<Normal> *cpu_normals = nullptr;
            ORUtils::MemoryBlock<Color> *cpu_colors = nullptr;
            bool shoulDelete = false;
            if (memoryType == MEMORYDEVICE_CUDA)
            {
                cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
                cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
                if(hasNormal){
                    cpu_normals = new ORUtils::MemoryBlock<Normal>(noMaxTriangles, MEMORYDEVICE_CPU);
                    cpu_normals->SetFrom(normals, ORUtils::MemoryBlock<Normal>::CUDA_TO_CPU);
                }
                if(hasColor){
                    cpu_colors = new ORUtils::MemoryBlock<Color>(noMaxTriangles, MEMORYDEVICE_CPU);
                    cpu_colors->SetFrom(colors, ORUtils::MemoryBlock<Color>::CUDA_TO_CPU);
                }
                shoulDelete = true;
            }
            else
            {
                cpu_triangles = triangles;
                cpu_normals = normals;
                cpu_colors = colors;
            }

            Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);
            Normal *normalArray = nullptr;
            Color *colorArray = nullptr;


           // hasColor = false;
            if(hasNormal) normalArray = cpu_normals->GetData(MEMORYDEVICE_CPU);
            if(hasColor) colorArray = cpu_colors->GetData(MEMORYDEVICE_CPU);

            {
                std::fstream fout;
                fout.open(fileName, std::ios::out);
                if (fout.fail()){
                    std::string errmsg = "Cannot open file at ";
                    throw std::runtime_error(errmsg);
                }
                fout << "ply" << std::endl;
                if(!write_binary)
                    fout << "format ascii 1.0" << std::endl;
                else
                    fout << "format binary_little_endian 1.0" << std::endl;
                fout << "element vertex " << noTotalTriangles*3 << std::endl;
                fout << "property float x" << std::endl;
                fout << "property float y" << std::endl;
                fout << "property float z" << std::endl;
                if(colorArray != nullptr) {
                    fout << "property uchar red" << std::endl;
                    fout << "property uchar green" << std::endl;
                    fout << "property uchar blue" << std::endl;
                    fout << "property uchar alpha" << std::endl;
                }
                if(normalArray != nullptr) {
                    fout << "property float nx" << std::endl;
                    fout << "property float ny" << std::endl;
                    fout << "property float nz" << std::endl;
                    fout << "property float curvature" << std::endl;
                }
                fout << "element face " << noTotalTriangles << std::endl;
                fout << "property list uchar int vertex_indices" << std::endl;
                fout << "end_header" << std::endl;

//                auto checkColor = [] (ORUtils::Vector4<float> &vec) {
//                    bool good=true;
//                    for(size_t i=0;i<4;++i)
//                        if(std::isnan(vec[i]) || std::isinf(vec[i]) || vec[i] < 0 || vec[i] > 1) good=false;
//                    if(!good){
//                        printf("bad! ");
//                        for(auto a=0;a<4;++a)
//                            printf("%f ", vec[a]);
//                        printf("\n");
//                    }
//                };


                if(write_binary){
                    fout.close();
                    fout.open(fileName, std::ios::out | std::ios::binary | std::ios::app);
                }

                for (size_t i=0;i<noTotalTriangles; ++i){
                    const Triangle &triangle = triangleArray[i];
                    if(!write_binary) {
                        fout << triangle.p2.x << " " << triangle.p2.y << " " << triangle.p2.z;
                        if(hasColor)
                            fout << " "<< int (255*colorArray[i].c2.x) << " " << int (255*colorArray[i].c2.y) << " " << int (255*colorArray[i].c2.z) << " " << int (255*colorArray[i].c2.w);
                        if(hasNormal)
                            fout << " "<< normalArray[i].n2.x << " " << normalArray[i].n2.y << " " << normalArray[i].n2.z << " "   << normalArray[i].n2.w;
                        fout << "\n";

                        fout << triangle.p1.x << " " << triangle.p1.y << " " << triangle.p1.z;
                        if(hasColor)
                            fout << " "<< int (255*colorArray[i].c1.x) << " " << int (255*colorArray[i].c1.y) << " " << int (255*colorArray[i].c1.z) << " " << int (255*colorArray[i].c1.w);
                        if(hasNormal)
                            fout << " "<< normalArray[i].n1.x << " " << normalArray[i].n1.y << " " << normalArray[i].n1.z << " "   << normalArray[i].n1.w;
                        fout << "\n";
                        fout << triangle.p0.x << " " << triangle.p0.y << " " << triangle.p0.z;
                        if(hasColor)
                            fout << " "<< int (255*colorArray[i].c0.x) << " " << int (255*colorArray[i].c0.y) << " " << int (255*colorArray[i].c0.z) << " " << int (255*colorArray[i].c0.w);
                        if(hasNormal)
                            fout << " "<< normalArray[i].n0.x << " " << normalArray[i].n0.y << " " << normalArray[i].n0.z << " "   << normalArray[i].n0.w;
                        fout << "\n";
                    } else {
                        write_point<float>(fout,triangle.p2, 3);
                        if(hasColor)
                            write_color(fout, colorArray[i].c2);
                        if(hasNormal)
                            write_point<float>(fout,normalArray[i].n2, 4);

                        write_point<float>(fout,triangle.p1,3);
                        if(hasColor)
                            write_color(fout, colorArray[i].c1);
                        if(hasNormal)
                            write_point<float>(fout,normalArray[i].n1,4);

                        write_point<float>(fout,triangle.p0,3);
                        if(hasColor)
                            write_color(fout, colorArray[i].c0);
                        if(hasNormal)
                            write_point<float>(fout,normalArray[i].n0,4);

                    }



                   // checkColor(colorArray[i].c0);
                    //checkColor(colorArray[i].c1);
                    //checkColor(colorArray[i].c2);
                }

                if(!write_binary) {
                    for (size_t i = 0; i < noTotalTriangles; i++) {
                        fout << "3";
                        fout << " " << 3 * i;
                        fout << " " << 3 * i + 1;
                        fout << " " << 3 * i + 2;
                        fout << std::endl;
                    }
                } else {
                    for (size_t i = 0; i < noTotalTriangles; i++) {
                        write_binary_shortcut(fout, static_cast<unsigned char>(3));
                        write_binary_shortcut(fout, static_cast<int>(3*i));
                        write_binary_shortcut(fout, static_cast<int>(3*i+1));
                        write_binary_shortcut(fout, static_cast<int>(3*i+2));
                    }
                }

                fout.close();
            }
            if (shoulDelete) delete cpu_triangles;
        }

		~ITMMesh()
		{
			delete triangles;
			delete normals;
			delete colors;
		}

		// Suppress the default copy constructor and assignment operator
		ITMMesh(const ITMMesh&);
		ITMMesh& operator=(const ITMMesh&);
	private:
        template<class T> void write_binary_shortcut(std::fstream &file, T data){
            file.write(reinterpret_cast<char*>(&data),sizeof(T));
        }
        template <class T> void write_point(std::fstream &file, const ORUtils::Vector4<float> &point, size_t size){
            for(size_t i=0;i<size;++i)
                write_binary_shortcut(file, static_cast<T>(point[i]));
        }
        void write_color(std::fstream &file, const ORUtils::Vector4<float> &point){
            for(size_t i=0;i<4;++i){
                unsigned char t = std::max(0, std::min(255, int(255*point[i])));
                write_binary_shortcut(file, static_cast<unsigned char >(t));
            }
//            unsigned char r = std::max(0, std::min(255, int(255*point.x)));
//            unsigned char g = std::max(0, std::min(255, int(255*point.y)));
//            unsigned char b = std::max(0, std::min(255, int(255*point.z)));
//            unsigned char w = 255;
//            write_binary_shortcut(file, static_cast<unsigned char >(r));
//            write_binary_shortcut(file, static_cast<unsigned char >(g));
//            write_binary_shortcut(file, static_cast<unsigned char >(b));
//            write_binary_shortcut(file, static_cast<unsigned char >(w));
        }
	};
}
