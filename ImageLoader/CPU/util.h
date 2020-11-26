//
// Created by sc on 11/25/20.
//

#ifndef SCSLAM_PROEJCT_UTIL_H
#define SCSLAM_PROEJCT_UTIL_H
#include <vector>
#include <string>
#include <fstream>
namespace SCFUSION {
    inline std::vector<float> LoadMatrixFromFile(const std::string &filename, int M, int N) {
        std::vector<float> matrix;
        std::fstream fp(filename, std::ios::in);
        if (!fp)
            throw std::runtime_error("LoadMatrixFromFile::Cannot open file for reading.\n");

        float tmp;
        while (fp >> tmp) {
            matrix.push_back(tmp);
        }
        fp.close();
        if (matrix.size() == 4) {
            std::vector<float> tmp;
            tmp.push_back(matrix[0]);
            tmp.push_back(0);
            tmp.push_back(matrix[2]);
            tmp.push_back(0);
            tmp.push_back(matrix[1]);
            tmp.push_back(matrix[3]);
            tmp.push_back(0);
            tmp.push_back(0);
            tmp.push_back(1);
            matrix.swap(tmp);
        } else if (matrix.size() < (size_t) M * N) {
            printf("Input format was not %d*%d matrix. matrix.size() = %zu\n", M, N, matrix.size());
            // The file is in different format.
            if (matrix.size() == 6 && M * N == 9) {
                std::vector<float> tmp;
                tmp.push_back(matrix[2]);
                tmp.push_back(0);
                tmp.push_back(matrix[4]);
                tmp.push_back(0);
                tmp.push_back(matrix[3]);
                tmp.push_back(matrix[5]);
                tmp.push_back(0);
                tmp.push_back(0);
                tmp.push_back(1);
                matrix.swap(tmp);
            } else
                throw std::runtime_error("Input format doesn't support!.\n");

        }

        return matrix;
    }
}
#endif //SCSLAM_PROEJCT_UTIL_H
