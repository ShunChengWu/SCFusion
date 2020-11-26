#include <numeric>
#include <random>
#include "../densecrf.h"
#include <algorithm>

class DenseCRFSample : public DenseCRF {
public:
    DenseCRFSample(int N, int M, float spatial_stddev):
            DenseCRF(N,M), spatial_stddev_(spatial_stddev){

    }
    ~DenseCRFSample() = default;
    // Add a Gaussian pairwise potential with standard deviation sx and sy
    void addPairwiseGaussian(const float* surfel_data, float w, const std::vector<int>& valid){
        const int features = 3;
        const int surfel_size = 3;
        float * feature = new float [N_*features];
        for (int i=0; i<N_; i++) {
            const int id = valid[i];
            int idx = 0;
            feature[(i*features)+(idx++)] = surfel_data[(id * surfel_size) + 0] / spatial_stddev_;
            feature[(i*features)+(idx++)] = surfel_data[(id * surfel_size) + 1] / spatial_stddev_;
            feature[(i*features)+(idx++)] = surfel_data[(id * surfel_size) + 2] / spatial_stddev_;
        }
        addPairwiseEnergy(feature, features, w, nullptr);
        delete [] feature;
    }

    float* runInference( int n_iterations, float relax ) {
        startInference();
        for (int it=0; it<n_iterations; it++)
            stepInference(relax);
        return current_;
    }

    void startInference(){
        // Initialize using the unary energies
        expAndNormalize( current_, unary_, -1 );
    }
    void stepInference( float relax ){
        for( int i=0; i<N_*M_; i++ )
            next_[i] = -unary_[i];
        // Add up all pairwise potentials
        for( unsigned int i=0; i<pairwise_.size(); i++ )
            pairwise_[i]->apply( next_, current_, tmp_, M_ );
        // Exponentiate and normalize
        expAndNormalize( current_, next_, 1.0, relax );
    }
private:
    float spatial_stddev_;
};
struct Point{
    float x,y,z;
};

int main(int argc, char **argv) {
    int num_classes = 3;
    int num_points  = 5;
    float spatial_stddev = 0.05;
    float color_stddev = 20;
    float normal_stddev = 0.1;
    int iterations = 10;
    float max_components = 10;
    std::vector<Point> points {
            {0,0,0},
            {1, 0, 0},
            {1.2,0,0},
            {1.8,0,0},
            {2,0,0},
    };

    std::vector<float> prob_table(num_classes*max_components);
    std::default_random_engine rd(2019);
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distri(0,1);

    for(size_t i=0;i<num_points;++i) {
        float sum = 0;
        for (size_t j = 0; j < num_classes; ++j) {
            size_t idx = j * max_components + i;
            prob_table[idx] = distri(gen);
            sum += prob_table[idx];
        }
        // normalize
        for (size_t j = 0; j < num_classes; ++j) {
            size_t idx = j * max_components + i;
            prob_table[idx] /= sum;
        }

        printf("Point [%zu]\n", i);
        float max = 0; int max_idx = 0;
        for (size_t j = 0; j < num_classes; ++j) {
            printf("%5.3f ", prob_table[j * max_components + i]);
            if(prob_table[j * max_components + i] > max) {
                max = prob_table[j * max_components + i];
                max_idx = j;
            }
        }
        printf(". Class: %d\n", max_idx);
        printf("\n");
    }

    std::vector<int> valid_ids(num_points);
    std::iota(valid_ids.begin(),valid_ids.end(), 0);

    std::vector<float> unary_potentials(num_points * num_classes);
    for(int i = 0; i < num_points; ++i) {
//        int id = valid_ids[i];
        for (int j = 0; j < num_classes; ++j) {
            unary_potentials.at(i * num_classes + j) =
                    -log(prob_table.at(j * max_components + i) + 1.0e-12);
        }
    }

    DenseCRFSample crf(num_points, num_classes, spatial_stddev);
    crf.setUnaryEnergy(unary_potentials.data());
    crf.addPairwiseGaussian(&points[0].x,3,valid_ids);
    float* resulting_probs = crf.runInference(iterations,1.0f);


    for (int i = 0; i < num_points; ++i) {
        for (int j = 0; j < num_classes; ++j) {
//            const int id = valid_ids[i];
            // Sometimes it returns nan resulting probs... filter these out
            if (resulting_probs[i * num_classes + j] > 0.0 && resulting_probs[i * num_classes + j] < 1.0) {
                prob_table.at(j * max_components + i) = resulting_probs[i * num_classes + j];
            }
        }
    }

    printf("===After CRF===\n");
    for(size_t i=0;i<num_points;++i){
        float max = 0; int max_idx = 0;
        printf("Point [%zu]\n", i);
        for (size_t j = 0; j < num_classes; ++j) {
            printf("%5.3f ", prob_table[j * max_components + i]);
            if(prob_table[j * max_components + i] > max) {
                max = prob_table[j * max_components + i];
                max_idx = j;
            }
        }
        printf(". Class: %d\n", max_idx);
        printf("\n");
    }
//    const float* gpu_prob_table = class_probabilities_gpu_->gpu_data();
//    float* gpu_max_map = class_max_gpu_->mutable_gpu_data();
//    updateMaxClass(current_table_size_,gpu_prob_table,num_classes_,gpu_max_map,max_components_);
//    map->UpdateSurfelClassGpu(max_components_,class_max_gpu_->gpu_data(),class_max_gpu_->gpu_data() + max_components_,colour_threshold_);
//    delete [] my_surfels;

    return 0;
}