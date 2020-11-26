#pragma once
#include "../../../DenseCRF/densecrf.h"
#include <assert.h>
#include "../../../ORUtils/Logging.h"
namespace SCFUSION {

    class DenseCRF3D_ : public DenseCRF {
    public:
        DenseCRF3D_(int N, int M, int F):
                DenseCRF(N,M), miFeatureSize(F){
            mvFeatureBuffer.resize(N_*miFeatureSize);
        }
        ~DenseCRF3D_() = default;
        // Add a Gaussian pairwise potential with standard deviation sx and sy
        void addPairwiseGaussian(const float* surfel_data, float w, const Vector3f &stddevs){
            for (int i=0; i<N_; i++) {
                for(size_t j=0;j<miFeatureSize;++j){
                    mvFeatureBuffer.at( i*miFeatureSize+j ) = surfel_data[ i*miFeatureSize+j ]/stddevs[j];
                }
            }
            addPairwiseEnergy(mvFeatureBuffer.data(), miFeatureSize, w, nullptr);
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
        size_t miFeatureSize;
        std::vector<float> mvFeatureBuffer;
    };

    class DenseCRFOptimization {
    public:
        DenseCRFOptimization(const Vector3i &dims, size_t labelNum):
                mTotalSize(dims.x*dims.y*dims.z), mLabelNum(labelNum), mIteration(5),
                mWeight(10), mRelexation(1.0),
                mDims(dims), mStddevs(3)
        {
            assert(mLabelNum>1);
            assert(mTotalSize>0);
            assert(mWeight>0);
        }

        void Optimize(float *label, float *conf, const bool *masks){
            bool bZeroUnsure = true;
//            int validLabelNum = int(mLabelNum)-1;
            CalculateValidIndex(label, masks, mTotalSize,mvValidIdx, bZeroUnsure);
            CalculateUnaryPotential(label,conf,mvValidIdx,mvPotentials,bZeroUnsure);
            CreatePairwise3D(mDims, mvValidIdx, mvPoints);

            mpDenseCRF.reset( new DenseCRF3D_(mvValidIdx.size(), mLabelNum-1, 3) );
            mpDenseCRF->setUnaryEnergy(mvPotentials.data());
            mpDenseCRF->addPairwiseGaussian(mvPoints.data(), mWeight, mStddevs);
            const float *resulting_probs = mpDenseCRF->runInference(mIteration,mRelexation);
            UpdateLabelAndConfidence(mLabelNum-1,resulting_probs, mvValidIdx, label, conf);
        }

        void SetWeight(float w) {mWeight = w;}
        void SetStddev(const Vector3f &stddevs) {mStddevs = stddevs;}
        void SetInterations(int value){mIteration=value;}
        void SetRelexation(float value){mRelexation=value;}

        const std::vector<size_t>& GetValidIdx(){return mvValidIdx;}
    private:
        size_t mTotalSize, mLabelNum, mIteration;
        float mWeight, mRelexation;
        Vector3i mDims;
        Vector3f mStddevs;
        std::vector<size_t> mvValidIdx;
        std::vector<float> mvPoints, mvPotentials;
        std::unique_ptr<DenseCRF3D_> mpDenseCRF;

        /**
         * If zeroUnsure is true, the labelNum should exclude label 0 (i.e. minus 1)
         * @param label
         * @param prob
         * @param labelNum
         * @param zeroUnsure
         * @return
         */
        std::vector<float> GetUnaryPotentialFromSingleLabelPropability(int label, float prob, int labelNum, bool zeroUnsure){
            assert(prob>0);
            assert(prob<1);
            auto p_energy = -log(prob);
            auto n_energy = -log((1.0 - prob) / (labelNum - 1));
            assert(!isnanf(p_energy));
            assert(!isnanf(n_energy));
            std::vector<float> probs(labelNum, n_energy);

            if(zeroUnsure) {
                if(label>0){
                    probs[label - 1] = p_energy; // -1 since to move 1 -> 0 since we don't want to include 0 label
                } else {
                    probs[0] = -log(1.0/labelNum); // set 0 to uniform distribution
                }
            } else {
                probs[label] = p_energy;
            }
            return probs;
        }

        template<typename T>
        void CalculateValidIndex(const T *data, const bool *masks, size_t size, std::vector<size_t> &validIdx, bool zeroUnsure){
            validIdx.clear();
            validIdx.reserve(size);
            for(size_t i=0; i < size;++i) {
                if (zeroUnsure) {
//                    if (masks[i] && data[i]>0.5) validIdx.push_back(i);
                    if (masks[i]) validIdx.push_back(i);
//                    if (data[i]>0.5) validIdx.push_back(i);
                } else {
                    if (masks[i]) validIdx.push_back(i);
                }
            }
        }
        template<typename T>
        void CalculateUnaryPotential(const T* data, const float *conf, const std::vector<size_t> &validIdx,
                                     std::vector<float> &unary_potentials, bool zeroUnsure){
            int labelNum = mLabelNum-1; // remove 0 which represent either unknown or unsure.
            size_t validIdxSize = validIdx.size();
            unary_potentials.clear();
            unary_potentials.reserve(validIdxSize*labelNum);
            for(size_t i=0;i<validIdxSize;++i){
                int idx = validIdx.at(i);
                auto label = (int)data[idx];
                auto prob  = conf[idx];//TODO: debug. see if the problem is due to weight
                auto tmp = GetUnaryPotentialFromSingleLabelPropability(label,prob,labelNum,zeroUnsure);
//                if(i==0){
//                    printf("lable[%d] Unary Potential: \n\t", int(data[i]));
//                    for(auto t:tmp)
//                        printf(" %f",t);
//                    printf("\n");
//                }
                unary_potentials.insert(unary_potentials.end(),
                                        std::move_iterator<std::vector<float>::iterator>(tmp.begin()),
                                        std::move_iterator<std::vector<float>::iterator>(tmp.end()));

            }
            assert(unary_potentials.size() == validIdxSize*labelNum);
        }
        void CreatePairwise3D(const Vector3i &dims, const std::vector<size_t> &validIdx,
                              std::vector<float> &points){
            points.clear();
            points.reserve(validIdx.size()*3);
            for(size_t i=0;i<validIdx.size();++i){
                int idx = validIdx.at(i);
                int z = idx / (dims.x * dims.y);
                int y = (idx % (dims.x * dims.y)) / dims.x;
                int x = (idx % (dims.x * dims.y)) % dims.x;
                points.push_back(x);
                points.push_back(y);
                points.push_back(z);
            }
            if(points.size() != validIdx.size()*3){
                SCLOG(ERROR) << "Expect points to have 3*validIdx.size(), but got " << points.size() << "(" << validIdx.size()*3 << ")";
            }
        }

        template<typename T>
        void UpdateLabelAndConfidence(int validLabelNum, const float *resulting_probs, const std::vector<size_t> &validIdx, T *label, float *conf) {
            size_t validIdxSize = validIdx.size();
//    std::vector<int> output_label_optimized(validIdxSize);

            for(size_t i=0;i<validIdxSize;++i){
                auto idx = validIdx[i];
                int max_idx=0; float max_value=0;
                for(size_t j=0;j<validLabelNum;++j){
                    float value = resulting_probs[i*validLabelNum+j];
                    if(value > max_value) {
                        max_idx = j;
                        max_value = value;
                    }
                }
                label[idx] = max_idx+1; // move 0 -> 1
                conf[idx]  = max_value;

//                if(i==0){
//                    printf("label[%d], Probability: \n\t", max_idx);
//                    for(size_t j=0;j<validLabelNum;++j)
//                        printf(" %f",resulting_probs[i*validLabelNum+j]);
//                    printf("\n");
//                }
            }
        }
    };
}