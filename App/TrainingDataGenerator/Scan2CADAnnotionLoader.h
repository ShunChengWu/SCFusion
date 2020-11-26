//
// Created by sc on 6/1/20.
//

#ifndef SCSLAM_PROEJCT_SCAN2CADANNOTIONLOADER_H
#define SCSLAM_PROEJCT_SCAN2CADANNOTIONLOADER_H

#include "JsonUtils.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

class Scan2CADAnnotionLoader{
public:
    Scan2CADAnnotionLoader(std::string pth_scannet,
                           std::string pth_shapenet, std::string pth_alignments):
            msPathScanNet(std::move(pth_scannet)), msPathShapeNet(std::move(pth_shapenet)),
            msPathAlignment(std::move(pth_alignments)){
        std::ifstream file(msPathAlignment, std::ios::in);
        assert(file.is_open());
        std::string dataset((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        std::string err;
        mpJson = std::make_unique<json11::Json>(json11::Json::parse(dataset, err));
        if (!err.empty()) {
            SCLOG(ERROR) << "Error reading " << msPathAlignment << " " << err;
        }
    }
    json11::Json *GetJson(){return mpJson.get();}

    json11::Json GetScan(const std::string &query_id_scan, bool &state){
        for(auto &json : mpJson->array_items()){
            const std::string id_scan = json["id_scan"].string_value();
            if(id_scan == query_id_scan) {
                state = true;
                return json;
            }
        }
        state = false;
        return json11::Json();
    }

    Eigen::Matrix4d GetTMatrix(const json11::Json &json){
        // trs
        Eigen::Matrix4d TMat;
        {
            auto rotation = JsonUtils::ReadNumberArrayItems(json["trs"]["rotation"]);
            auto scale = JsonUtils::ReadNumberArrayItems(json["trs"]["scale"]);
            auto translation = JsonUtils::ReadNumberArrayItems(json["trs"]["translation"]);
            assert(rotation.size()==4);
            assert(scale.size()==3);
            assert(translation.size()==3);
            TMat =  buildTMatFromTQS(translation.data(),rotation.data(),scale.data());
        }
        return TMat;
    }

private:
    std::string msPathScanNet, msPathShapeNet, msPathAlignment;
    std::unique_ptr<json11::Json> mpJson;


    Eigen::Matrix4d buildTMatFromTQS(double* t, double* q, double *s){
        Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond quat;
        quat.w() = q[0];
        quat.x() = q[1];
        quat.y() = q[2];
        quat.z() = q[3];
        SCLOG(DEBUG) << "quat: " << quat.w() << ","<<quat.x()<<","<<quat.y()<<","<<quat.z();
        SCLOG(DEBUG) << "quat: " << q[0] << ","<<q[1]<<","<<q[2]<<","<<q[3];
        output.topRightCorner<3,1>() = Eigen::Vector3d(t);
        output.topLeftCorner<3,3>()  = quat.toRotationMatrix();

        Eigen::Matrix4d scaleMat = Eigen::Matrix4d::Identity();
        for(size_t i=0;i<3;++i) scaleMat(i,i)=s[i];
        return output * scaleMat;
    }
};

#endif //SCSLAM_PROEJCT_SCAN2CADANNOTIONLOADER_H
