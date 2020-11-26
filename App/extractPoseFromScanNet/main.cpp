/*
This program loads and extract pose information from ScanNet *.sens files
*/
#include <ImageLoader/ImageLoader.hpp>
#include <ImageLoader/ImageLoadFactory.h>
#include <CxxTools/PathTool.hpp>
#include <CxxTools/parser.hpp>
#include "../../ORUtils/Logging.h"
struct Params{
    std::string pth_scan,pth_out;
};

int ParseInputArguments(int argc, char**argv, Params&params){
    tools::Parser parser(argc,argv);
    parser.addOption(pkgcname("pth_scan", &params.pth_scan),"path to scannet folder.", true);
    parser.addOption(pkgcname("pth_out", &params.pth_out),"path to outpu folder.", true);

    if(parser.showMsg()<0){
        SCLOG(ERROR) << "";
    }
}

int main(int argc, char **argv) {
#ifndef NDEBUG
    SCLOG_ON(VERBOSE);
#else
    SCLOG_ON(INFO);
#endif
    Params params;
    ParseInputArguments(argc, argv, params);

    // create output folder
    tools::PathTool::create_folder(params.pth_out);

    // get all scans
    auto scans = tools::PathTool::get_files_in_folder(params.pth_scan);
    std::unique_ptr<SCFUSION::IO::ImageLoader> imageLoader;

    ORUtils::Matrix4<float> pose;
    std::vector<ORUtils::Matrix4<float>> poses;
    for (const auto &scan: scans) {
        auto scan_name = tools::PathTool::getFileName(scan);
//        if(scan_name != "scene0549_01")continue;

        SCLOG(INFO) << "process scene: " << scan_name;
        imageLoader.reset(SCFUSION::ImageLoaderFactory::MakeImageLoader(SCFUSION::IO::InputDateType::INPUTTYPE_SCANNET,
                                                                      params.pth_scan + "/" + scan_name));
        imageLoader->Init();
        SCLOG(VERBOSE) << "Num of images: " << imageLoader->NumberOfImages();
        poses.clear();
        poses.reserve(imageLoader->NumberOfImages());
        SCLOG(VERBOSE) << "loop over all poses";
        while(true) {
            auto idx = imageLoader->Next();
            if (idx < 0) break;
//            if(idx>50)break; //TODO: remove me after debug
            imageLoader->getPose(idx, &pose);
            poses.push_back(pose);
            SCLOG(DEBUG) << "pose: \n" << pose;
        }

        // Save
        SCLOG(VERBOSE) << "saving pose";
        tools::PathTool::create_folder(params.pth_out+"/"+scan_name);
        std::string pth_out = params.pth_out+"/"+scan_name+"/"+scan_name+".pose";
        std::fstream f(pth_out, std::ios::out);
        if(!f.is_open()) SCLOG(ERROR) << "Cannot open a file to save pose at " << pth_out;
        f << poses.size() << "\n";
        for(size_t i=0;i<poses.size();++i){
            for(size_t j=0;j<16;++j)
                f <<  poses[i].m[j] << " ";
            f << "\n";
        }
//        f.write((char*)poses.data(), poses.size() * sizeof(ORUtils::Matrix4<float>));
        SCLOG(VERBOSE) << "output saved to " << pth_out;
//        break;
    }
}