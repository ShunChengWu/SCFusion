#pragma once
#include <map>
#include "../ORUtils/Vector.h"
#include <string>

static std::map<unsigned short, std::string> NYU13{
	{0, "Unknown"}, 
	{1, "Bed"}, 
	{2, "Books"}, 
	{3, "Ceiling"}, 
	{4, "Chair"}, 
	{5, "Floor"}, 
	{6, "Furniture"}, 
	{7, "Objects"}, 
	{8, "Picture"}, 
	{9, "Sofa"}, 
	{10, "Table"}, 
	{11, "TV"}, 
	{12, "Wall"}, 
	{13, "Window"}, 
};

static std::map<ORUtils::Vector3<int>, unsigned short> NYU13ColorToLabel {
        {ORUtils::Vector3<int>(255,255,255),0},
        {ORUtils::Vector3<int>(0,0,255),1},
        {ORUtils::Vector3<int>(232,88,47),2},
        {ORUtils::Vector3<int>(0,217,0),3},
        {ORUtils::Vector3<int>(148,0,240),4},
        {ORUtils::Vector3<int>(222,241,23),5},
        {ORUtils::Vector3<int>(255,205,205),6},
        {ORUtils::Vector3<int>(0,223,228),7},
        {ORUtils::Vector3<int>(106,135,204),8},
        {ORUtils::Vector3<int>(116,28,1),9},
        {ORUtils::Vector3<int>(240,35,235),10},
        {ORUtils::Vector3<int>(0,166,156),11},
        {ORUtils::Vector3<int>(249,139,0),12},
        {ORUtils::Vector3<int>(225,228,194),13},
};


static std::map<unsigned short, ORUtils::Vector4<float>> NYU13ColorLabel {
{0, ORUtils::Vector4<float>(0.0, 0.0, 0.0, 255)},
{1, ORUtils::Vector4<float>(0.0, 0.0, 1.0, 255)},
{2, ORUtils::Vector4<float>(0.9137, 0.349, 0.1882, 255)},
{3, ORUtils::Vector4<float>(0.0, 0.8549, 0.0, 255)},
{4, ORUtils::Vector4<float>(0.5843, 0.0, 0.9412, 255)},
{5, ORUtils::Vector4<float>(0.8706, 0.9451, 0.0941, 255)},
{6, ORUtils::Vector4<float>(1.0, 0.8078, 0.8078, 255)},
{7, ORUtils::Vector4<float>(0.0, 0.8784, 0.898, 255)},
{8, ORUtils::Vector4<float>(0.4157, 0.5333, 0.8, 255)},
{9, ORUtils::Vector4<float>(0.4588, 0.1137, 0.1608, 255)},
{10, ORUtils::Vector4<float>(0.9412, 0.1373, 0.9216, 255)},
{11, ORUtils::Vector4<float>(0.0, 0.6549, 0.6118, 255)},
{12, ORUtils::Vector4<float>(0.9765, 0.5451, 0.0, 255)},
{13, ORUtils::Vector4<float>(0.8824, 0.898, 0.7608, 255)},
};
