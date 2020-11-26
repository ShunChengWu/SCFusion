//
// Created by sc on 6/1/20.
//

#ifndef SCSLAM_PROEJCT_JSONUTILS_H
#define SCSLAM_PROEJCT_JSONUTILS_H
#include "third_party/json11.hpp"
#include <cassert>
class JsonUtils{
public:
    static void JsonPrintType(const json11::Json &json){
        switch (json.type()) {
            case json11::Json::NUL:
                printf("NULL\n");
                break;
            case json11::Json::NUMBER:
                printf("NUMBER\n");
                break;
            case json11::Json::BOOL:
                printf("BOOL\n");
                break;
            case json11::Json::STRING:
                printf("STRING\n");
                break;
            case json11::Json::ARRAY:
                printf("ARRAY\n");
                break;
            case json11::Json::OBJECT:
                printf("OBJECT\n");
                break;
        }
    }
    static std::string GetTypeString(const json11::Json &json){
        switch (json.type()) {
            case json11::Json::NUL:
                return "NULL";
            case json11::Json::NUMBER:
                return "NUMBER";
            case json11::Json::BOOL:
                return "BOOL";
            case json11::Json::STRING:
                return "STRING";
            case json11::Json::ARRAY:
                return "ARRAY";
            case json11::Json::OBJECT:
                return "OBJECT";
        }
    }

    static void PrintStruct(const json11::Json &json, int level= 0, int max_level = -1){
        if(max_level>0) if(level>max_level)return;
        std::stringstream ss;
        for(int i=0;i<level;++i) ss << "\t";
        auto s = ss.str();
        auto space = s.c_str();

        printf("%s[%d] type: %s\n", space, level, GetTypeString(json).c_str());
        switch (json.type()) {
            case json11::Json::NUL:
                break;
            case json11::Json::NUMBER:
                printf("%s%f\n", space, json.number_value());
                break;
            case json11::Json::BOOL:
                printf("%s%d\n", space, json.bool_value());
                break;
            case json11::Json::STRING:
                printf("%s%s\n", space, json.string_value().c_str());
                break;
            case json11::Json::ARRAY:
                for(const auto &items : json.array_items()){
                    PrintStruct(items, level + 1, max_level);
                }
                break;
            case json11::Json::OBJECT:
                for(const auto &items : json.object_items()){
                    printf("%s%s:\n", space, items.first.c_str());
                    PrintStruct(items.second, level + 1, max_level);
                }
                break;
        }
    }
    static std::vector<double> ReadNumberArrayItems(const json11::Json &json){
        assert(json.is_array());
        std::vector<double> output;
        output.reserve(json.array_items().size());
        for(const auto &j :json.array_items()){
            assert(j.is_number());
            output.push_back(j.number_value());
        }
        return output;
    }
};

#endif //SCSLAM_PROEJCT_JSONUTILS_H
