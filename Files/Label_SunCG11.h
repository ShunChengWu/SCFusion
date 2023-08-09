//
// Created by sc on 6/1/20.
//

#ifndef SCSLAM_PROEJCT_SUNCG11_H
#define SCSLAM_PROEJCT_SUNCG11_H

#include <map>
#include <string>
#include <ORUtils/Vector.h>

static std::map<unsigned short, std::string> SunCG11 {
    {0,"None"},
    {1,"Ceiling"},
    {2,"Floor"},
    {3,"Wall"},
    {4,"Window"},
    {5,"Chair"},
    {6,"Bad"},
    {7,"Sofa"},
    {8,"Table"},
    {9,"TV"},
    {10,"Furniture"},
    {11,"Objects"},
};

static std::map<unsigned short, ORUtils::Vector4<int>> SunCG11ColorLabel {
        {0, ORUtils::Vector4<int>(0, 0, 0, 255)},
        {1, ORUtils::Vector4<int>(253, 210, 250, 255)},
        {2, ORUtils::Vector4<int>(214, 199, 137, 255)},
        {3, ORUtils::Vector4<int>(205, 216, 253, 255)},
        {4, ORUtils::Vector4<int>(86, 52, 135, 255)},
        {5, ORUtils::Vector4<int>(234, 249, 71, 255)},
        {6, ORUtils::Vector4<int>(254, 19, 75, 255)},
        {7, ORUtils::Vector4<int>(160, 185, 156, 255)},
        {8, ORUtils::Vector4<int>(237, 125, 49, 255)},
        {9, ORUtils::Vector4<int>(22, 217, 17, 255)},
        {10, ORUtils::Vector4<int>(150, 185, 217, 255)},
        {11, ORUtils::Vector4<int>(31, 102, 130, 255)},
};

static std::map<ORUtils::Vector3<int>, unsigned short> SunCG11ColorToLabel {
        {ORUtils::Vector3<int>(0, 0, 0), 0},
        {ORUtils::Vector3<int>(253, 210, 250), 1},
        {ORUtils::Vector3<int>(214, 199, 137), 2},
        {ORUtils::Vector3<int>(205, 216, 253), 3},
        {ORUtils::Vector3<int>(86, 52, 135), 4},
        {ORUtils::Vector3<int>(234, 249, 71), 5},
        {ORUtils::Vector3<int>(254, 19, 75), 6},
        {ORUtils::Vector3<int>(160, 185, 156), 7},
        {ORUtils::Vector3<int>(237, 125, 49), 8},
        {ORUtils::Vector3<int>(22, 217, 17), 9},
        {ORUtils::Vector3<int>(150, 185, 217), 10},
        {ORUtils::Vector3<int>(31, 102, 130), 11},
};

static std::map<unsigned short, unsigned short> NYU13ToSunCG11{
        {0, 0},
        {1, 6},
        {2, 11},
        {3, 1},
        {4, 5},
        {5, 2},
        {6, 10},
        {7, 11},
        {8, 11}, // picture -> object
        {9, 7},
        {10, 8},
        {11, 9},
        {12, 3},
        {13, 4},
};


static std::map<unsigned short, unsigned short> NYU13ToSunCG11SC{
        {0, 0},
        {1, 6},
        {2, 11},
        {3, 1},
        {4, 5},
        {5, 2},
        {6, 10},
        {7, 11},
        {8, 3}, // picture -> wall
        {9, 7},
        {10, 8},
        {11, 9},
        {12, 3},
        {13, 4},
};

static std::map<unsigned short, unsigned short> NYU40ToSunCG11 {
        {0, 0}, // None -> None
        {1, 3}, // wall -> wall
        {2, 2}, // floor -> floor
        {3, 10}, // cabinet -> Furniture
        {4, 6}, // bed -> bed
        {5, 5},  // chair -> chair
        {6, 7}, // sofa -> sofa
        {7, 8}, // table
        {8, 3}, // door -> wall
        {9, 4}, // window
        {10, 10}, // bookshelf -> Furniture
        {11, 3}, // picture -> wall (->objects(11))
        {12, 8}, // counter -> Table (->furni.(10))
        {13, 0}, // blinds -> None (->window(4))
        {14, 8}, // desk -> Table
        {15, 10}, // shelves -> Furniture
        {16, 3}, // curtain -> wall (->window(4))
        {17, 10}, // dresser -> Furniture
        {18, 11}, // pillow -> Objects
        {19, 0}, // mirror -> None (->objects(11))
        {20, 2}, // floor mat -> Floor
        {21, 0}, // clothes -> None (->objects(11))
        {22, 1}, // ceiling
        {23, 11}, // books -> Objects
        {24, 10}, // refrigerator -> furniture (->objects(11))
        {25, 9}, // television
        {26, 0}, // paper -> None (-> objects(11))
        {27, 11}, // towel -> Objects
        {28, 0}, // shower curtain -> None (->objects)
        {29, 11}, // box -> objects
        {30, 3}, // whiteboard -> wall (->objects)
        {31, 0}, // person -> none (->objects)
        {32, 8}, // night stand -> Table (furniture)
        {33, 11}, // toilet -> objects (furniture)
        {34, 0}, // sink -> none (object)
        {35, 11}, // lamp -> Objects
        {36, 0}, // bathtub -> none (furniture)
        {37, 11}, // bag -> objects
        {38, 0}, // otherstructure -> none (object)
        {39, 0}, // otherfurn -> none (funiture)
        {40, 0}, // otherprop -> none
        {255,0},
};


static std::map<unsigned short, unsigned short> NYU40ToSunCG11SC {
        {0,0},
        {1, 3}, // wall -> wall
        {2, 2}, // floor -> floor
        {3, 10}, // cabinet -> furniture
        {4, 6}, // bed -> bed
        {5, 5}, // chair - >chair
        {6, 7},//sofa -> sofa
        {7, 8}, // table -> table
        {8, 3}, //door -> wall
        {9, 4}, // window -> window ( or None?)
        {10, 10}, // bookshelf -> furniture
        {11, 3},// picture -> wall
        {12, 10}, // counter -> furniture
        {13, 0}, // blinds -> None
        {14, 8}, // desk -> table
        {15, 10}, // shelves -> furniture
        {16, 3}, // curtain -> wall (or None?)
        {17, 10}, // dresser->furniture
        {18, 11},// pillow ->objects
        {19, 0},// mirror ->None
        {20, 2},// floor mat -> floor
        {21, 0},// clothes -> None
        {22, 1},//ceiling->ceiling
        {23, 11},//books->objects
        {24, 10},// refridgerator -> furniture
        {25, 9},// tv -> tv
        {26, 0},// paper -> None
        {27, 11},//towel ->object
        {28, 0},//shower curtain ->None
        {29, 11},//box ->obj
        {30, 3},//whiteboard -> Wall ( or None?)
        {31, 0},//person->None
        {32, 10},//night stand -> furniture//table
        {33, 11},//toilet -> object
        {34, 0},//sink -> None
        {35, 11},//lamp ->obj
        {36, 0},//bathtub->None
        {37, 11},//bag ->obj
        {38, 0},//otherstructure->None
        {39, 11},//otherfurniture->None (object)
        {40, 0},//otherprop->None
        {255, 0},
};

#endif //SCSLAM_PROEJCT_SUNCG11_H