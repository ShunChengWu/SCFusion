#pragma once
//#define LOGODD_MAX 12.787536f //34.7609869
//#define LOGODD_MAX 34.7609869
#define LOGODD_MAX 1000
#define LOGODD_MIN -LOGODD_MAX
#define LOGODD_OCU 1.2787536f //3.47609869
//#define LOGODD_OCU 3.47609869
#define LOGODD_ETY -LOGODD_OCU

#define LOGODD_RATIO 0.80f
#define LOGODD_BIAS 0.15f
//#define LOGODD_SURFACE 2.9f // at around 0.95 probability.
//#define LOGODD_SURFACE 0.5f // test
//#define LOGODD_SURFACE 1.25f // test
#define LOGODD_SURFACE 1e-3 // test

#define measureW 2
#define predictscW 1
#define MaxLabelW 10.f

namespace SCFUSION{
    namespace Policy {
        enum Integrate {
            Integrate_WEIGHTED, Integrate_DIRECT
        };
    };
}