
#include "Vec.h"
#include <vector>

namespace topo {

class Mesh
{
public:
    vector<Vec3> m_vtx;
    vector<int> m_idx;
    Vec3 m_pmax, m_pmin;
    Vec3 m_center;

    vector<float> m_vtxdist;

    bool loadObj(const char* path);
    bool loadFromJs(const char* objname);
    void scale(const Vec3& f);
    void translate(const Vec3& t);
    void recalcMinMax();
};

}