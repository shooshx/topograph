
#include "Vec.h"
#include <vector>



class Mesh
{
public:
    vector<Vec3> m_vtx;
    vector<int> m_idx;
    Vec3 m_pmax, m_pmin;

    vector<float> m_vtxdist;

    bool loadObj(const char* path);
    void scale(const Vec3& f);
    void translate(const Vec3& t);
    void recalcMinMax();
};

