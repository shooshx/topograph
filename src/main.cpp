#include "general.h"
#include "Mesh.h"
#include <memory>

#ifdef EMSCRIPTEN
#include <emscripten.h>
#include <emscripten/bind.h>
#endif

using namespace std;



unique_ptr<Mesh> g_mesh;

#define CANVAS_SCALE 800
#define MARGINS_SCALE 50 // just to avoid just touching the margins

bool loadMesh(const string& path)
{
    g_mesh.reset(new Mesh);
    Mesh& mesh = *g_mesh;
    mesh.loadObj(path.c_str());

    LOG("loaded " << mesh.m_vtx.size() << "  " << mesh.m_idx.size());
    Vec3 d = mesh.m_pmax - mesh.m_pmin;
    LOG("range " << mesh.m_pmax << " : " << mesh.m_pmin << " = " << d);

    // rescale to fit (0-1,0-1) 
    float mxd = mMax(d.x, mMax(d.y, d.z));
    float scalef = 1.0 / mxd * (CANVAS_SCALE - 2*MARGINS_SCALE);
    LOG("scale " << scalef);

    mesh.scale(Vec3(scalef, -scalef, scalef)); // reverse y canvas is top left centered
    mesh.recalcMinMax();
    mesh.translate(-mesh.m_pmin + Vec3(MARGINS_SCALE, MARGINS_SCALE,0) );
    mesh.recalcMinMax();


    d = mesh.m_pmax - mesh.m_pmin;
    LOG("range " << mesh.m_pmax << " : " << mesh.m_pmin << " = " << d);
    return true;
}

#ifdef EMSCRIPTEN
void paintMesh() {
    Mesh& mesh = *g_mesh;
    
    EM_ASM(ctx.beginPath());
    //EM_ASM(ctx.moveTo(10,10));
    //EM_ASM(ctx.lineTo(100, 100));
    for (int i = 0; i < mesh.m_idx.size(); i += 3) {
        const Vec3& va = mesh.m_vtx[mesh.m_idx[i]];
        const Vec3& vb = mesh.m_vtx[mesh.m_idx[i+1]];
        const Vec3& vc = mesh.m_vtx[mesh.m_idx[i+2]];

        EM_ASM_(ctx.moveTo($0, $1), va.x, va.y);
        EM_ASM_(ctx.lineTo($0, $1), vb.x, vb.y);
        EM_ASM_(ctx.lineTo($0, $1), vc.x, vc.y);
    }
    EM_ASM(ctx.stroke());
}
#endif

// http://mathworld.wolfram.com/HessianNormalForm.html
// http://mathworld.wolfram.com/Point-PlaneDistance.html
class Plane
{
public:
    Vec3 normal, point;
    float a,b,c,d;
    float p;
    float add_d;

    Plane(const Vec3& _normal, const Vec3& _point) {
        normal = _normal;
        point = _point;
        a = normal.x;
        b = normal.y;
        c = normal.z;
        d = - Vec3::dotProd(normal, point);
        p = d / sqrt(a*a + b*b + c*c);
        add_d = 0;
    }


    float distToPoint(const Vec3& pnt) {
        return Vec3::dotProd(normal, pnt) + p - add_d;
    }

    bool segmentIntersect(const Vec3& pnt1, const Vec3& pnt2, Vec3& outp) {
        float d1 = distToPoint(pnt1);
        float d2 = distToPoint(pnt2);

        if (d1*d2 > 0)  // points on the same side of plane
            return false;

        float t = d1 / (d1 - d2); // 'time' of intersection point on the segment
        outp = pnt1 + t * (pnt2 - pnt1);
        return true;
    }

    void triangleIntersect(const Vec3& triA, const Vec3& triB, const Vec3 triC, vector<Vec3>& outSegTips)
    {
        Vec3 intersectionPoint;
        if (segmentIntersect(triA, triB, intersectionPoint))
            outSegTips.push_back(intersectionPoint);

        if (segmentIntersect(triB, triC, intersectionPoint))
            outSegTips.push_back(intersectionPoint);

        if (segmentIntersect(triC, triA, intersectionPoint))
            outSegTips.push_back(intersectionPoint);
    }

};


class Lines {
public:
    vector<Vec3> pairs;

};

unique_ptr<Lines> g_lines;


// https://stackoverflow.com/questions/3142469/determining-the-intersection-of-a-triangle-and-a-plane
void topograph(int lvlCount, float interval) 
{
    LOG("topograph " << lvlCount << " " << interval);
    Plane plane( Vec3(0,0,1), Vec3(0,0, 0) );
    //Plane plane(Vec3(0.5, 0, 1).unitized(), Vec3(0, 0, 0));

    Mesh& mesh = *g_mesh;
    mesh.m_vtxdist.resize(mesh.m_vtx.size());
    for (int i = 0; i < mesh.m_vtx.size(); ++i) {
        float d = plane.distToPoint(mesh.m_vtx[i]);
        mesh.m_vtxdist[i] = d;
    }

    if (!g_lines)
        g_lines.reset(new Lines);
    Lines& lines = *g_lines;
    lines.pairs.clear();

    plane.add_d = 0;

    for (int lvl = 0; lvl < lvlCount; ++lvl)
    {
        int linesCountStart = lines.pairs.size();
        vector<Vec3> intp;
        for (int i = 0; i < mesh.m_idx.size(); i += 3) 
        {
            const Vec3& va = mesh.m_vtx[mesh.m_idx[i]];
            const Vec3& vb = mesh.m_vtx[mesh.m_idx[i + 1]];
            const Vec3& vc = mesh.m_vtx[mesh.m_idx[i + 2]];

            plane.triangleIntersect(va, vb, vc, intp);
            if (!intp.empty()) {
                if (intp.size() == 2) {
                    lines.pairs.push_back(intp[0]);
                    lines.pairs.push_back(intp[1]);
                }
                intp.clear();
            }
        }

        //LOG("lvl " << lvl << " z=" << plane.add_d << "  " << lines.pairs.size() - linesCountStart);

        plane.add_d += interval;
    }
    LOG(lines.pairs.size() << " lines");
}

#ifdef EMSCRIPTEN

void paintLines()
{
    Lines& lines = *g_lines;

    EM_ASM(ctx.beginPath());
    for(int i = 0; i < lines.pairs.size(); i += 2) {
        const Vec3& va = lines.pairs[i];
        const Vec3& vb = lines.pairs[i + 1];

        EM_ASM_(ctx.moveTo($0, $1), va.x, va.y);
        EM_ASM_(ctx.lineTo($0, $1), vb.x, vb.y);
    }
    EM_ASM(ctx.stroke());
}

EMSCRIPTEN_BINDINGS(my_module)
{
    emscripten::function("loadMesh", &loadMesh);
    emscripten::function("paintMesh", &paintMesh);
    emscripten::function("topograph", &topograph);
    emscripten::function("paintLines", &paintLines);
}


#endif

#ifndef EMSCRIPTEN
int main()
{
    loadMesh("C:/projects/topograph/models/bunny.obj");
    topograph();
}
#else
int main() // called when everything was loaded
{
    EM_ASM(start());
}
#endif