#include "general.h"
#include "Mesh.h"
#include <memory>
#include <unordered_map>
#include <set>
#include <functional>
#include <map>

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

    bool segmentIntersect(float d1, float d2, const Vec3& pnt1, const Vec3& pnt2, Vec3& outp) {
        if (d1*d2 > 0)  // points on the same side of plane
            return false;

        float t = d1 / (d1 - d2); // 'time' of intersection point on the segment
        outp = pnt1 + t * (pnt2 - pnt1);
        return true;
    }

    int triangleIntersect(float dA, float dB, float dC, const Vec3& triA, const Vec3& triB, const Vec3 triC, Vec3 outSegTips[])
    {
        int arrIdx = 0;
        Vec3 intersectionPoint;
        if (segmentIntersect(dA, dB, triA, triB, intersectionPoint))
            outSegTips[arrIdx++] = intersectionPoint;

        if (segmentIntersect(dB, dC, triB, triC, intersectionPoint))
            outSegTips[arrIdx++] = intersectionPoint;

        if (segmentIntersect(dC, dA, triC, triA, intersectionPoint))
            outSegTips[arrIdx++] = intersectionPoint;
        return arrIdx;
    }

};


class Lines {
public:
    vector<Vec3> pairs;
    vector<Vec3> lines;
};

unique_ptr<Lines> g_lines;



static inline uint32_t rotl32(uint32_t n, unsigned int c)
{
    const unsigned int mask = (8*sizeof(n) - 1);  // assumes width is a power of 2.
    c &= mask;
    return (n << c) | (n >> ((-c)&mask));
}

size_t hashVec(const Vec3& v) {
    uint32_t h = *(uint32_t*)&v.x;
    h ^= rotl32( *(uint32_t*)&v.y, 16);
    h ^= rotl32( *(uint32_t*)&v.z, 32);
    return h;
}

Vec3 vRound(const Vec3& a) {
    return Vec3(mRound(a.x), mRound(a.y), mRound(a.z));
}

// https://stackoverflow.com/questions/3142469/determining-the-intersection-of-a-triangle-and-a-plane
void topograph(int lvlCount, float interval) 
{
    //LOG("topograph " << lvlCount << " " << interval);
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
    lines.lines.clear();

    plane.add_d = 0;


    for (int lvl = 0; lvl < lvlCount; ++lvl)
    {
        int linesCountStart = lines.pairs.size();
        Vec3 intp[3];
        for (int i = 0; i < mesh.m_idx.size(); i += 3) 
        {
            int ia = mesh.m_idx[i];
            int ib = mesh.m_idx[i + 1];
            int ic = mesh.m_idx[i + 2];
            const Vec3& va = mesh.m_vtx[ia];
            const Vec3& vb = mesh.m_vtx[ib];
            const Vec3& vc = mesh.m_vtx[ic];
            float da = mesh.m_vtxdist[ia] - plane.add_d;
            float db = mesh.m_vtxdist[ib] - plane.add_d;
            float dc = mesh.m_vtxdist[ic] - plane.add_d;

            int retCount = plane.triangleIntersect(da, db, dc, va, vb, vc, intp);
            if (retCount == 2) {
                Vec3 p0 = vRound(intp[0]);
                Vec3 p1 = vRound(intp[1]);
                if (p0 == p1) {
                    continue;
                }
                lines.pairs.push_back(p0);
                lines.pairs.push_back(p1);
            }
        }

        //LOG("lvl " << lvl << " z=" << plane.add_d << "  " << lines.pairs.size() - linesCountStart);

        plane.add_d += interval;
    }

    LOG(lines.pairs.size() << " lines");
    //return;

  /*  set<uint32_t> hashs;
    set<uint32_t> hashs2;

    auto comp = [](Vec3 x, Vec3 y){ 
        if (isNear(x,y))
            return false;
        return x < y; 
    };
    auto vecs = std::set<Vec3, std::function<bool(Vec3, Vec3)>>(comp);
    */

    // make the lines into continuous loops
    map<Vec3, pair<int,int> > pair_hash; // hash from Vec3 hash to its index in the pairs array (can point to either vec in a pair)

    int coll = 0;
    for(int i = 0; i < lines.pairs.size(); ++i) {
        Vec3 h = lines.pairs[i];
        auto it = pair_hash.find(h);
        if (it == pair_hash.end())
            pair_hash.insert(make_pair(h, make_pair(i,-1)));
        else {
            if (it->second.second != -1) {
                //LOG("collision!");
                ++coll;
            }
            it->second.second = i;
        }
       // hashs.insert(h);
       // hashs2.insert(hash2Vec(lines.pairs[i]));
       // vecs.insert(lines.pairs[i]);
    }

    int paths = 0;
    int atpair = 0;
    while (atpair < lines.pairs.size()) {
        if (lines.pairs[atpair].x == -1.0) {// marks a pair we've already visited
            atpair += 2;
            continue;
        }

        // start loop
        int curIndex = atpair;
        int toIndex = -1;
        while (true) {
            lines.lines.push_back( lines.pairs[curIndex] );
            lines.pairs[curIndex].x = -1.0;

            if ((curIndex % 2) == 0)  // which of the pair is this pointing to?
                toIndex = curIndex + 1;
            else
                toIndex = curIndex - 1;

            Vec3 h = lines.pairs[toIndex];
            lines.pairs[toIndex].x = -1.0;

            auto it = pair_hash.find(h);
            if (it == pair_hash.end())
                break; // end of chain
            auto& indices = it->second;

            if (toIndex == indices.first)
                curIndex = indices.second;
            else
                curIndex = indices.first;

            if (curIndex == -1 || curIndex == atpair)
                break; // reached start of loop
            if (lines.pairs[curIndex].x == -1.0)
                break;

        }
        lines.lines.push_back(Vec3(0,0,0));

        lines.pairs[atpair].x = -1.0; // mark as visited
        atpair += 2;
        ++paths;
    }

    LOG(paths << " paths " << lines.lines.size() << " lines " << lines.pairs.size() << " pairs");

    //LOG("unique " << added << " / " << lines.pairs.size());
    //LOG(lines.pairs.size() << " lines");
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

void paintPaths(int upto)
{
    Lines& lines = *g_lines;

    EM_ASM(ctx.beginPath());
    //LOG("--begin " << lines.lines.size());
    bool nextIsMove = true;
    for(int i = 0; i < lines.lines.size(); ++i) {
        const Vec3& va = lines.lines[i];
        /*if (i < upto)
            LOG("V" << i<< " " << va.x << "," << va.y);
        else {
            EM_ASM(ctx.stroke());
         //   LOG("--stroke");
            return;
        }*/

        if (va.x == 0.0 && va.y == 0.0) {
            EM_ASM(ctx.stroke());
            nextIsMove = true;
            EM_ASM(ctx.beginPath());
           // LOG("--stroke,nextIsMove");
            continue;
        }
        
        if (nextIsMove) {
            EM_ASM_(ctx.moveTo($0, $1), va.x, va.y);
            //LOG("--moveTo");
            nextIsMove = false;
        }
        else {
            EM_ASM_(ctx.lineTo($0, $1), va.x, va.y);
            //LOG("--lineTo");
        }
    }
    EM_ASM(ctx.stroke());
    //LOG("--stroke");
}

EMSCRIPTEN_BINDINGS(my_module)
{
    emscripten::function("loadMesh", &loadMesh);
    emscripten::function("paintMesh", &paintMesh);
    emscripten::function("topograph", &topograph);
    emscripten::function("paintLines", &paintLines);
    emscripten::function("paintPaths", &paintPaths);
}


#endif

#ifndef EMSCRIPTEN
int main()
{
    loadMesh("C:/projects/topograph/models/bunny.obj");
    //topograph(60,20);
    topograph(60, 91); // few loops
}
#else
int main() // called when everything was loaded
{
    EM_ASM(start());
}
#endif