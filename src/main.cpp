#include "general.h"
#include "Mesh.h"
#include "MatStack.h"
#include <memory>
#include <unordered_map>
#include <set>
#include <functional>
#include <map>
#include <cfloat>
#include <algorithm>
#include <sstream>

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
    LOG("center=" << mesh.m_center);
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


enum EAxis { Xaxis, Yaxis, Zaxis, XYaxis, XZaxis, YZaxis };
class Transform
{
public:
    EAxis m_axis;
    Mat4 m_model;
    Mat4 m_base;

    Transform() {
        m_axis = XYaxis;
        m_model.identity();
        m_base.identity();
    }
    void rotate(int x, int y, Mat4& m) {
        Mat4 wmat;
        double fx = (double)x, fy = (double)y;

        wmat = m;
        m.identity();

        switch (m_axis)
        {
        case Xaxis:	m.rotate(fx, 1, 0, 0); break;
        case Yaxis:	m.rotate(fx, 0, 1, 0); break;
        case Zaxis:	m.rotate(fx, 0, 0, 1); break;
        case XYaxis: m.rotate(fx, 0, 1, 0); m.rotate(fy, 1, 0, 0); break;
        case XZaxis: m.rotate(fx, 0, 0, 1); m.rotate(fy, 1, 0, 0); break;
        case YZaxis: m.rotate(fx, 0, 1, 0); m.rotate(fy, 0, 0, 1); break;
        }

        m.mult(wmat);
    }
};

Transform g_transform;


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
    //vector<Vec3> pairs;
    vector<Vec3> lines;
    int numLevels = 0;
};

unique_ptr<Lines> g_lines;





Vec3 vRound(const Vec3& a) {
    return Vec3(mRound(a.x), mRound(a.y), mRound(a.z));
}

// https://stackoverflow.com/questions/3142469/determining-the-intersection-of-a-triangle-and-a-plane
void topograph(int lvlCount, float interval, float offset) 
{
    LOG("------topograph " << lvlCount << "," << interval << "," << offset);
    Mesh& mesh = *g_mesh;

    Mat4 mat;
    mat.identity();
    mat.translate(mesh.m_center);
    mat.mult(g_transform.m_base);
    mat.translate(mesh.m_center);

    Plane plane( Vec3(0,0,1), Vec3(0,0, 0) );

    Vec3 tmax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    Vec3 tmin(FLT_MAX, FLT_MAX, FLT_MAX);

    mesh.m_vtxdist.resize(mesh.m_vtx.size());
    for (int i = 0; i < mesh.m_vtx.size(); ++i) {
        Vec3 v = mesh.m_vtx[i];
        v = mat.transformVec(v);
        tmax.pmax(v);
        tmin.pmin(v);
        float d = plane.distToPoint(v);
        mesh.m_vtxdist[i] = d;
    }

    LOG("model range= " << tmin.z << " : " << tmax.z);

    if (!g_lines)
        g_lines.reset(new Lines);
    Lines& lines = *g_lines;

    lines.lines.clear();
    lines.numLevels = 0;

    float start_lines = round(tmin.z / 10.) * 10. + offset; // don't want to start at at the very furthest pixel. start abit before to have some room
    plane.add_d = start_lines;

    int paths = 0;
    

    for (int lvl = 0; lvl < lvlCount; ++lvl)
    {
        vector<Vec3> pairs;

        Vec3 intp[3];
        int lvlLines = 0;
        for (int i = 0; i < mesh.m_idx.size(); i += 3) 
        {
            int ia = mesh.m_idx[i];
            int ib = mesh.m_idx[i + 1];
            int ic = mesh.m_idx[i + 2];
            if (ia < 0 || ib < 0 || ic < 0) {
                LOG_ERR("negative index");
                return;
            }
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
                pairs.push_back(p0);
                pairs.push_back(p1);
                ++lvlLines;
            }
        }

        //if (lvlLines > 0)
        //    LOG("lvl " << lvl << " z=" << plane.add_d << "  lines=" << lvlLines);

        plane.add_d += interval;

        if (lvlLines == 0)
            continue;

        //LOG(pairs.size() << " lines  plane range= " << start_lines << " : " << plane.add_d);
        ++lines.numLevels;

        // make the lines into continuous loops
        map<Vec3, pair<int,int> > pair_hash; // hash from Vec3 hash to its index in the pairs array (can point to either vec in a pair)

        int coll = 0;
        for(int i = 0; i < pairs.size(); ++i) {
            Vec3 h = pairs[i];
            auto it = pair_hash.find(h);
            if (it == pair_hash.end())
                pair_hash.insert(make_pair(h, make_pair(i,-1))); // -1 the other one is still not known
            else {
                if (it->second.second != -1) {
                    //LOG("collision!");
                    ++coll;
                }
                it->second.second = i;
            }

        }
        if (coll > 0)
            LOG_ERR("collisions!" << coll);

        int atpair = 0;
        // find a first opening to start from, if not found, start from 0
        for (const auto& ph : pair_hash) {
            const auto& indices = ph.second;
            if (indices.first == -1) {
                atpair = indices.second;
                break;
            }
            if (indices.second == -1) {
                atpair = indices.first;
                break;
            }
        }
        int startedAt = atpair;
        do {
            if (pairs[atpair].x == -1.0) {// marks a pair we've already visited
                atpair += 2;
                if (atpair >= pairs.size())
                    atpair = 0;
                continue;
            }

            // start loop
            int curIndex = atpair;
            Vec3 startPnt = pairs[curIndex];
            int toIndex = -1;
            // loop single path
            while (true) 
            {
                //if (lines.lines.size() == 154)
                //    LOG("bla");
                lines.lines.push_back( pairs[curIndex] );
                pairs[curIndex].x = -1.0;

                if ((curIndex % 2) == 0)  // which of the pair is this pointing to?
                    toIndex = curIndex + 1;
                else
                    toIndex = curIndex - 1;

                Vec3 h = pairs[toIndex];
                Vec3 toIndexPnt = pairs[toIndex];
                pairs[toIndex].x = -1.0;

                auto it = pair_hash.find(h);
                if (it == pair_hash.end())
                    break; // end of chain
                auto& indices = it->second;

                if (toIndex == indices.first)
                    curIndex = indices.second;
                else
                    curIndex = indices.first;

                if (curIndex == atpair) {
                    //LOG("reached end");
                    lines.lines.push_back(startPnt); // lines.pairs[curIndex] was already marked with -1 so we need to get the saved one
                    break; // reached start of loop
                }
                if (curIndex == -1) { // add the last one that we found to be an ending
                    lines.lines.push_back(toIndexPnt);
                    break; 
                }
            
                if (pairs[curIndex].x == -1.0)
                    break;

            }
            lines.lines.push_back(Vec3(0,0,0));
            ++paths;

            pairs[atpair].x = -1.0; // mark as visited
            atpair += 2;
            if (atpair >= pairs.size())
                atpair = 0;

        } while (atpair != startedAt && (atpair + 1) != startedAt);

        lines.lines.back().z = 1.0; // means we should change color
        //LOG("LVL " << lvl << " paths " << paths);
    } // lvl for

    LOG(paths << " paths " << lines.lines.size() << " lines");

    //LOG("unique " << added << " / " << lines.pairs.size());
    //LOG(lines.pairs.size() << " lines");
}



Vec2i g_lastPos;

void mouseDown(int rightButton, int x, int y) {
    g_lastPos = Vec2i(x, y);
}
void mouseUp(int rightButton, int x, int y) {
}
void mouseMove(int buttons, int ctrlPressed, int x, int y) {
    if (buttons == 0) 
        return;
    int dx = x - g_lastPos.x;
    int dy = y - g_lastPos.y;
    //cout << "delta " << dx << "," << dy << endl;

    Mat4* which = &g_transform.m_base;
    if (ctrlPressed)
        which = &g_transform.m_model;
    //g_transform.rotate(dx, dy, g_transform.m_model);
    g_transform.rotate(dx, dy, *which);
    g_lastPos = Vec2i(x, y);
}

void zeroTransforms() {
    g_transform.m_base.identity();
    g_transform.m_model.identity();
}

#ifdef EMSCRIPTEN

/*void paintLines()
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
}*/

stringstream g_svg;

void paintPaths(int upto, bool fill, bool makeSvg)
{
    Lines& lines = *g_lines;

    //cout << "center=" << g_mesh->m_center << endl;
    //cout << "model=" << g_transform.m_model.cur() << endl;

    Mat4 mat;
    mat.identity();
    mat.translate(g_mesh->m_center);
    mat.mult( g_transform.m_base );
    mat.mult( g_transform.m_model );
    mat.translate(-g_mesh->m_center);
    //cout << mat << endl;

    float colPos = 0;
    float colDelta = 1.0 / lines.numLevels;
    EM_ASM(ctx.fillStyle = 'rgb(0,0,0)');

    EM_ASM(ctx.beginPath());
    if (makeSvg) {
        g_svg << "<path d=\""; 
    }
    //LOG("--begin " << lines.lines.size());
    bool nextIsMove = true;
    for(int i = 0; i < lines.lines.size(); ++i) {
        const Vec3& v = lines.lines[i];
        if (v.x == 0.0 && v.y == 0.0) {
            
            if (fill) {
                EM_ASM(ctx.fill());
            }
            else {
                EM_ASM(ctx.stroke());
            }
            if (v.z == 1.0) {
                colPos += colDelta;
                int col = (int)(round(255.0*colPos));
                EM_ASM_(ctx.fillStyle = 'rgb('+$0+','+$0+','+$0+')', col);
            }
            //EM_ASM(ctx.fill());
            nextIsMove = true;
            EM_ASM(ctx.beginPath());
            if (makeSvg) {
                g_svg << "\"/>\n<path d=\""; //M10 10"/>
            }

            continue;
        }

        Vec3 va = mat.transformVec(v);

        
        if (nextIsMove) {
            EM_ASM_(ctx.moveTo($0, $1), va.x, va.y);
            if (makeSvg) {
                g_svg << "M " << va.x << " "  << va.y << " ";
            }
            nextIsMove = false;
        }
        else {
            EM_ASM_(ctx.lineTo($0, $1), va.x, va.y);
            if (makeSvg) {
                g_svg << "L " << va.x << " " << va.y << " ";
            }
        }

    }

    if (fill) {
        EM_ASM(ctx.fill());
    }
    else {
        EM_ASM(ctx.stroke());
    }
    if (makeSvg) {
        g_svg << "\"/>\n";
    }
}

string makeSvg(int upto) {
    g_svg = stringstream();
    g_svg << "<svg width=\"700\" height=\"700\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    paintPaths(upto, true, true);
    g_svg << "</svg>";
    return g_svg.str();
}


EMSCRIPTEN_BINDINGS(my_module)
{
    emscripten::function("loadMesh", &loadMesh);
    emscripten::function("paintMesh", &paintMesh);
    emscripten::function("topograph", &topograph);
  //  emscripten::function("paintLines", &paintLines);
    emscripten::function("paintPaths", &paintPaths);
    emscripten::function("mouseDown", &mouseDown);
    emscripten::function("mouseUp", &mouseUp);
    emscripten::function("mouseMove", &mouseMove);
    emscripten::function("zeroTransforms", &zeroTransforms);
    emscripten::function("makeSvg", &makeSvg);
    
}


#endif

#ifndef EMSCRIPTEN
int main()
{
    loadMesh("C:/projects/my_topograph/models/cube.obj");
    //topograph(60,20);
    //topograph(60, 445, 0); // duplicate vertex!
    topograph(60, 91, 0);
}
#else
int main() // called when everything was loaded
{
    EM_ASM(start());
}
#endif