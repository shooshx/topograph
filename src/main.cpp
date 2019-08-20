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

namespace topo {

using namespace std;

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

class Lines {
public:
    //vector<Vec3> pairs;
    vector<Vec3> lines; // line sequence with .x==0,.y==0 marker to stop segment .z == 1 to stop level (there may be multi segments in a level)
    int numLevels = 0;
};

// http://mathworld.wolfram.com/HessianNormalForm.html
// http://mathworld.wolfram.com/Point-PlaneDistance.html
class Plane
{
public:
    Vec3 normal, point;
    float a,b,c,d;
    float p;
    float add_d;

    Plane() {}
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


    float distToPoint(const Vec3& pnt) const {
        return Vec3::dotProd(normal, pnt) + p - add_d;
    }

    bool segmentIntersect(float d1, float d2, const Vec3& pnt1, const Vec3& pnt2, Vec3& outp) const {
        if (d1*d2 > 0)  // points on the same side of plane
            return false;

        float t = d1 / (d1 - d2); // 'time' of intersection point on the segment
        outp = pnt1 + t * (pnt2 - pnt1);
        return true;
    }

    int triangleIntersect(float dA, float dB, float dC, const Vec3& triA, const Vec3& triB, const Vec3 triC, Vec3 outSegTips[]) const
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




struct State {
    float m_meshScale = 0.0;
    unique_ptr<Mesh> m_mesh;
    Transform m_transform;
    Plane m_plane;
    
    // used only by topograph() and paintLines()
    unique_ptr<Lines> m_lines;
};

vector<State> g_instances;


void setTransform_ext(int instance, 
                      float m0, float m1, float m2, float m3, 
                      float m4, float m5, float m6, float m7,
                      float m8, float m9, float m10, float m11,
                      float m12, float m13, float m14, float m15)
{
    M_CHECK(instance < g_instances.size());
    Mat4& m = g_instances[instance].m_transform.m_base;
    m.m[0] = m0;    m.m[1] = m1;    m.m[2] = m2;    m.m[3] = m3;
    m.m[4] = m4;    m.m[5] = m5;    m.m[6] = m6;    m.m[7] = m7;
    m.m[8] = m8;    m.m[9] = m9;    m.m[10] = m10;  m.m[11] = m11;
    m.m[12] = m12;  m.m[13] = m13;  m.m[14] = m14;  m.m[15] = m15;
}



#define CANVAS_SCALE 800
#define MARGINS_SCALE 50 // just to avoid just touching the margins
#define CANVAS_CENTER 400

bool loadMesh(int instance, const string& path, bool is_jsobj, bool canvas_center)
{
    if (instance >= g_instances.size())
        g_instances.resize(instance+1);
    auto& state = g_instances[instance];
    Mesh* mptr = new Mesh;
    state.m_mesh.reset(mptr);
    Mesh& mesh = *mptr;
    if (!is_jsobj)
        mesh.loadObj(path.c_str());
    else
        mesh.loadFromJs(path.c_str());

    LOG("loaded vtx=" << mesh.m_vtx.size() << "  idx=" << mesh.m_idx.size() << " poly=" << mesh.m_idx.size() / 3);
    Vec3 d = mesh.m_pmax - mesh.m_pmin;
    LOG("range " << mesh.m_pmax << " : " << mesh.m_pmin << " = " << d);

    // rescale to fit the size of the screen (0-w,0-h) 
    float mxd = mMax(d.x, mMax(d.y, d.z));
    float scalef = 1.0 / mxd * (CANVAS_SCALE - 2*MARGINS_SCALE);
    state.m_meshScale = scalef;
    LOG("scale " << scalef);

    float scale_y = -scalef;
    if (!canvas_center)
        scale_y = scalef; // don't flip y in opengl
    mesh.scale(Vec3(scalef, scale_y, scalef)); // reverse y canvas is top left centered
    mesh.recalcMinMax();
    if (canvas_center) { // not needed in opengl
        Vec3 tr = -mesh.m_center; // + Vec3(MARGINS_SCALE, MARGINS_SCALE, 0);
        // center on 0,0,0
        LOG("translate to center= " << tr.x << "," << tr.y << "," << tr.z);
        mesh.translate(tr);
        mesh.recalcMinMax();
    }


    d = mesh.m_pmax - mesh.m_pmin;
    LOG("range " << mesh.m_pmax << " : " << mesh.m_pmin << " = " << d);
    LOG("center=" << mesh.m_center);
    return true;
}


#ifdef EMSCRIPTEN
void paintMesh(int instance) {
    M_CHECK(instance < g_instances.size());
    Mesh& mesh = *g_instances[instance].m_mesh;
    
    EM_ASM(ctx.beginPath());
    
    //EM_ASM(ctx.moveTo(10,10));
    //EM_ASM(ctx.lineTo(100, 100));
    for (int i = 0; i < mesh.m_idx.size(); i += 3) {
        const Vec3& va = mesh.m_vtx[mesh.m_idx[i]];
        const Vec3& vb = mesh.m_vtx[mesh.m_idx[i+1]];
        const Vec3& vc = mesh.m_vtx[mesh.m_idx[i+2]];

        EM_ASM_(ctx.moveTo($0, $1), va.x + CANVAS_CENTER, va.y + CANVAS_CENTER);
        EM_ASM_(ctx.lineTo($0, $1), vb.x + CANVAS_CENTER, vb.y + CANVAS_CENTER);
        EM_ASM_(ctx.lineTo($0, $1), vc.x + CANVAS_CENTER, vc.y + CANVAS_CENTER);
    }
    EM_ASM(ctx.stroke());
}
#endif






Vec3 vRound(const Vec3& a) {
    return Vec3(mRound(a.x), mRound(a.y), mRound(a.z));
}



void initPlane(State& state, const Vec3& normal, const Vec3 point, Plane* out_plane, Vec3* tmin, Vec3* tmax)
{
    Mesh& mesh = *state.m_mesh;
    Transform& transform = state.m_transform;
    
    Mat4 mat;
    mat.identity();
   // mat.translate(mesh.m_center); TBD TBD figure out
    mat.mult(transform.m_base);
   // mat.translate(mesh.m_center);
    
    Plane plane( normal, point );

    *tmax = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    *tmin = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    
    float dmin = FLT_MAX, dmax = -FLT_MAX;
    
    // go over the mesh and for each vertex calculate the distance from the plane
    mesh.m_vtxdist.resize(mesh.m_vtx.size());
    for (int i = 0; i < mesh.m_vtx.size(); ++i) {
        Vec3 v = mesh.m_vtx[i];
        v = mat.transformVec(v);
        tmax->pmax(v);
        tmin->pmin(v);
        float d = plane.distToPoint(v);
        mesh.m_vtxdist[i] = d;
        if (d > dmax) dmax = d;
        if (d < dmin) dmin = d;
    } 
    //LOG("vtx-dist range=" << dmin << " " << dmax);
    *out_plane = plane;
}

void initPlane_ext(int instance, float nx, float ny, float nz, float px, float py, float pz)
{
    M_CHECK(instance < g_instances.size());
    auto& state = g_instances[instance];
    
    Vec3 tmin, tmax;
    initPlane(state, Vec3(nx,ny,nz), Vec3(px,py,pz), &state.m_plane, &tmin, &tmax);
}

//vector<Vec3> pairs_unrounded;

// TBD - Teapot is pretty bad
struct IntersectInfo {
    int index; // index of the 
};

int planeIntersect(State& state, const Plane& plane, vector<Vec3>& pairs)
{
    Mesh& mesh = *state.m_mesh;
    Transform& transform = state.m_transform;
    Mat4 mat;
    mat.identity();
   // mat.translate(mesh.m_center); TBD TBD figure out
    mat.mult(transform.m_base);
    
    Vec3 intp[3];
    int lvlLines = 0;
    for (int i = 0; i < mesh.m_idx.size(); i += 3) 
    {
        int ia = mesh.m_idx[i];
        int ib = mesh.m_idx[i + 1];
        int ic = mesh.m_idx[i + 2];
        if (ia < 0 || ib < 0 || ic < 0) {
            LOG_ERR("negative index");
            return 0;
        }        
        Vec3 va = mesh.m_vtx[ia];
        va = mat.transformVec(va);
        Vec3 vb = mesh.m_vtx[ib];
        vb = mat.transformVec(vb);
        Vec3 vc = mesh.m_vtx[ic];
        vc = mat.transformVec(vc);
        
        float da = mesh.m_vtxdist[ia] - plane.add_d;
        float db = mesh.m_vtxdist[ib] - plane.add_d;
        float dc = mesh.m_vtxdist[ic] - plane.add_d;

        int retCount = plane.triangleIntersect(da, db, dc, va, vb, vc, intp);
        if (retCount == 2) {
            Vec3 p0 = vRound(intp[0]);  // TBD this 
            Vec3 p1 = vRound(intp[1]);
            if (p0 == p1) {
                continue;
            }
            pairs.push_back(p0);
            pairs.push_back(p1);
            ++lvlLines;

            //pairs_unrounded.push_back(intp[0]);
            //pairs_unrounded.push_back(intp[1]);
        }
    }

    return lvlLines;
}

int makeContinuousLine(vector<Vec3>& pairs, Lines& lines)
{
    
    int paths = 0;
    // make the lines into continuous loops
    map<Vec3, pair<int,int> > pair_hash; // hash from Vec3 hash to its index in the pairs array (can point to either vec in a pair)

    // add all vertices for easier finding
    int coll = 0;
    for(int i = 0; i < pairs.size(); ++i) {
        Vec3 h = pairs[i];
        auto it = pair_hash.find(h);
        if (it == pair_hash.end())
            pair_hash.insert(make_pair(h, make_pair(i,-1))); // -1 the other one is still not known
        else {
            if (it->second.second != -1) {
                //LOG("collision! " << pairs_unrounded[i] << " -- " << pairs_unrounded[it->second.second]);
                ++coll;
            }
            it->second.second = i;
        }

    }
    if (coll > 0)
        LOG_ERR("collisions!" << coll);

    //pairs_unrounded.clear();

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
        if (pairs[atpair].x == -FLT_MAX) {// marks a pair we've already visited
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
            pairs[curIndex].x = -FLT_MAX;

            if ((curIndex % 2) == 0)  // which of the pair is this pointing to?
                toIndex = curIndex + 1;
            else
                toIndex = curIndex - 1;

            Vec3 h = pairs[toIndex];
            Vec3 toIndexPnt = pairs[toIndex];
            pairs[toIndex].x = -FLT_MAX;

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
        
            if (pairs[curIndex].x == -FLT_MAX)
                break;

        }
        lines.lines.push_back(Vec3(0,0,0)); // marker to stop line segment
        ++paths;

        pairs[atpair].x = -FLT_MAX; // mark as visited (this destroys the data)
        atpair += 2;
        if (atpair >= pairs.size())
            atpair = 0; // wrap around (we might have started from the middle

    } while (atpair != startedAt && (atpair + 1) != startedAt);

    lines.lines.back().z = FLT_MAX; // mark the marker Vec3 to mean we should change color
    return paths;
}

// API for a single plane intersection.
// before this call initPlane_ext() and loadMesh()
bool planeIntersectLine_ext(int instance, float at_dist, const string& lines_into) 
{
    M_CHECK(instance < g_instances.size());
    auto& state = g_instances[instance];
    
    vector<Vec3> pairs; // every two consecutive points are a line
    state.m_plane.add_d = at_dist * state.m_meshScale; // transfer it to the int coordinate system
    //LOG("dist to center=" << state.m_plane.distToPoint(state.m_plane.point));
    //auto start = EM_ASM_DOUBLE(return new Date().valueOf());
    int lvlLines = planeIntersect(state, state.m_plane, pairs);

    if (lvlLines == 0) {
        LOG("intersection did not produce anything");
        return false;
    }
    //auto mid = EM_ASM_DOUBLE(return new Date().valueOf());
    Lines lines;
    int paths = makeContinuousLine(pairs, lines);
    //auto end = EM_ASM_DOUBLE(return new Date().valueOf());

    //LOG("produced " << paths << " paths t1=" << end - start << "  t2=" << mid - start);
    LOG("produced " << paths << " for " << at_dist);
#ifdef EMSCRIPTEN
    for(const auto& l: lines.lines) {
        EM_ASM_(window[Pointer_stringify($0)].push($1,$2,$3), lines_into.c_str(), 
                l.x / state.m_meshScale, l.y / state.m_meshScale, l.z / state.m_meshScale); // back to model coordinates
    }
#endif
    return true;
}


// https://stackoverflow.com/questions/3142469/determining-the-intersection-of-a-triangle-and-a-plane
void topograph(int instance, int lvlCount, float interval, float offset) 
{
    M_CHECK(instance < g_instances.size());
    auto& state = g_instances[instance];
    
    LOG("------topograph " << lvlCount << "," << interval << "," << offset);
    Mesh& mesh = *state.m_mesh;
    Plane plane;
    Vec3 tmin, tmax;

    initPlane(state, Vec3(0,0,1), Vec3(0,0,0), &plane, &tmin, &tmax);

    LOG("model range= " << tmin.z << " : " << tmax.z);

    if (!state.m_lines)
        state.m_lines.reset(new Lines);
    Lines& lines = *state.m_lines;

    lines.lines.clear();
    lines.numLevels = 0;

    float start_lines = round(tmin.z / 10.) * 10. + offset; // don't want to start at at the very furthest pixel. start abit before to have some room
    float at_dist = start_lines;

    int paths = 0;
    
    vector<Vec3> pairs;
    for (int lvl = 0; lvl < lvlCount; ++lvl)
    {
        pairs.clear();

        plane.add_d = at_dist;
        int lvlLines = planeIntersect(state, plane, pairs);
        //if (lvlLines > 0)
        //    LOG("lvl " << lvl << " z=" << at_dist << "  lines=" << lvlLines);

        at_dist += interval;

        if (lvlLines == 0)
            continue;

        //LOG(pairs.size() << " lines  plane range= " << start_lines << " : " << at_dist);
        ++lines.numLevels;
        
        paths += makeContinuousLine(pairs, lines);

        //LOG("LVL " << lvl << " paths " << paths);
    } // lvl for

    LOG(paths << " paths " << lines.lines.size() << " lines");

    //LOG("unique " << added << " / " << lines.pairs.size());
    //LOG(lines.pairs.size() << " lines");
}



Vec2i g_lastPos;

void mouseDown(int instance, int rightButton, int x, int y) {
    g_lastPos = Vec2i(x, y);
}
void mouseUp(int instance, int rightButton, int x, int y) {
}
void mouseMove(int instance, int buttons, int ctrlPressed, int x, int y) {
    M_CHECK(instance < g_instances.size());
    auto& transform = g_instances[instance].m_transform;
    
    if (buttons == 0) 
        return;
    int dx = x - g_lastPos.x;
    int dy = y - g_lastPos.y;
    //cout << "delta " << dx << "," << dy << endl;

    Mat4* which = &transform.m_base;
    if (ctrlPressed)
        which = &transform.m_model;
    //g_transform.rotate(dx, dy, g_transform.m_model);
    transform.rotate(dx, dy, *which);
    g_lastPos = Vec2i(x, y);
}

void zeroTransforms(int instance) {
    M_CHECK(instance < g_instances.size());
    auto& transform = g_instances[instance].m_transform;
    
    transform.m_base.identity();
    transform.m_model.identity();
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

void paintPaths(int instance, int upto, bool fill, bool makeSvg, float zoom)
{
    M_CHECK(instance < g_instances.size());
    auto& state = g_instances[instance];
    Lines& lines = *state.m_lines;

    //cout << "center=" << g_mesh->m_center << endl;
    //cout << "model=" << g_transform.m_model.cur() << endl;

    Mat4 mat;
    mat.identity();
  //  mat.translate(state.m_mesh->m_center);
  //  mat.mult( state.m_transform.m_base );
    mat.mult( state.m_transform.m_model );
  //  mat.translate(-state.m_mesh->m_center);
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
            if (v.z == FLT_MAX) {
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
        va.x = va.x * zoom + CANVAS_CENTER;
        va.y = va.y * zoom + CANVAS_CENTER;
        
        
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

string makeSvg(int instance, int upto) {
    g_svg = stringstream();
    g_svg << "<svg width=\"700\" height=\"700\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    paintPaths(instance, upto, true, true, 1.0);
    g_svg << "</svg>";
    return g_svg.str();
}

#endif

} // namespace

#ifdef EMSCRIPTEN

EMSCRIPTEN_BINDINGS(my_module)
{
    emscripten::function("loadMesh", &topo::loadMesh);
    emscripten::function("paintMesh", &topo::paintMesh);
    emscripten::function("topograph", &topo::topograph);
  //  emscripten::function("paintLines", &topo::paintLines);
    emscripten::function("paintPaths", &topo::paintPaths);
    emscripten::function("mouseDown", &topo::mouseDown);
    emscripten::function("mouseUp", &topo::mouseUp);
    emscripten::function("mouseMove", &topo::mouseMove);
    emscripten::function("zeroTransforms", &topo::zeroTransforms);
    emscripten::function("makeSvg", &topo::makeSvg);
    
    // API for single plane intersection
    emscripten::function("initPlane_ext", &topo::initPlane_ext);
    emscripten::function("planeIntersectLine_ext", &topo::planeIntersectLine_ext);
    emscripten::function("setTransform_ext", &topo::setTransform_ext);
}


#endif



#ifndef EMSCRIPTEN

#define INST 0
int main_topo()
{
    topo::loadMesh(INST, "C:/projects/my_topograph/models/cube.obj", false, true);
    //topograph(60,20);
    //topograph(60, 445, 0); // duplicate vertex!
    topo::topograph(INST, 60, 91, 0);
    return 0;
}

int main()
{
    //loadMesh(INST, "C:/Projects/maze_balls/sphere_0.obj", false, false);
    topo::loadMesh(INST, "C:/Projects/maze_balls/sphere_1.obj", false, false);
    topo::initPlane_ext(INST, 0, 1, 0, 0, 0, 0);
    topo::planeIntersectLine_ext(INST, -0.66999, "lines_sphere_0");
    return 0;
}

#else
int main() // called when everything was loaded
{
    EM_ASM(start());
}
#endif

