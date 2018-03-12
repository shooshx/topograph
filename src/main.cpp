#include "general.h"
#include "Mesh.h"

#ifdef EMSCRIPTEN
#include <emscripten.h>
#include <emscripten/bind.h>
#endif

using namespace std;

#ifndef EMSCRIPTEN
int main()
{
    loadMesh("C:/projects/topograph/models/bunny.obj");
}
#endif

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


void paint() {
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


EMSCRIPTEN_BINDINGS(my_module)
{
    emscripten::function("loadMesh", &loadMesh);
    emscripten::function("paint", &paint);
}