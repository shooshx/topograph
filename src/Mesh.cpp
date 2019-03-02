#include "Mesh.h"
#include "general.h"
#include <string>
#include <fstream>
#include <cfloat>
#ifdef EMSCRIPTEN
#include <emscripten.h>
#endif

namespace topo {

bool Mesh::loadObj(const char* path)
{
    ifstream ifs(path);
    if (!ifs.good()) {
        LOG_ERR("Failed opening file " << path);
        return false;
    }

    m_pmax = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    m_pmin = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    
    while (!ifs.eof()) {
        string line;
        getline(ifs, line);
        if (line.size() < 4 || line[0] == '#')
            continue;
        char* linep = (char*)line.c_str() + 2;
        char* lineend = (char*)line.c_str() + line.size();
        if (line[0] == 'v' && line[1] == ' ') {
            Vec3 v;
            v.x = strtod(linep, &linep);
            v.y = strtod(linep, &linep);
            v.z = strtod(linep, &linep);
            m_vtx.push_back(v);
            m_pmax.pmax(v);
            m_pmin.pmin(v);
        }
        else if (line[0] == 'f') {
            int a = strtol(linep, &linep, 10);
            while (*linep != ' ' && linep < lineend)
                ++linep;
            int b = strtol(linep, &linep, 10);
            while (*linep != ' ' && linep < lineend)
                ++linep;
            int c = strtol(linep, &linep, 10);
            m_idx.insert(m_idx.end(), {a-1,b-1,c-1}); // indices area 1 based
        }
    }

    LOG("loaded mesh min=" << m_pmin << " - " << m_pmax << ": " << m_vtx[0] << m_vtx[1] << m_vtx[2]);


    return true;
}

bool Mesh::loadFromJs(const char* objname)
{
#ifdef EMSCRIPTEN
    m_pmax = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    m_pmin = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    
    
    int vtx_len = EM_ASM_INT(return window[Pointer_stringify($0)].vertexPositions.length, objname);
    m_vtx.reserve(vtx_len);
    for(int i = 0; i < vtx_len; i += 3) {
        Vec3 v;
        v.x = EM_ASM_DOUBLE(return window[Pointer_stringify($0)].vertexPositions[$1], objname, i);
        v.y = EM_ASM_DOUBLE(return window[Pointer_stringify($0)].vertexPositions[$1], objname, i+1);
        v.z = EM_ASM_DOUBLE(return window[Pointer_stringify($0)].vertexPositions[$1], objname, i+2);
        m_vtx.push_back(v);
        m_pmax.pmax(v);
        m_pmin.pmin(v);        
    }
    int idx_len = EM_ASM_INT(return window[Pointer_stringify($0)].triangles.length, objname);
    m_idx.reserve(idx_len);
    for(int i = 0; i < idx_len; ++i) {
        int a = EM_ASM_INT(return window[Pointer_stringify($0)].triangles[$1], objname, i); 
        m_idx.push_back(a);
    }
    LOG("Loaded " << objname << " vtx=" << vtx_len << "(" << m_vtx[0].x << ")" << " idx=" << idx_len << "(" << m_idx[0] << ") -- " 
        << m_pmin << " - " << m_pmax << ":  " << m_vtx[0] << m_vtx[1] << m_vtx[2]);
#endif
    return true;
}


void Mesh::scale(const Vec3& f) {
    for(auto& v: m_vtx) {
        v *= f;
    }
}

void Mesh::translate(const Vec3& t) {
    for (auto& v : m_vtx) {
        v += t;
    }
}

void Mesh::recalcMinMax() {
    m_pmax = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    m_pmin = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);

    for (auto& v : m_vtx) {
        m_pmax.pmax(v);
        m_pmin.pmin(v);
    }

    m_center = (m_pmax + m_pmin) * 0.5;
    LOG("mesh-center=" << m_center);
}

}