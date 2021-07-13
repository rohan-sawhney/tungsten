#include "simulator/MarchingCubesMesher.hpp"
#include "simulator/DeformationMesher.hpp"
#include "simulator/Simulator.hpp"

#include "thread/ThreadUtils.hpp"
#include "thread/ThreadPool.hpp"

#include "math/Quaternion.hpp"
#include "math/MathUtil.hpp"
#include "math/Angle.hpp"
#include "math/Mat.hpp"
#include "math/Vec.hpp"
#include "math/Box.hpp"

#include "io/ImageIO.hpp"

#include "CameraControls.hpp"
#include "InstanceIO.hpp"
#include "Colormap.hpp"
#include "Memory.hpp"
#include "Timer.hpp"

#include "GL/glew.h"
#include <GLFW/glfw3.h>

#include <fstream>
#include <iomanip>
#include <atomic>

#define USE_GRAPHICS 1

using namespace Tungsten;

static const bool Record = false;
Profiler profiler;

Vec2r getMousePos(GLFWwindow *window)
{
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    return Vec2r(Real(xpos), Real(ypos));
}

UniformSampler rng(0xDEADBEEFBA5EBA11ULL);


#if USE_GRAPHICS
/*static const int GWidth = 1920;
static const int GHeight = 1080;*/
static const int GWidth = 1280;
static const int GHeight = 720;

#if SIMULATION_3D
template<int D> HOST DEVICE Vec4c MpmSimulator<D>::raytracePixel(int w, int h, float x, float y, Vec3f p0, Mat<float, 4> A, Real time) const
{
    Vec2f uv(x/w, y/w);
    Vec2r pos = uv*4.0f - Vec2r(1.0f, 0.0f);

    /*Vec3r g, i;
    Real d = evalScene(Vec3r(pos[0], pos[1], 0.0f), g, i, time);
    uint8 dxi = clamp(int(((std::abs(i[0])*10.0f + 0.0f)*0.5f)*255.0f), 0, 255);
    uint8 dyi = clamp(int(((std::abs(i[1])*10.0f + 0.0f)*0.5f)*255.0f), 0, 255);
    //uint8 di = clamp(int(((d + 0.1f)*2.0f)*255.0f), 0, 255);
    return Vec4c(dxi, dyi, uint8(0), uint8(255));*/

    Real z = 0.5;
    Vec4f in = A*(Vec4f(x/w, y/h, z, 1.0f)*2.0f - 1.0f);
    Vec3f dir = in.xyz()/in.w() - p0;
    dir.normalize();

    Vec3f p = p0;

    const Real maxT = 100;
    const Real eps = 1e-3f;
    Vec3f n, u;
    Real t = 0;
    for (int i = 0; i < 100; ++i) {
        Real d = evalScene(p + dir*t, n, u, time);

        t += d;

        if (t > maxT) {
            return Vec4c(Vec4i(128, 128, 128, 255));
        } else if (d < eps) {
            Vec3c c = Vec3c(clamp(Vec3i((n*0.5f + 0.5f)*255.0f), Vec3i(0), Vec3i(255)));
            return Vec4c(c[0], c[1], c[2], uint8(255));
        }
    }
    return Vec4c(Vec4i(0, 0, 0, 255));
}
template<int D> void MpmSimulator<D>::renderImage(Mat4f proj, Mat4f view, int w, int h, Array<Vec4c> dst, Real time) const
{
    Mat4f A2 = (proj*view).invert();
    Mat<float, 4> A;
    for (int i = 0; i < 16; ++i)
        A[i] = A2[i];

    Vec3f p0 = view.pseudoInvert()*Vec3f(0.0f);

    MpmSimulator that = *this;
    FOR_EACH(w*h, dst[i] = that.raytracePixel(w, h, (i % w) + 0.5f, (i/w) + 0.5f, p0, A, time); )
}
#endif

#if SIMULATION_3D

struct SimulatorState
{
    int iter = -1;
    std::vector<Vec3f> pos, o;
    std::vector<Vec3f> verts;
    std::vector<Vec3u> tris;
};

void draw(const SimulatorState &state, CameraControls &camera, const MpmSimulator &simulator, Real time, bool drawScene)
{
    Mat4f proj = Mat4f::perspective(60.0f, GWidth/float(GHeight), 1e-2f, 100.0f);
    Mat4f view = camera.toMatrix().pseudoInvert();

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj.transpose().data());
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view.transpose().data());

    if (drawScene) {
        Array<Vec4c> imageBuf(GWidth*GHeight);
        simulator.renderImage(proj, view, GWidth, GHeight, imageBuf, time);
        imageBuf.gpuToCpu();
        glDrawPixels(GWidth, GHeight, GL_RGBA, GL_UNSIGNED_BYTE, imageBuf.getCpu());
    }

    glClear(GL_DEPTH_BUFFER_BIT);

    glPointSize(4.0f);
    glBegin(GL_POINTS);
    glColor3f(0.25f, 0.25f, 0.25f);
    for (Vec3f v : state.pos)
        glVertex3fv(v.data());
    glEnd();

    /*glBegin(GL_QUADS);
    glColor3f(0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < state.pos.size(); i += 10) {
        Mat3f m = rotMatrix(state.o[i]);
        Vec3f n = (m*Vec3f(0.0f, 1.0f, 0.0f)).normalized();
        float r = quart(quart(max(n.y(), 0.0f)));
        glColor3f(r, r, r);
        Real size = 0.01f;
        Vec3f p00 = state.pos[i] + m*Vec3f(-1.0f, 0.0f, -1.0f)*size;
        Vec3f p10 = state.pos[i] + m*Vec3f( 1.0f, 0.0f, -1.0f)*size;
        Vec3f p11 = state.pos[i] + m*Vec3f( 1.0f, 0.0f,  1.0f)*size;
        Vec3f p01 = state.pos[i] + m*Vec3f(-1.0f, 0.0f,  1.0f)*size;
        glVertex3fv(p00.data());
        glVertex3fv(p10.data());
        glVertex3fv(p11.data());
        glVertex3fv(p01.data());
    }
    glEnd();*/

    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);
    for (const auto &t : state.tris) {
        Vec3f v0 = state.verts[t[0]];
        Vec3f v1 = state.verts[t[1]];
        Vec3f v2 = state.verts[t[2]];
        Vec3r n = (v1 - v0).cross(v2 - v0).normalized();
        glColor3fv((n*0.5f + 0.5f).data());
        glVertex3fv(v0.data());
        glVertex3fv(v1.data());
        glVertex3fv(v2.data());
    }
    glEnd();
}

void saveData(const SimulatorState &state, bool compress)
{
    if (!Record)
        return;

    const char *ext = compress ? "zdat" : "dat";
    auto out = FileUtils::openOutputStream(Path(tfm::format("D:/frames/%04d.%s", state.iter, ext)));
    if (!out)
        return;

    uint32 instanceCount = state.pos.size();
    FileUtils::streamWrite(out, instanceCount);
    FileUtils::streamWrite(out, uint32(compress ? CompressionLossy : 0));

    Box3f bounds;
    for (uint32 i = 0; i < instanceCount; ++i)
        bounds.grow(state.pos[i]);
    FileUtils::streamWrite(out, bounds);

    if (compress) {
        for (uint32 i = 0; i < instanceCount; ++i)
            saveLossyInstance(out, bounds, state.pos[i], state.o[i]);
    } else {
        for (uint32 i = 0; i < instanceCount; ++i)
            saveLosslessInstance(out, state.pos[i], state.o[i]);
    }
    for (uint32 i = 0; i < instanceCount; ++i)
        FileUtils::streamWrite(out, uint8(0));
}
#else

class MpmVisualizer
{
    MpmSimulator<2> &_sim;
    const LevelSet<2> &_scene;
    const BinMapper &_binner;

    Box2r _bounds;
    Vec2r _pixelScale;
    Box2u _cellBounds;

    std::unique_ptr<MarchingCubesMesher<2>> _mcMesher;
    std::unique_ptr<DeformationMesher<2>> _deformMesher;

    Vec2r worldToPixel(Vec2r p) const { return (p - _bounds.min())*_pixelScale; }
    Vec2r cellToPixel(Vec2u q) const { return worldToPixel(_binner.unmapF(q)); }

    void drawGrid()
    {
        glBegin(GL_LINES);
        glColor3f(0.7, 0.7, 0.7);
        for (auto y : _cellBounds.range(1)) {
            auto p0 = cellToPixel(Vec2u(_cellBounds.min().x(), y));
            auto p1 = cellToPixel(Vec2u(_cellBounds.max().x(), y));
            glVertex2f(p0[0], p0[1]);
            glVertex2f(p1[0], p1[1]);
        }
        for (auto x : _cellBounds.range(0)) {
            auto p0 = cellToPixel(Vec2u(x, _cellBounds.min().y()));
            auto p1 = cellToPixel(Vec2u(x, _cellBounds.max().y()));
            glVertex2f(p0[0], p0[1]);
            glVertex2f(p1[0], p1[1]);
        }
        glEnd();
    }

    void drawMesh(const Mesh<2> &mesh)
    {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, GL_ALWAYS, GL_ALWAYS);
        glStencilOp(GL_INVERT, GL_INVERT, GL_INVERT);

        glColorMask(false, false, false, false);

        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < mesh.tris.size(); ++i) {
            glVertex2fv(worldToPixel(mesh.verts[mesh.tris[i][0]]).data());
            glVertex2fv(worldToPixel(mesh.verts[mesh.tris[i][1]]).data());
            glVertex2f(GWidth*0.5f, GHeight*0.5f);
        }
        glEnd();

        glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

        glColorMask(true, true, true, true);

        glBegin(GL_QUADS);
        glColor3f(0.5f, 0.5f, 0.5f);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 0.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex2f(GWidth, 0.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex2f(GWidth, GHeight);
        glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, GHeight);
        glEnd();

        glDisable(GL_STENCIL_TEST);
    }
    void drawMesh(const Array<Vec2r> &verts)
    {

#if 0
        uint32 wi = (b1[0] - b0[0])*MetaSize;
        uint32 hi = (b1[1] - b0[1])*MetaSize;
        auto data = zeroAlloc<Vec4c>(wi*hi);
        auto density = zeroAlloc<uint8>(wi*hi);

        //const Vec3f sigmaT = -std::log(Vec3f(38.0f, 150.0f, 170.0f)/256.0f)*4.0f;
        //const Vec3f sigmaT = Vec3f(0.6100000143051147f, 0.3199999928474426f, 1.5f)*4.0f;
        const Vec3f sigmaT = Vec3f(1.5f, 0.6100000143051147f, 0.3199999928474426f)*4.0f;

        _dist.gpuToCpu();
        for (uint32 i = 0; i < _metaCount; ++i) {
            VecDu q = cellToCoord(i);
            if (q[0] < b0[0] || q[1] < b0[1])
                continue;
            for (int y = 0; y < MetaSize; ++y) {
                for (int x = 0; x < MetaSize; ++x) {
                    Vec2u xy = (q - b0)*MetaSize + Vec2u(x, y);
                    if (xy[0] >= wi || xy[1] >= hi)
                        continue;
                    //Vec3f c = std::exp(-sigmaT*sqr(1.0f - _dist[i](x, y)*0.8f));
                    //Vec3f c = sqr(min(1.0f - std::exp(-sigmaT*_dist[i](x, y)*0.00013f), Vec3f(1.0f)));
                    //int idx = min(int(255.0f*_dist[i](x, y)*0.00013f*1.5f), 255);
                    //float *cm = parula_cm[idx];
                    //float *cm = viridis_cm[idx];
                    //float *cm = plasma_cm[idx];
                    //float *cm = magma_cm[idx];
                    //float *cm = inferno_cm[idx];
                    //Vec3f c(cm[0], cm[1], cm[2]);
                    Vec3f c(std::exp(-Vec3f(0.9f, 0.95f, 1.0f)*_dist[i](x, y)*0.075f));
                    //Vec3f c = lerp(std::exp(-sigmaT), Vec3f(0.9f), 1.0f - min(_dist[i](x, y)*0.0001f, 1.0f));
                    Vec4f ca(c[0], c[1], c[2], 1.0f);
                    data[xy[0] + xy[1]*wi] = Vec4c(clamp(Vec4i(ca*255.0f), Vec4i(0), Vec4i(255)));
                    density[xy[0] + (hi - 1 - xy[1])*wi] = clamp(int(_dist[i](x, y)*255.0f), 0, 255);
                }
            }
        }

        glEnable(GL_TEXTURE_2D);
        GLuint tex;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, wi, hi, 0, GL_RGBA, GL_UNSIGNED_BYTE, &data[0].x());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
#endif

        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, GL_ALWAYS, GL_ALWAYS);
        glStencilOp(GL_INVERT, GL_INVERT, GL_INVERT);

        glColorMask(false, false, false, false);

        glBegin(GL_TRIANGLES);
        for (uint32 i = 0; i < verts.size(); i += 2) {
            glVertex2fv(worldToPixel(verts[i + 0]).data());
            glVertex2fv(worldToPixel(verts[i + 1]).data());
            glVertex2f(GWidth*0.5f, GHeight*0.5f);
        }
        glEnd();

        glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

        glColorMask(true, true, true, true);

        glBegin(GL_QUADS);
        glColor3f(0.5f, 0.5f, 0.5f);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 0.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex2f(GWidth, 0.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex2f(GWidth, GHeight);
        glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, GHeight);
        glEnd();

        glDisable(GL_STENCIL_TEST);

#if 0
        glDeleteTextures(1, &tex);
        glDisable(GL_TEXTURE_2D);
#endif
    }

    void drawScene()
    {
        glBegin(GL_TRIANGLES);

        Vec2u cellExtent = _cellBounds.diagonal() + 2;
        uint32 w = cellExtent[0];
        uint32 h = cellExtent[1];

        std::unique_ptr<Real[]> d(new Real[w*h]);
        for (uint32 y = 0; y < h; ++y) {
            for (uint32 x = 0; x < w; ++x) {
                d[x + y*w] = _scene.distance(_binner.unmapF(Vec2u(x, y) + _cellBounds.min())).d;
            }
        }

        for (uint32 y = 0; y < h - 1; ++y) {
            for (uint32 x = 0; x < w - 1; ++x) {
                auto addTri = [&](Vec2r v0, Vec2r v1, Vec2r v2) {
                    glColor3f(0.0f, 0.0f, 0.0f);
                    glVertex2fv(v0.data());
                    glVertex2fv(v1.data());
                    glVertex2fv(v2.data());
                };
                auto addQuad = [&](Vec2r v0, Vec2r v1, Vec2r v2, Vec2r v3) {
                    addTri(v0, v1, v2);
                    addTri(v0, v2, v3);
                };
                auto addPenta = [&](Vec2r v0, Vec2r v1, Vec2r v2, Vec2r v3, Vec2r v4) {
                    addTri(v0, v1, v2);
                    addTri(v0, v2, v3);
                    addTri(v0, v3, v4);
                };

                Real d00 = d[(x + 0) + (y + 0)*w];
                Real d10 = d[(x + 1) + (y + 0)*w];
                Real d11 = d[(x + 1) + (y + 1)*w];
                Real d01 = d[(x + 0) + (y + 1)*w];
                int idx = (d00 < 0)*1 + (d10 < 0)*2 + (d11 < 0)*4 + (d01 < 0)*8;
                if (idx == 0)
                    continue;

                auto p0 = cellToPixel(Vec2u(x, y) + _cellBounds.min() + 0);
                auto p1 = cellToPixel(Vec2u(x, y) + _cellBounds.min() + 1);

                Vec2r v00(p0[0], p0[1]), v10(p1[0], p0[1]);
                Vec2r v01(p0[0], p1[1]), v11(p1[0], p1[1]);

                if (idx == 15) {
                    addQuad(v00, v10, v11, v01);
                    continue;
                }

                Vec2r i0 = lerp(v00, v10, d00/(d00 - d10));
                Vec2r i1 = lerp(v10, v11, d10/(d10 - d11));
                Vec2r i2 = lerp(v11, v01, d11/(d11 - d01));
                Vec2r i3 = lerp(v01, v00, d01/(d01 - d00));

                switch (idx) {
                case 0: break;
                case 1: addTri(v00, i0, i3); break;
                case 2: addTri(v10, i1, i0); break;
                case 4: addTri(v11, i2, i1); break;
                case 8: addTri(v01, i3, i2); break;
                case  3: addQuad(v00, v10, i1, i3); break;
                case  6: addQuad(v10, v11, i2, i0); break;
                case 12: addQuad(v11, v01, i3, i1); break;
                case  9: addQuad(v01, v00, i0, i2); break;
                case 14: addPenta(i0, v10, v11, v01, i3); break;
                case 13: addPenta(i1, v11, v01, v00, i0); break;
                case 11: addPenta(i2, v01, v00, v10, i1); break;
                case  7: addPenta(i3, v00, v10, v11, i2); break;
                case  5: addTri(v00, i0, i3); addTri(v11, i2, i1); break;
                case 10: addTri(v10, i1, i0); addTri(v01, i3, i2); break;
                case 15: addQuad(v00, v10, v11, v01); break;
                }
            }
        }
        glEnd();
    }

    void drawParticles()
    {
        if (_sim.ghostParticles().size() > 0) {
            glBegin(GL_LINES);
            glColor3f(0.0f, 0.0f, 0.0f);
            for (size_t i = 0; i < _sim.ghostParticles().size(); ++i) {
                auto rotM = rotMatrix(_sim.ghostParticles().o(i));
                auto dx = rotM*Vec2r(5.0f, 0.0f);
                auto p0 = worldToPixel(_sim.ghostParticles().x(i));
                glVertex2fv((p0 + dx).data());
                glVertex2fv((p0 - dx).data());
            }
            glEnd();
        } else {
            _sim.particles().xToCpu();
            glPointSize(4.0f);
            glBegin(GL_POINTS);
            glColor3f(0.0f, 0.0f, 0.0f);
            for (size_t i = 0; i < _sim.particles().size(); ++i)
                glVertex2fv(worldToPixel(_sim.particles().x(i)).data());
            glEnd();
        }
    }

    void drawNormals()
    {
        glBegin(GL_LINES);
        for (size_t i = 0; i < _sim.cellCount(); ++i) {
            Vec2r p0 = cellToPixel(_sim.cellToCoord(i));
            Vec2r p1 = p0 + _sim.grid().N(i)*10.0f;
            glColor3f(0, 1, 1);
            glVertex2f(p0[0], p0[1]);
            glVertex2f(p1[0], p1[1]);
        }
        glEnd();
    }

    void drawActiveCells()
    {
        glBegin(GL_QUADS);
        for (size_t i = 0; i < _sim.cellCount(); ++i) {
            Vec2r p0 = cellToPixel(_sim.cellToCoord(i));
            Vec2r p1 = cellToPixel(_sim.cellToCoord(i) + 1);
            glColor3f(0, 0.3f, 0);
            glVertex2f(p0[0], p0[1]);
            glVertex2f(p1[0], p0[1]);
            glVertex2f(p1[0], p1[1]);
            glVertex2f(p0[0], p1[1]);
        }
        glEnd();
    }

public:
    MpmVisualizer(MpmSimulator<2> &sim, const Json &j)
    : _sim(sim),
      _scene(sim.scene()),
      _binner(sim.binner())
    {
        Box2r bounds = _scene.bounds();
        Vec2r size = bounds.diagonal();
        Real aspect = Real(GWidth)/GHeight;
        size = max(size, Vec2r(size[1]*aspect, size[0]/aspect));
        Vec2r center = bounds.center();
        _bounds = Box2r(center - size*0.5f, center + size*0.5f);
        _pixelScale = Vec2r(GWidth, GHeight)/_bounds.diagonal();
        _cellBounds = Box2u(
            _sim.binner().map(_bounds.min()),
            _sim.binner().map(_bounds.max())
        );

        if (_sim.mesher() == MesherMarchingCubes)
            _mcMesher.reset(new MarchingCubesMesher<2>(_sim, j["mesher"]));
        else if (_sim.mesher() == MesherDeformation)
            _deformMesher.reset(new DeformationMesher<2>(_sim, j["mesher"]));
    }

    void draw()
    {
        glDisable(GL_DEPTH_TEST);

        drawGrid();

        if (_sim.mesher() == MesherMarchingCubes) {
            _mcMesher->update(_sim, profiler);
            drawMesh(_mcMesher->verts());
        } else if (_sim.mesher() == MesherDeformation) {
            _deformMesher->update(_sim);
            drawMesh(_deformMesher->mesh());
        } else {
            drawParticles();
        }

        drawScene();
    }
};
#endif
#endif

GLFWwindow *window;

void saveImage(int iter)
{
    static std::shared_ptr<TaskGroup> recordTask;
    static std::unique_ptr<Vec3c[]> buf;

    if (!Record)
        return;

    int SWidth, SHeight;
    glfwGetFramebufferSize(window, &SWidth, &SHeight);
    SWidth = GWidth;
    SHeight = GHeight;
    int AA = SWidth/GWidth;

	if (!buf) {
		buf.reset(new Vec3c[SWidth*SHeight]);
		std::cout << "Allocated of size " << SWidth << " " << SHeight << std::endl;
	}

    if (recordTask)
        recordTask->wait();
    glReadPixels(0, 0, SWidth, SHeight, GL_RGB, GL_UNSIGNED_BYTE, buf.get());

    Vec3c *bufP = buf.get();
    recordTask = ThreadUtils::pool->enqueue([AA, SWidth, bufP, iter](uint32, uint32, uint32) {
        for (int y = 0; y < GHeight; ++y) {
            for (int x = 0; x < GWidth; ++x) {
                Vec3i sum(0, 0, 0);
                for (int yi = 0; yi < AA; ++yi)
                    for (int xi = 0; xi < AA; ++xi)
                        sum += Vec3i(bufP[(x*AA + xi) + (y*AA + yi)*SWidth]);

                bufP[x + y*GWidth] = Vec3c(sum/(AA*AA));
            }
        }

        for (int y = 0; y < GHeight/2; ++y)
            for (int x = 0; x < GWidth; ++x)
                std::swap(bufP[x + y*GWidth], bufP[x + (GHeight - 1 - y)*GWidth]);
        ImageIO::saveLdr(Path(tfm::format("frames/%05d.png", iter)), &bufP[0][0], GWidth, GHeight, 3);
    }, 1, [](){});
}

void setupGraphics()
{
    if (!glfwInit())
        return;

    rng.next2D();
    rng.next2D();
    rng.next2D();

    glfwWindowHint(GLFW_SAMPLES, 16);
    window = glfwCreateWindow(GWidth, GHeight, "FEM Soft Body", NULL, NULL);
    glfwMakeContextCurrent(window);

    glfwSwapInterval(0);

    glMatrixMode(GL_PROJECTION_MATRIX);
    glOrtho(0.0f, GWidth, 0.0f, GHeight, -1.0f, 1.0f);

    glEnable(GL_DEPTH_TEST);
}

template<int D>
void femMain(const Json &j)
{
    setupGraphics();

    MpmSimulator<D> simulator(j);

#if SIMULATION_3D
    Mesh mesh;
    std::mutex stateMutex;
    std::deque<std::unique_ptr<SimulatorState>> states;
    std::unique_ptr<SimulatorState> state;
    int oldIter = -1;
#endif

    /*uint32 startIter;
    Path backupPath("checkpoint.dat");
    if (backupPath.exists() && false) {
        auto in = FileUtils::openInputStream(backupPath);
        FileUtils::streamRead(in, startIter);
        simulator.load(in);
    } else {
        startIter = 0;
    }
    iter = startIter;*/

    simulator.prepareForSolve();

    MpmVisualizer vis(simulator, j);

    CameraControls camera;
    camera.set(Vec3f(4.0f, 2.0f, 4.0f), Vec3f(0.0f), Vec3f(0.0f, 1.0f, 0.0f));

#if SIMULATION_3D
    auto simulatorStep = [&]() {
        try {

        uint32 iter = startIter;

        while (iter < 60*60*3) {
            Timer timer;
            /*if ((iter % 4) == 0) {
                auto out = FileUtils::openOutputStream(backupPath);
                FileUtils::streamWrite(out, iter);
                simulator.save(out);
            }*/
            timer.bench("Save checkpoint"); timer.start();
            simulator.updateImplicitOptimization(profiler, iter*dt, dt, 0.2f);
            timer.bench("Time per frame");

            std::unique_ptr<SimulatorState> newState(new SimulatorState);
            if (simulator.ghostParticles().size()) {
                newState->pos.resize(simulator.ghostParticles().size());
                newState->  o.resize(simulator.ghostParticles().size());
                for (uint32 i = 0; i < simulator.ghostParticles().size(); ++i) {
                    newState->pos[i] = simulator.ghostParticles().x(i);
                    newState->  o[i] = simulator.ghostParticles().o(i);
                }
            } else {
                newState->pos.resize(simulator.particles().size());
                newState->  o.resize(simulator.particles().size());
                for (uint32 i = 0; i < simulator.particles().size(); ++i) {
                    newState->pos[i] = simulator.particles().x(i);
                    newState->  o[i] = Vec3f(0.0f, 1.0f, 0.0f);
                }
            }
            newState->verts.resize(simulator.mesh().verts.size());
            for (uint32 i = 0; i < simulator.mesh().verts.size(); ++i)
                newState->verts[i] = simulator.mesh().verts[i];
            newState->tris.resize(simulator.mesh().tris.size());
            for (uint32 i = 0; i < simulator.mesh().tris.size(); ++i) {
                newState->tris[i] = simulator.mesh().tris[i];
            }
            newState->iter = iter;

            {
                std::lock_guard<std::mutex> guard(stateMutex);
                states.emplace_back(std::move(newState));
            }
            if (iter == 0)
                while (!startSim) std::this_thread::sleep_for(std::chrono::milliseconds(100));
            iter++;
        }

        } catch(const std::exception &e) {
            std::cout << "Whoopsie!" << std::endl;
            std::cout << e.what() << std::endl;
            std::_Exit(0);
        }
    };
    std::thread simulatorThread(simulatorStep);
    simulatorThread.detach();

    camera.set(Vec3f(-0.772652f, 2.45832f, 2.24487f), Vec3f(-0.126015f, 0.602182f, -0.439287f), Vec3f(0.0f, 1.0f, 0.0f));

    bool drawScene = false;
#endif

    while (!glfwWindowShouldClose(window) && !glfwGetKey(window, GLFW_KEY_ESCAPE)) {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        camera.update(window, GWidth, GHeight, 60.0f);

#if SIMULATION_3D
        {
            std::lock_guard<std::mutex> guard(stateMutex);
            if (!states.empty()) {
                state = std::move(states.front());
                states.pop_front();
                drawScene = true;
            }
        }
        if (state) {
            draw(*state, camera, simulator, state->iter*dt, drawScene);

            if (oldIter != state->iter) {
                oldIter = state->iter;
                //saveImage(oldIter);
                Timer timer;
#if HYPERELASTIC
                saveWo3(Path(tfm::format("D:/frames/jelly/%04d.wo3", state->iter)), state->verts, state->tris);
#else
                saveData(*state, true);
#endif
                timer.bench("Save ghost particles");
            }

            if (startSim)
                drawScene = false;
        }
#else
        /*{
            auto out = FileUtils::openOutputStream(backupPath);
            FileUtils::streamWrite(out, iter);
            simulator.save(out);
        }*/

        simulator.update(profiler);

        vis.draw();
#endif

        glfwSwapBuffers(window);
        glfwPollEvents();

        //saveImage(simulator.iteration());
    }

    profiler.print();

    std::_Exit(0);
}

int main()
{
    ThreadUtils::startThreads(max(ThreadUtils::idealThreadCount() - 1, 1u));

    std::string err;
    auto config = Json::parse(FileUtils::loadText("data/aurum/config.json"), err);

    if (!config) {
        std::cerr << err << std::endl;
        return -1;
    }

    int dimension = 2;
    config.getOptional("dimension", dimension);

    /*if (dimension == 3)
        femMain<3>(config);
    else*/
        femMain<2>(config);

    return 0;
}
