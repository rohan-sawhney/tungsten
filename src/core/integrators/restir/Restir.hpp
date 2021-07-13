#ifndef RESTIR_HPP_
#define RESTIR_HPP_

#include "samplerecords/LightSample.hpp"

#include "primitives/Primitive.hpp"

#include "sampling/SampleWarp.hpp"

#include "math/Quaternion.hpp"

#include "Timer.hpp"

namespace Tungsten {

struct RestirSample
{
    const Primitive *prim = nullptr;
    float pdf;
    Vec3f e;
    Vec3f p;
    Vec3f N;
    float geom;

    RestirSample() = default;
    RestirSample(const Primitive *prim_, float pdf_, Vec3f e_, Vec3f p_, Vec3f N_, float geom_) : prim(prim_), pdf(pdf_), e(e_), p(p_), N(N_), geom(geom_) {}

    float inversePdf() const { return prim ? 1.0f/pdf : 0.0f; }
};

struct RestirShadingContext
{
    IntersectionTemporary data;
    IntersectionInfo info;
    SurfaceScatterEvent event;

    void pointerFixup() { event.info = &info; }

    float q(const RestirSample &candidate) const
    {
        if (!candidate.prim)
            return 0.0f;

        Vec3f d = candidate.p - info.p;
        float distSq = d.lengthSq();
        d.normalize();

        float geom = candidate.prim->isInfinite() ? 0.0f : max(candidate.N.dot(-d), 0.0f)/distSq;
        float result = (info.bsdf->eval(event.makeWarpedQuery(event.wi, event.frame.toLocal(d)), false)*candidate.e).luminance()*geom;
        return result;
    }

    float q(const RestirSample &candidate, PathSampleGenerator &/*sampler*/) const
    {
        return q(candidate);
    }
};

struct RestirReservoir
{
    float weightSum = 0.0f;
    float selectedQ = 0.0f;
    RestirSample selected;
    static const int N = 1;
    int M = 0;
    int Z = 0;

    RestirShadingContext data;
    Vec3f throughput;
    Vec3f emission;
    bool valid = false;

    void addSample(const RestirSample &candidate, float invPdf, PathSampleGenerator &sampler, int cM = 1, int cZ = 1)
    {
        float q = data.q(candidate, sampler);
        float weight = q*invPdf*cM;

        M += cM;
        Z += cZ;
        weightSum += weight;
        if (weightSum*sampler.next1D() < weight) {
            selectedQ = q;
            selected = candidate;
        }
    }

    void clear()
    {
        weightSum = selectedQ = 0.0f;
        M = 0;
        Z = 0;
        selected = RestirSample();
    }

          RestirSample &getSelected(int /*i*/)       { return selected; }
    const RestirSample &getSelected(int /*i*/) const { return selected; }

    int &getZ(int /*i*/) { return Z; }

    float inversePdf(int /*n*/ = 1) const
    {
        return selectedQ == 0.0f ? 0.0f : (weightSum/Z)/selectedQ;
    }
};

template<typename TargetReservoir, typename SourceReservoir>
void addSamples(TargetReservoir &r, const SourceReservoir **reservoirs, int K, PathSampleGenerator &sampler)
{
    int totalM[4] = {0};
    for (int j = 0; j < K; ++j) {
        const auto &o = *reservoirs[j];
        for (int i = 0; i < o.N; ++i) {
            const auto &candidate = o.getSelected(i);
            float pi = o.data.q(candidate, sampler);
            float pSum = pi;
            for (int k = 0; k < K; ++k)
                if (k != j)
                    pSum += reservoirs[k]->data.q(candidate, sampler);
            float wi = pSum > 0.0f ? pi/(pSum/K) : 0.0f;

            r.addSample(candidate, wi*o.inversePdf(i), sampler, o.M/o.N, o.M/o.N);
            totalM[j] += o.M/o.N;
        }
    }
    /*for (int i = 0; i < r.N; ++i)
        for (int j = 0; j < K; ++j)
            if (reservoirs[j]->data.q(r.getSelected(i)) > 0.0f)
                r.getZ(i) += totalM[j];*/
}

enum RestirStage {
    RestirTraceHits,
    RestirTraceVisibility,
    RestirSpatialReuse,
    RestirTemporalReuse,
    RestirStoreTemporal,
    RestirShade,
};
static const RestirStage RestirStages[] = {
    RestirTraceHits,
    RestirTraceVisibility,
    RestirSpatialReuse,
    RestirSpatialReuse,
    RestirTraceVisibility,
    RestirShade
};

class Restir
{
    int _w, _h;
    uint32 _restirPhase;
    std::vector<RestirReservoir> _reservoirsA, _reservoirsB, _temporalReservoirs;

    std::vector<std::shared_ptr<Primitive>> &_lights;

public:
    Restir(int w, int h, std::vector<std::shared_ptr<Primitive>> &lights)
    : _w(w), _h(h), _restirPhase(0), _reservoirsA(w*h), _reservoirsB(w*h), _temporalReservoirs(w*h), _lights(lights)
    {
    }

    void clear()
    {
        for (int i = 0; i < _w*_h; ++i)
            _reservoirsA[i] = _reservoirsB[i] = RestirReservoir();
        _restirPhase = 0;
    }

    RestirReservoir &getReservoir(Vec2u pixel) { return _reservoirsA[pixel.x() + pixel.y()*_w]; }
    bool inside(Vec2u pixel) { return pixel.x() < uint32(_w) && pixel.y() < uint32(_h); }

    RestirStage getStage() const { return RestirStages[_restirPhase]; }

    void advance(UniformSampler &/*rng*/)
    {
        RestirStage stage = getStage();
        _restirPhase++;

        if (stage == RestirSpatialReuse)
            finalizeSpatialReuse();
        if (stage == RestirTemporalReuse)
            finalizeTemporalReuse();
        if (stage == RestirStoreTemporal)
            storeTemporalSamples();
        if (stage == RestirShade)
            _restirPhase = 0;
    }

    float pairwiseMis(const RestirShadingContext &c, const RestirShadingContext &n, int k, const RestirSample &s, PathSampleGenerator &sampler)
    {
        float qc = c.q(s, sampler);
        float qn = n.q(s, sampler);

        return qn == 0.0f ? 0.0f : qn/(qn + qc/k);
    }

    void spatialReuse(Vec2u pixel, PathSampleGenerator &sampler)
    {
        const RestirReservoir &c = getReservoir(pixel);
        RestirReservoir r = c;
        r.clear();
        if (c.valid) {
            const int Radius = 32;

            float mc = 1.0f;
            int realK = 1;
            const int K = 5;
            for (int k = 0; k < K; ++k) {
                Vec2i p = Vec2i(pixel) + Vec2i((sampler.next2D() - 0.5f)*Radius);
                if (p[0] < 0 || p[1] < 0 || p[0] >= _w || p[1] >= _h)
                    continue;

                const RestirReservoir &n = getReservoir(Vec2u(p));
                if (n.valid) {
                    realK++;
                    float mo =   pairwiseMis(c.data, n.data, K, n.selected, sampler);
                    mc += 1.0f - pairwiseMis(c.data, n.data, K, c.selected, sampler);

                    r.addSample(n.selected, mo*n.inversePdf(), sampler);
                }
            }

            r.addSample(c.selected, mc*c.inversePdf(), sampler);
        }

        _reservoirsB[pixel.x() + pixel.y()*_w] = r;
    }

    void temporalReuse(Vec2u pixel, PathSampleGenerator &sampler)
    {
        const RestirReservoir &o = getReservoir(pixel);
        const RestirReservoir &n = _temporalReservoirs[pixel.x() + pixel.y()*_w];

        RestirReservoir r = o;
        r.clear();
        if (o.valid) {
            r.addSample(o.selected, o.inversePdf(), sampler, o.M);
            if (n.valid)
                r.addSample(n.selected, n.inversePdf(), sampler, min(n.M, 20*o.M));
        }

        _reservoirsB[pixel.x() + pixel.y()*_w] = r;
    }

    void finalizeTemporalReuse()
    {
        _reservoirsA.swap(_reservoirsB);
        for (auto &r : _reservoirsA)
            r.data.pointerFixup();
    }

    void storeTemporalSamples()
    {
        std::memcpy(&_temporalReservoirs[0], &_reservoirsA[0], _reservoirsA.size()*sizeof(RestirReservoir));
        for (auto &r : _temporalReservoirs)
            r.data.pointerFixup();
    }

    void finalizeSpatialReuse()
    {
        _reservoirsA.swap(_reservoirsB);
        for (auto &r : _reservoirsA)
            r.data.pointerFixup();
    }
};

}

#endif /* RESTIR_HPP_ */
