#ifndef RESTIRTRACER_HPP_
#define RESTIRTRACER_HPP_

#include "integrators/TraceBase.hpp"

#include "RestirSettings.hpp"
#include "Restir.hpp"

namespace Tungsten {

class RestirTracer : public TraceBase
{
    RestirSettings _settings;

    RestirSample generateCandidate(SurfaceScatterEvent event, PathSampleGenerator &sampler, const Ray &parentRay);

public:
    RestirTracer(TraceableScene *scene, const RestirSettings &settings, uint32 threadId);

    void tracePrimaryHit(Vec2u pixel, PathSampleGenerator &sampler, Restir &restir);
    void traceVisibility(Vec2u pixel, PathSampleGenerator &sampler, Restir &restir);
    void spatialReuse   (Vec2u pixel, PathSampleGenerator &sampler, Restir &restir);
    void temporalReuse  (Vec2u pixel, PathSampleGenerator &sampler, Restir &restir);
    Vec3f shadeSample   (Vec2u pixel, PathSampleGenerator &sampler, Restir &restir);
};

}

#endif /* RESTIRTRACER_HPP_ */
