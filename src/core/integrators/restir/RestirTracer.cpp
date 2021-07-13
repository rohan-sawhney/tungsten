#include "RestirTracer.hpp"
#include "bsdfs/TransparencyBsdf.hpp"

namespace Tungsten {

RestirTracer::RestirTracer(TraceableScene *scene, const RestirSettings &settings, uint32 threadId)
: TraceBase(scene, settings, threadId),
  _settings(settings)
{
}

static float geomTerm(const Ray &ray, const IntersectionInfo &info)
{
    return info.primitive->isInfinite() ? 1.0f : std::abs(info.Ng.dot(ray.dir()))/sqr(ray.farT());
}

static Vec4f getRayDirLen(const RestirReservoir &r, const RestirSample &s)
{
    Vec3f d = s.p - r.data.info.p;
    float dist = d.length();
    d.normalize();
    return Vec4f(d[0], d[1], d[2], dist);
}

static float geomTerm(const RestirSample &s, Vec4f rayDirLen)
{
    return s.prim->isInfinite() ? 1.0f : std::abs(s.N.dot(rayDirLen.xyz()))/sqr(rayDirLen.w());
}

RestirSample RestirTracer::generateCandidate(SurfaceScatterEvent event, PathSampleGenerator &sampler, const Ray &parentRay)
{
    float brdfP = 0.2f;

    if (sampler.nextBoolean(brdfP)) {
        if (!event.info->bsdf->sample(event, false))
            return RestirSample();
        if (event.weight == 0.0f)
            return RestirSample();

        Vec3f wo = event.frame.toGlobal(event.wo);
        if (!isConsistent(event, wo))
            return RestirSample();

        Ray ray = parentRay.scatter(event.info->p, wo, event.info->epsilon);
        IntersectionTemporary data;
        IntersectionInfo info;
        if (!_scene->intersectLights(ray, data, info))
            return RestirSample();

        float brdfPdf = event.pdf;
        float areaPdf = data.primitive->directPdf(_threadId, data, info, event.info->p)*lightSelectionPdf(data.primitive);
        float geom = geomTerm(ray, info);
        float pdf = lerp(areaPdf, brdfPdf, brdfP)*geom;

        return RestirSample{
            data.primitive,
            pdf,
            data.primitive->evalDirect(data, info),
            info.p,
            info.Ng,
            geom
        };
    } else {
        float weight;
        auto light = chooseLight(sampler, event.info->p, weight);

        LightSample sample;
        if (!light->sampleDirect(_threadId, event.info->p, sampler, sample))
            return RestirSample();

        Ray ray = parentRay.scatter(event.info->p, sample.d, event.info->epsilon);
        IntersectionTemporary data;
        IntersectionInfo info;
        if (!light->intersect(ray, data) || std::abs(ray.farT() - sample.dist) > event.info->epsilon)
            return RestirSample();
        light->intersectionInfo(data, info);
        info.p = ray.pos() + ray.dir()*ray.farT();
        info.w = ray.dir();
        info.epsilon = 5e-4f;

        float areaPdf = sample.pdf*lightSelectionPdf(light);
        float brdfPdf = event.info->bsdf->pdf(event.makeWarpedQuery(event.wi, event.frame.toLocal(sample.d)));
        float geom = geomTerm(ray, info);
        sample.pdf = lerp(areaPdf, brdfPdf, brdfP)*geom;

        return RestirSample{light, sample.pdf, sample.e, info.p, info.Ng, geom};
    }
}

void RestirTracer::tracePrimaryHit(Vec2u pixel, PathSampleGenerator &sampler, Restir &restir)
{
    RestirReservoir &r = restir.getReservoir(pixel);
    r.valid = false;

    PositionSample point;
    if (!_scene->cam().samplePosition(sampler, point))
        return;
    DirectionSample direction;
    if (!_scene->cam().sampleDirection(sampler, point, pixel, direction))
        return;

    Vec3f throughput = point.weight*direction.weight;
    Ray ray(point.p, direction.d);
    ray.setPrimaryRay(true);

    IntersectionTemporary data;
    IntersectionInfo info;
    const Medium *medium = _scene->cam().medium().get();
    const bool includeSurfaces = _settings.includeSurfaces;

    int bounce = 0;
    bool didHit = _scene->intersect(ray, data, info);

    Vec3f emission(0.0f);

    while ((medium || didHit) && bounce < _settings.maxBounces) {
        bounce++;

        if (medium) {
            /*if (bounce > 1) {
                Vec3f estimate(0.0f);

                result += throughput*estimate;
            }*/
            // TODO: Medium interactions
            throughput *= medium->transmittance(sampler, ray, true, true);
        }
        if (!didHit || !includeSurfaces)
            break;

        const Bsdf &bsdf = *info.bsdf;

        SurfaceScatterEvent event = makeLocalScatterEvent(data, info, ray, &sampler);

        Vec3f transparency = bsdf.eval(event.makeForwardEvent(), false);
        float transparencyScalar = transparency.avg();

        Vec3f wo;
        if (sampler.nextBoolean(transparencyScalar)) {
            wo = ray.dir();
            throughput *= transparency/transparencyScalar;
        } else {
            event.requestedLobe = BsdfLobes::SpecularLobe;
            if (!bsdf.sample(event, false))
                break;

            wo = event.frame.toGlobal(event.wo);

            throughput *= event.weight;
        }

        bool geometricBackside = (wo.dot(info.Ng) < 0.0f);
        medium = info.primitive->selectMedium(medium, geometricBackside);

        ray = ray.scatter(ray.hitpoint(), wo, info.epsilon);

        if (std::isnan(ray.dir().sum() + ray.pos().sum()))
            break;
        if (std::isnan(throughput.sum()))
            break;

        if (bounce < _settings.maxBounces)
            didHit = _scene->intersect(ray, data, info);
    }

    if (!_settings.includeSurfaces)
        return;

    if (!didHit) {
        if (!medium && bounce > _settings.minBounces && _scene->intersectInfinites(ray, data, info))
            emission += throughput*info.primitive->evalDirect(data, info);
        return;
    }
    if (info.primitive->isEmissive() && bounce > _settings.minBounces)
        emission += throughput*info.primitive->evalDirect(data, info);

    r.data.data = data;
    r.data.info = info;
    r.data.event = makeLocalScatterEvent(r.data.data, r.data.info, ray, &sampler);

    const int M = 32;
    for (int i = 0; i < M; ++i) {
        auto candidate = generateCandidate(r.data.event, sampler, ray);
        r.addSample(candidate, candidate.inversePdf(), sampler);
    }

    r.emission = emission;
    r.throughput = throughput;
    r.valid = true;
}

void RestirTracer::traceVisibility(Vec2u pixel, PathSampleGenerator &/*sampler*/, Restir &restir)
{
    RestirReservoir &r = restir.getReservoir(pixel);
    if (r.valid && r.selected.prim) {
        Vec4f rayDirLen = getRayDirLen(r, r.selected);
        float eps = r.data.info.epsilon;

        Ray shadowRay = Ray(r.data.info.p, rayDirLen.xyz(), eps);
        IntersectionTemporary shadowData;
        IntersectionInfo info;
        if (r.selected.prim->intersect(shadowRay, shadowData) && std::abs(shadowRay.farT() - rayDirLen.w()) < eps) {
            r.selected.prim->intersectionInfo(shadowData, info);

            shadowRay.setFarT(shadowRay.farT()*(1.0f - eps));

            if (_scene->occluded(shadowRay))
                r.selected.prim = nullptr;
        }
    }
}

void RestirTracer::spatialReuse(Vec2u pixel, PathSampleGenerator &sampler, Restir &restir)
{
    restir.spatialReuse(pixel, sampler);
}

void RestirTracer::temporalReuse(Vec2u pixel, PathSampleGenerator &sampler, Restir &restir)
{
    restir.temporalReuse(pixel, sampler);
}

Vec3f RestirTracer::shadeSample(Vec2u pixel, PathSampleGenerator &/*sampler*/, Restir &restir)
{
    RestirReservoir &r = restir.getReservoir(pixel);
    if (!r.valid)
        return Vec3f(0.0f);

    Vec3f e = r.emission;
    if (r.selected.prim) {
        Vec4f rayDirLen = getRayDirLen(r, r.selected);
        Vec3f f = r.data.info.bsdf->eval(r.data.event.makeWarpedQuery(r.data.event.wi, r.data.event.frame.toLocal(rayDirLen.xyz())));
        e += r.throughput*f*r.selected.e*geomTerm(r.selected, rayDirLen)*r.inversePdf();

    }

    return e;
}

}
