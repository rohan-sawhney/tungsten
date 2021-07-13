#include "RestirIntegrator.hpp"
#include "sampling/UniformPathSampler.hpp"
#include "sampling/SobolPathSampler.hpp"

#include "cameras/Camera.hpp"

#include "thread/ThreadUtils.hpp"
#include "thread/ThreadPool.hpp"

namespace Tungsten {

CONSTEXPR uint32 RestirIntegrator::TileSize;

RestirIntegrator::RestirIntegrator()
: Integrator(),
  _w(0),
  _h(0),
  _sampler(0xBA5EBA11)
{
}

void RestirIntegrator::diceTiles()
{
    for (uint32 y = 0; y < _h; y += TileSize) {
        for (uint32 x = 0; x < _w; x += TileSize) {
            _tiles.emplace_back(
                x,
                y,
                min(TileSize, _w - x),
                min(TileSize, _h - y),
                _scene->rendererSettings().useSobol() ?
                    std::unique_ptr<PathSampleGenerator>(new SobolPathSampler(MathUtil::hash32(_sampler.nextI()))) :
                    std::unique_ptr<PathSampleGenerator>(new UniformPathSampler(MathUtil::hash32(_sampler.nextI())))
            );
        }
    }
}

void RestirIntegrator::doWork(uint32 id, uint32 tileId)
{
    int stage = _reservoirs->getStage();

    ImageTile &tile = _tiles[tileId];
    for (uint32 y = 0; y < tile.h; ++y) {
        for (uint32 x = 0; x < tile.w; ++x) {
            Vec2u pixel(tile.x + x, tile.y + y);
            uint32 pixelIndex = pixel.x() + pixel.y()*_w;

            Vec3f c;
            tile.sampler->startPath(pixelIndex, _currentSpp);
            switch (stage) {
            case RestirTraceHits      :     _tracers[id]->tracePrimaryHit(pixel, *tile.sampler, *_reservoirs); break;
            case RestirTraceVisibility:     _tracers[id]->traceVisibility(pixel, *tile.sampler, *_reservoirs); break;
            case RestirSpatialReuse   :     _tracers[id]->spatialReuse   (pixel, *tile.sampler, *_reservoirs); break;
            case RestirTemporalReuse  :     _tracers[id]->temporalReuse  (pixel, *tile.sampler, *_reservoirs); break;
            case RestirShade          : c = _tracers[id]->shadeSample    (pixel, *tile.sampler, *_reservoirs); break;
            }

            if (stage == RestirShade && _temporalFrame == _settings.temporalFrames)
                _scene->cam().colorBuffer()->addSample(pixel, c);
        }
    }
}

void RestirIntegrator::saveState(OutputStreamHandle &out)
{
    for (ImageTile &i : _tiles)
        i.sampler->saveState(out);
}

void RestirIntegrator::loadState(InputStreamHandle &in)
{
    for (ImageTile &i : _tiles)
        i.sampler->loadState(in);
}

void RestirIntegrator::fromJson(JsonPtr value, const Scene &/*scene*/)
{
    _settings.fromJson(value);
}

rapidjson::Value RestirIntegrator::toJson(Allocator &allocator) const
{
    return _settings.toJson(allocator);
}

void RestirIntegrator::prepareForRender(TraceableScene &scene, uint32 seed)
{
    _currentSpp = 0;
    _sampler = UniformSampler(MathUtil::hash32(seed));
    _scene = &scene;
    advanceSpp();
    if (_scene->rendererSettings().sppStep() != 1)
        std::cout << "Warning: SPP step is not 1. This will not work as expected!" << std::endl;
    scene.cam().requestColorBuffer();

    for (uint32 i = 0; i < ThreadUtils::pool->threadCount(); ++i)
        _tracers.emplace_back(new RestirTracer(&scene, _settings, i));

    _w = scene.cam().resolution().x();
    _h = scene.cam().resolution().y();
    _reservoirs.reset(new Restir(_w, _h, scene.lights()));
    diceTiles();

    _temporalFrame = 0;
}

void RestirIntegrator::teardownAfterRender()
{
    _group.reset();

    _reservoirs.reset();
    _tracers.clear();
    _tiles  .clear();
    _tracers.shrink_to_fit();
    _tiles  .shrink_to_fit();
}

bool RestirIntegrator::supportsResumeRender() const
{
    return true;
}

void RestirIntegrator::startRender(std::function<void()> completionCallback)
{
    if (done()) {
        _currentSpp = _nextSpp;
        advanceSpp();
        completionCallback();
        return;
    }

    using namespace std::placeholders;
    _group = ThreadUtils::pool->enqueue(
        std::bind(&RestirIntegrator::doWork, this, _3, _1),
        _tiles.size(),
        [&, completionCallback]() {
            RestirStage stage = _reservoirs->getStage();
            _reservoirs->advance(_sampler);

            if (stage == RestirShade) {
                _temporalFrame++;
                if (_temporalFrame > _settings.temporalFrames) {
                    _temporalFrame = 0;
                    _reservoirs->clear();
                    _currentSpp = _nextSpp;
                    advanceSpp();
                }
            }
            completionCallback();
        }
    );
}

void RestirIntegrator::waitForCompletion()
{
    if (_group) {
        _group->wait();
        _group.reset();
    }
}

void RestirIntegrator::abortRender()
{
    if (_group) {
        _group->abort();
        _group->wait();
        _group.reset();
    }
}

}
