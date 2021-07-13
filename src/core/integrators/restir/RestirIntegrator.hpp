#ifndef RESTIRINTEGRATOR_HPP_
#define RESTIRINTEGRATOR_HPP_

#include "RestirTracer.hpp"
#include "RestirSettings.hpp"
#include "Restir.hpp"

#include "integrators/Integrator.hpp"
#include "integrators/ImageTile.hpp"

#include "sampling/PathSampleGenerator.hpp"
#include "sampling/UniformSampler.hpp"

#include "thread/TaskGroup.hpp"

#include "math/MathUtil.hpp"


#include <thread>
#include <memory>
#include <vector>
#include <atomic>

namespace Tungsten {

class RestirIntegrator : public Integrator
{
    static CONSTEXPR uint32 TileSize = 16;

    RestirSettings _settings;

    std::shared_ptr<TaskGroup> _group;

    uint32 _w;
    uint32 _h;
    int _temporalFrame;

    UniformSampler _sampler;
    std::vector<std::unique_ptr<RestirTracer>> _tracers;

    std::vector<ImageTile> _tiles;

    std::unique_ptr<Restir> _reservoirs;

    void diceTiles();

    void doWork(uint32 id, uint32 tileId);
    void traceSample(uint32 id, uint32 tileId);

    virtual void saveState(OutputStreamHandle &out) override;
    virtual void loadState(InputStreamHandle &in) override;

public:
    RestirIntegrator();

    virtual void fromJson(JsonPtr value, const Scene &scene) override;
    virtual rapidjson::Value toJson(Allocator &allocator) const override;

    virtual void prepareForRender(TraceableScene &scene, uint32 seed) override;
    virtual void teardownAfterRender() override;

    virtual bool supportsResumeRender() const override;

    virtual void startRender(std::function<void()> completionCallback) override;
    virtual void waitForCompletion() override;
    virtual void abortRender() override;
    
    const RestirSettings &settings() const
    {
        return _settings;
    }
};

}

#endif /* RESTIRINTEGRATOR_HPP_ */
