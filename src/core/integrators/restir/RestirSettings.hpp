#ifndef RESTIRSETTINGS_HPP_
#define RESTIRSETTINGS_HPP_

#include "integrators/TraceSettings.hpp"

#include "io/JsonObject.hpp"

namespace Tungsten {

struct RestirSettings : public TraceSettings
{
    bool includeSurfaces;
    int temporalFrames;

    RestirSettings()
    : includeSurfaces(true),
      temporalFrames(8)
    {
    }

    void fromJson(JsonPtr value)
    {
        TraceSettings::fromJson(value);
        value.getField("include_surfaces", includeSurfaces);
        value.getField("temporal_frames", temporalFrames);
    }

    rapidjson::Value toJson(rapidjson::Document::AllocatorType &allocator) const
    {
        return JsonObject{TraceSettings::toJson(allocator), allocator,
            "type", "restir",
            "include_surfaces", includeSurfaces,
            "temporal_frames", temporalFrames
        };
    }
};

}

#endif /* RESTIRSETTINGS_HPP_ */
