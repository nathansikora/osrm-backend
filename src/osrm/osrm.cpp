#include "osrm/osrm.hpp"

#include "engine/algorithm.hpp"
#include "engine/api/match_parameters.hpp"
#include "engine/api/nearest_parameters.hpp"
#include "engine/api/route_parameters.hpp"
#include "engine/api/base_parameters.hpp"
#include "engine/api/table_parameters.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/engine.hpp"
#include "engine/engine_config.hpp"
#include "engine/status.hpp"

#include <boost/algorithm/string/join.hpp>

#include <memory>

namespace osrm
{

// Pimpl idiom

OSRM::OSRM(engine::EngineConfig &config)
{
    using CH = engine::routing_algorithms::ch::Algorithm;
    using MLD = engine::routing_algorithms::mld::Algorithm;

    // First, check that necessary core data is available
    if (!config.use_shared_memory && !config.storage_config.IsValid())
    {
        const auto &missingFiles = config.storage_config.GetMissingFiles();
        throw util::exception("Required files are missing, cannot continue. Have all the "
                              "pre-processing steps been run? "
                              "Missing files: " +
                              boost::algorithm::join(missingFiles, ", "));
    }

    // Now, check that the algorithm requested can be used with the data
    // that's available.
    switch (config.algorithm)
    {
    case EngineConfig::Algorithm::CH:
        engine_ = std::make_unique<engine::Engine<CH>>(config);
        break;
    case EngineConfig::Algorithm::MLD:
        engine_ = std::make_unique<engine::Engine<MLD>>(config);
        break;
    default:
        throw util::exception("Algorithm not implemented!");
    }
}
OSRM::~OSRM() = default;
OSRM::OSRM(OSRM &&) noexcept = default;
OSRM &OSRM::operator=(OSRM &&) noexcept = default;

// Forward to implementation

BaseParameters OSRM::NearestPreCalcFix(const BaseParameters &params) const
{
    BaseParameters new_params = params;
    engine::api::ResultT nearest_res;
    json::Object nearest_json_res;
    json::Array waypoints;
    json::Array location;
    int32_t lat;
    int32_t lon;

    try
    {
        NearestParameters nearest_params = NearestParameters();
        BaseParameters dummy_base_params = BaseParameters({util::Coordinate()});
        nearest_params.coordinates = dummy_base_params.coordinates;
        for (size_t i = 0; i < params.coordinates.size(); ++i)
        {
            nearest_params.coordinates[0] = params.coordinates[i];
            nearest_res = engine::api::ResultT();
            engine_->Nearest(nearest_params, nearest_res);
            nearest_json_res = std::get<json::Object>(nearest_res);
            waypoints = get<json::Array>(nearest_json_res.values.at("waypoints"));
            location = get<json::Array>(get<json::Object>(waypoints.values.at(0)).values.at("location"));
            lon = (std::int32_t)(get<json::Number>(location.values.at(0)).value * 1e6);
            lat = (std::int32_t)(get<json::Number>(location.values.at(1)).value * 1e6);
            new_params.coordinates[i].lon = FixedLongitude{lon};
            new_params.coordinates[i].lat = FixedLatitude{lat};
        }
        return new_params;
    }
    catch (...) {
        return params;
    }
}

Status OSRM::Route(const engine::api::RouteParameters &params, json::Object &json_result) const
{
    osrm::engine::api::ResultT result = json::Object();
    auto status = engine_->Route(params, result);
    json_result = std::move(std::get<json::Object>(result));
    return status;
}

Status OSRM::Route(const RouteParameters &params, engine::api::ResultT &result) const
{
    RouteParameters new_params = params;
    new_params.coordinates = NearestPreCalcFix(params).coordinates;
    std::cout << 'ababanana';
    return engine_->Route(new_params, result);
    // return engine_->Route(params, result);
}

Status OSRM::Table(const engine::api::TableParameters &params, json::Object &json_result) const
{
    osrm::engine::api::ResultT result = json::Object();
    auto status = engine_->Table(params, result);
    json_result = std::move(std::get<json::Object>(result));
    return status;
}

Status OSRM::Table(const TableParameters &params, engine::api::ResultT &result) const
{
    TableParameters new_params = params;
    new_params.coordinates = NearestPreCalcFix(params).coordinates;
    std::cout << 'ababanana';
    return engine_->Table(new_params, result);
    // return engine_->Table(params, result);
}

Status OSRM::Nearest(const engine::api::NearestParameters &params, json::Object &json_result) const
{
    osrm::engine::api::ResultT result = json::Object();
    auto status = engine_->Nearest(params, result);
    json_result = std::move(std::get<json::Object>(result));
    return status;
}

Status OSRM::Nearest(const NearestParameters &params, engine::api::ResultT &result) const
{
    return engine_->Nearest(params, result);
}

Status OSRM::Trip(const engine::api::TripParameters &params, json::Object &json_result) const
{
    osrm::engine::api::ResultT result = json::Object();
    auto status = engine_->Trip(params, result);
    json_result = std::move(std::get<json::Object>(result));
    return status;
}

engine::Status OSRM::Trip(const engine::api::TripParameters &params,
                          engine::api::ResultT &result) const
{
    TripParameters new_params = params;
    new_params.coordinates = NearestPreCalcFix(params).coordinates;
    return engine_->Trip(new_params, result);
}

Status OSRM::Match(const engine::api::MatchParameters &params, json::Object &json_result) const
{
    osrm::engine::api::ResultT result = json::Object();
    auto status = engine_->Match(params, result);
    json_result = std::move(std::get<json::Object>(result));
    return status;
}

Status OSRM::Match(const MatchParameters &params, engine::api::ResultT &result) const
{
    return engine_->Match(params, result);
}

Status OSRM::Tile(const engine::api::TileParameters &params, std::string &str_result) const
{
    osrm::engine::api::ResultT result = std::string();
    auto status = engine_->Tile(params, result);
    str_result = std::move(std::get<std::string>(result));
    return status;
}

Status OSRM::Tile(const engine::api::TileParameters &params, engine::api::ResultT &result) const
{
    return engine_->Tile(params, result);
}

} // namespace osrm
