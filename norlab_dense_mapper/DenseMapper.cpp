#include "DenseMapper.h"
#include <fstream>
#include <chrono>

norlab_dense_mapper::DenseMapper::DenseMapper(const std::string& inputFiltersConfigFilePath,
                                              const std::string& mapPostFiltersConfigFilePath,
                                              std::string mapUpdateCondition,
                                              const float& mapUpdateDelay,
                                              const float& mapUpdateDistance,
                                              const float& minDistNewPoint,
                                              const float& sensorMaxRange,
                                              const float& priorDynamic,
                                              const float& thresholdDynamic,
                                              const float& beamHalfAngle,
                                              const float& epsilonA,
                                              const float& epsilonD,
                                              const float& alpha,
                                              const float& beta,
                                              const bool& is3D,
                                              const bool& isOnline,
                                              const bool& computeProbDynamic,
                                              const bool& isMapping,
                                              const bool& saveMapCellsOnHardDrive) :
    mapUpdateCondition(std::move(mapUpdateCondition)),
    mapUpdateDelay(mapUpdateDelay),
    mapUpdateDistance(mapUpdateDistance),
    is3D(is3D),
    isOnline(isOnline),
    isMapping(isMapping),
    denseMap(minDistNewPoint,
             sensorMaxRange,
             priorDynamic,
             thresholdDynamic,
             beamHalfAngle,
             epsilonA,
             epsilonD,
             alpha,
             beta,
             is3D,
             isOnline,
             computeProbDynamic,
             saveMapCellsOnHardDrive),
    trajectory(is3D ? 3 : 2),
    transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
    loadYamlConfig(inputFiltersConfigFilePath, mapPostFiltersConfigFilePath);

    PM::Parameters radiusFilterParams;
    radiusFilterParams["dim"] = "-1";
    radiusFilterParams["dist"] = std::to_string(sensorMaxRange);
    radiusFilterParams["removeInside"] = "0";
    radiusFilter = PM::get().DataPointsFilterRegistrar.create("DistanceLimitDataPointsFilter",
                                                              radiusFilterParams);
}

void norlab_dense_mapper::DenseMapper::loadYamlConfig(
    const std::string& inputFiltersConfigFilePath,
    const std::string& mapPostFiltersConfigFilePath)
{
    if (!inputFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(inputFiltersConfigFilePath.c_str());
        inputFilters = PM::DataPointsFilters(ifs);
        ifs.close();
    }

    if (!mapPostFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(mapPostFiltersConfigFilePath.c_str());
        mapPostFilters = PM::DataPointsFilters(ifs);
        ifs.close();
    }
}

void norlab_dense_mapper::DenseMapper::processInput(
    const PM::DataPoints& inputInSensorFrame,
    const PM::TransformationParameters& currentPose,
    const std::chrono::time_point<std::chrono::steady_clock>& timeStamp)
{
    PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);
    inputFilters.apply(filteredInputInSensorFrame);
    PM::DataPoints input = transformation->compute(filteredInputInSensorFrame, currentPose);

    if (denseMap.isLocalPointCloudEmpty())
    {
        // denseMap.updatePose(currentPose);
        updateMap(input, currentPose, timeStamp);
    }
    else
    {
        // denseMap.updatePose(currentPose);

        if (shouldUpdateMap(timeStamp, currentPose))
        {
            updateMap(input, currentPose, timeStamp);
        }
    }

    poseLock.lock();
    pose = currentPose;
    poseLock.unlock();

    int euclideanDim = is3D ? 3 : 2;
    trajectoryLock.lock();
    trajectory.addPoint(currentPose.topRightCorner(euclideanDim, 1));
    trajectoryLock.unlock();
}

bool norlab_dense_mapper::DenseMapper::shouldUpdateMap(
    const std::chrono::time_point<std::chrono::steady_clock>& currentTime,
    const PM::TransformationParameters& currentPose) const
{
    if (!isMapping.load())
    {
        return false;
    }

    if (isOnline)
    {
        // if previous update is not over
        if (mapUpdateFuture.valid() &&
            mapUpdateFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        {
            return false;
        }
    }

    if (mapUpdateCondition == "delay")
    {
        return (currentTime - lastTimeMapWasUpdated) > std::chrono::duration<float>(mapUpdateDelay);
    }
    else
    {
        int euclideanDim = is3D ? 3 : 2;
        PM::Vector lastLocation = lastPoseWhereMapWasUpdated.topRightCorner(euclideanDim, 1);
        PM::Vector currentLocation = currentPose.topRightCorner(euclideanDim, 1);
        return std::fabs((currentLocation - lastLocation).norm()) > mapUpdateDistance;
    }
}

void norlab_dense_mapper::DenseMapper::updateMap(
    const PM::DataPoints& currentInput,
    const PM::TransformationParameters& currentPose,
    const std::chrono::time_point<std::chrono::steady_clock>& currentTimeStamp)
{
    lastTimeMapWasUpdated = currentTimeStamp;
    lastPoseWhereMapWasUpdated = currentPose;

    if (isOnline && !denseMap.isLocalPointCloudEmpty())
    {
        mapUpdateFuture = std::async(std::launch::async,
                                     &DenseMap::updateLocalPointCloud,
                                     &denseMap,
                                     currentInput,
                                     currentPose,
                                     mapPostFilters);
    }
    else
    {
        denseMap.updateLocalPointCloud(currentInput, currentPose, mapPostFilters);
    }
}

norlab_dense_mapper::DenseMapper::PM::DataPoints norlab_dense_mapper::DenseMapper::getMap()
{
    return denseMap.getGlobalPointCloud();
}

void norlab_dense_mapper::DenseMapper::setMap(const PM::DataPoints& newMap)
{
    denseMap.setGlobalPointCloud(newMap);
    trajectoryLock.lock();
    trajectory.clearPoints();
    trajectoryLock.unlock();
}

bool norlab_dense_mapper::DenseMapper::getNewLocalMap(PM::DataPoints& mapOut)
{
    return denseMap.getNewLocalPointCloud(mapOut);
}

norlab_dense_mapper::DenseMapper::PM::TransformationParameters
norlab_dense_mapper::DenseMapper::getPose()
{
    std::lock_guard<std::mutex> lock(poseLock);
    return pose;
}

bool norlab_dense_mapper::DenseMapper::getIsMapping() const
{
    return isMapping.load();
}

void norlab_dense_mapper::DenseMapper::setIsMapping(const bool& newIsMapping)
{
    isMapping.store(newIsMapping);
}

Trajectory norlab_dense_mapper::DenseMapper::getTrajectory()
{
    std::lock_guard<std::mutex> lock(trajectoryLock);
    return trajectory;
}
