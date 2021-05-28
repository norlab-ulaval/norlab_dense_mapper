#include "DenseMapper.h"
#include <fstream>
#include <chrono>

norlab_dense_mapper::DenseMapper::DenseMapper(
    const std::string& sensorFiltersConfigFilePath,
    const std::string& robotFiltersConfigFilePath,
    const std::string& robotStabilizedFiltersConfigFilePath,
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
    transformation(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
    loadYamlConfig(sensorFiltersConfigFilePath,
                   robotFiltersConfigFilePath,
                   robotStabilizedFiltersConfigFilePath,
                   mapPostFiltersConfigFilePath);

    PM::Parameters radiusFilterParams{{"dim", "-1"},
                                      {"dist", std::to_string(sensorMaxRange)},
                                      {"removeInside", "0"}};
    radiusFilter = PM::get().DataPointsFilterRegistrar.create("DistanceLimitDataPointsFilter",
                                                              radiusFilterParams);
}

void norlab_dense_mapper::DenseMapper::loadYamlConfig(
    const std::string& sensorFiltersConfigFilePath,
    const std::string& robotFiltersConfigFilePath,
    const std::string& robotStabilizedFiltersConfigFilePath,
    const std::string& mapPostFiltersConfigFilePath)
{
    if (!sensorFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(sensorFiltersConfigFilePath.c_str());
        sensorFilters = PM::DataPointsFilters(ifs);
        ifs.close();
    }

    if (!robotFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(robotFiltersConfigFilePath.c_str());
        robotFilters = PM::DataPointsFilters(ifs);
        ifs.close();
    }

    if (!robotStabilizedFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(robotStabilizedFiltersConfigFilePath.c_str());
        robotStabilizedFilters = PM::DataPointsFilters(ifs);
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
    const PM::TransformationParameters& sensorToRobot,
    const PM::TransformationParameters& robotToRobotStabilized,
    const PM::TransformationParameters& robotStabilizedToMap,
    const std::chrono::time_point<std::chrono::steady_clock>& timeStamp)
{
    PM::DataPoints filteredInputInSensorFrame = radiusFilter->filter(inputInSensorFrame);

    // Apply the sensor filters to the point cloud in the sensor frame (lidar)
    sensorFilters.apply(filteredInputInSensorFrame);
    // Compute the transformation between the sensor frame (lidar) and the robot frame (base_link)
    PM::DataPoints inputInRobotFrame =
        transformation->compute(filteredInputInSensorFrame, sensorToRobot);

    // Apply the robot filters to the point cloud in the robot frame (base_link)
    robotFilters.apply(inputInRobotFrame);
    // Compute the transformation between the robot frame (base_link) and the stabilized robot frame
    // (base_link_stabilized)
    PM::DataPoints inputInRobotStabilizedFrame =
        transformation->compute(inputInRobotFrame, robotToRobotStabilized);

    // Apply the robot stabilized filters to the point cloud in the robot stabilized frame
    robotStabilizedFilters.apply(inputInRobotStabilizedFrame);
    // Compute the transformation between the stabilized robot frame (base_link_stabilized) and the
    // map frame
    PM::DataPoints inputInMapFrame =
        transformation->compute(inputInRobotStabilizedFrame, robotStabilizedToMap);

    PM::TransformationParameters sensorToMap =
        robotStabilizedToMap * robotToRobotStabilized * sensorToRobot;

    if (denseMap.isLocalPointCloudEmpty())
    {
        // denseMap.updatePose(sensorToRobot);
        updateMap(inputInMapFrame, sensorToMap, timeStamp);
    }
    else
    {
        // denseMap.updatePose(sensorToRobot);

        if (shouldUpdateMap(timeStamp, sensorToMap))
        {
            updateMap(inputInMapFrame, sensorToMap, timeStamp);
        }
    }
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
}

bool norlab_dense_mapper::DenseMapper::getNewLocalMap(PM::DataPoints& mapOut)
{
    return denseMap.getNewLocalPointCloud(mapOut);
}

bool norlab_dense_mapper::DenseMapper::getIsMapping() const
{
    return isMapping.load();
}
void norlab_dense_mapper::DenseMapper::setIsMapping(const bool& newIsMapping)
{
    isMapping.store(newIsMapping);
}
