#ifndef DENSE_MAPPER_H
#define DENSE_MAPPER_H

#include <pointmatcher/PointMatcher.h>
#include "DenseMap.h"
#include <future>
#include <mutex>

namespace norlab_dense_mapper
{
class DenseMapper
{
  private:
    typedef PointMatcher<float> PM;

    PM::DataPointsFilters depthCameraFilters;
    PM::DataPointsFilters sensorFilters;
    PM::DataPointsFilters robotFilters;
    PM::DataPointsFilters robotStabilizedFilters;
    PM::DataPointsFilters mapPostFilters;
    std::string mapUpdateCondition;
    std::string depthCameraFrame;
    float mapUpdateDelay;
    float mapUpdateDistance;
    bool is3D;
    bool isDeptCameraEnabled;
    bool isOnline;
    std::atomic_bool isMapping;
    DenseMap denseMap;
    PM::TransformationParameters pose;
    std::shared_ptr<PM::Transformation> transformation;
    std::shared_ptr<PM::DataPointsFilter> radiusFilter;
    std::chrono::time_point<std::chrono::steady_clock> lastTimeMapWasUpdated;
    PM::TransformationParameters lastPoseWhereMapWasUpdated;
    std::future<void> mapUpdateFuture;
    std::mutex poseLock;

    bool shouldUpdateMap(const std::chrono::time_point<std::chrono::steady_clock>& currentTime,
                         const PM::TransformationParameters& currentPose) const;
    void updateMap(const PM::DataPoints& currentInput,
                   const PM::TransformationParameters& currentPose,
                   const std::chrono::time_point<std::chrono::steady_clock>& currentTimeStamp);

  public:
    DenseMapper(const std::string& depthCameraFiltersConfigFilePath,
                const std::string& sensorFiltersConfigFilePath,
                const std::string& robotFiltersConfigFilePath,
                const std::string& robotStabilizedFiltersConfigFilePath,
                const std::string& mapPostFiltersConfigFilePath,
                std::string mapUpdateCondition,
                std::string depthCameraFrame,
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
                const bool& isDepthCameraEnabled,
                const bool& isOnline,
                const bool& isMapping,
                const bool& computeProbDynamic,
                const bool& saveMapCellsOnHardDrive);
    void loadYamlConfig(const std::string& depthCameraFiltersConfigFilePath,
                        const std::string& sensorFiltersConfigFilePath,
                        const std::string& robotFiltersConfigFilePath,
                        const std::string& robotInputFiltersConfigFilePath,
                        const std::string& mapPostFiltersConfigFilePath);
    void processInput(const std::string& sensorFrameId,
                      const PM::DataPoints& inputInSensorFrame,
                      const PM::TransformationParameters& sensorToRobot,
                      const PM::TransformationParameters& robotToRobotStabilized,
                      const PM::TransformationParameters& robotStabilizedToMap,
                      const std::chrono::time_point<std::chrono::steady_clock>& timeStamp);
    PM::DataPoints getMap();
    void setMap(const PM::DataPoints& newMap);
    bool getNewLocalMap(PM::DataPoints& mapOut);
    bool getIsMapping() const;
    void setIsMapping(const bool& newIsMapping);
    PM::TransformationParameters getPose();
};
}

#endif
