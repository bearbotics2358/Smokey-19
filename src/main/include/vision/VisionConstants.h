#pragma once

#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "vision/IVisionInput.h"
#include "vision/VisionInputPhotonVision.h"
#include "vision/VisionInputPhotonVisionSim.h"

class VisionConstants {
public:
  static inline const frc::AprilTagFieldLayout kAprilTagFieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);

  static constexpr double kMaxAmbiguity = 0.3;
  static constexpr units::meter_t kMaxZError = 0.75_m;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  static constexpr units::meter_t kLinearStdDevBaseline = 0.02_m;
  static constexpr units::radian_t kAngularStdDevBaseline = 0.06_rad;

public:
  static std::vector<std::unique_ptr<IVisionInput>> GetLocalizationCameras(subsystems::CommandSwerveDrivetrain* drivetrain) {
    if (frc::RobotBase::IsSimulation()) {
      return VisionConstants::CreateSimLocalizationCameras(drivetrain);
    } else {
      return VisionConstants::CreateLocalizationCameras();
    }
  }

private:
  /**
   * Generate a list of cameras used for localization
   */
  static std::vector<std::unique_ptr<IVisionInput>> CreateLocalizationCameras() {
    std::vector<std::unique_ptr<IVisionInput>> localization_cameras;

    // @todo Add the real cameras here!

    return localization_cameras;
  }

  /**
   * Generate a list of simulated cameras for running on the simluator
   */
  static std::vector<std::unique_ptr<IVisionInput>> CreateSimLocalizationCameras(subsystems::CommandSwerveDrivetrain* drivetrain) {
    const frc::Transform3d kSimRobotToCamera0{0.2_m, 0.0_m, 0.2_m, frc::Rotation3d{0.0_rad, -0.4_rad, 0.0_rad}};
    const std::string kCameraName0{"localization0"};

    std::vector<std::unique_ptr<IVisionInput>> localization_cameras;

    localization_cameras.push_back(std::make_unique<VisionInputPhotonVisionSim>(kCameraName0, kSimRobotToCamera0, [drivetrain] {
      return drivetrain->GetState().Pose;
    }));

    return localization_cameras;
  }
};