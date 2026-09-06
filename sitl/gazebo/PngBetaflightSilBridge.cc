#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace png::sitl
{
namespace
{
constexpr std::uint16_t kMotorPort = 9002;
constexpr std::uint16_t kStatePort = 9003;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kGravityMps2 = 9.80665;
constexpr double kPi = 3.14159265358979323846;

#pragma pack(push, 1)
struct MotorPacket
{
  float normalized[4];
};

struct FdmPacket
{
  double timestamp;
  double angularVelocityGazeboBodyFlu[3];
  double linearAccelerationBodyFrd[3];
  double orientationBodyFrdToNedWxyz[4];
  double velocityEnu[3];
  double longitudeLatitudeAltitude[3];
  double pressurePa;
};
#pragma pack(pop)

static_assert(sizeof(MotorPacket) == 16, "official Betaflight motor ABI changed");
static_assert(sizeof(FdmPacket) == 144, "official Betaflight FDM ABI changed");

template<typename T>
T SdfValue(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name,
    const T &_default)
{
  if (!_sdf || !_sdf->HasElement(_name))
    return _default;
  return _sdf->Get<T>(_name);
}

sockaddr_in LoopbackAddress(std::uint16_t _port)
{
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(_port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  return address;
}

double PressureAtAltitude(double _altitudeM)
{
  const double base = std::max(0.01, 1.0 - 2.25577e-5 * _altitudeM);
  return 101325.0 * std::pow(base, 5.25588);
}
}

class BetaflightSilBridge final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: ~BetaflightSilBridge() override
  {
    if (this->socketFd >= 0)
      close(this->socketFd);
  }

  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
      throw std::runtime_error("PngBetaflightSilBridge must be attached to a model");
    const auto linkEntity = this->model.CanonicalLink(_ecm);
    if (linkEntity == gz::sim::kNullEntity)
      throw std::runtime_error("PngBetaflightSilBridge requires a canonical link");
    this->link = gz::sim::Link(linkEntity);
    this->link.EnableVelocityChecks(_ecm, true);

    this->maxRotVelocity = SdfValue<double>(_sdf, "maxRotVelocity", 2200.0);
    this->originLatitudeDeg = SdfValue<double>(_sdf, "originLatitudeDeg", 22.7991667);
    this->originLongitudeDeg = SdfValue<double>(_sdf, "originLongitudeDeg", 113.86);
    this->originAltitudeM = SdfValue<double>(_sdf, "originAltitudeM", 20.0);
    this->motorTopic = SdfValue<std::string>(
        _sdf, "motorCommandTopic", "/interceptor/command/motor_speed");
    if (!(this->maxRotVelocity > 0.0) || this->motorTopic.empty())
      throw std::runtime_error("invalid Betaflight SIL bridge configuration");

    this->motorPublisher =
        this->node.Advertise<gz::msgs::Actuators>(this->motorTopic);
    if (!this->motorPublisher)
      throw std::runtime_error("failed to advertise Gazebo motor command topic");

    this->socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->socketFd < 0)
      throw std::runtime_error("failed to create Betaflight SIL UDP socket");
    const int reuse = 1;
    setsockopt(this->socketFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    const auto receiveAddress = LoopbackAddress(kMotorPort);
    if (bind(
            this->socketFd,
            reinterpret_cast<const sockaddr *>(&receiveAddress),
            sizeof(receiveAddress)) != 0)
      throw std::runtime_error("failed to bind Betaflight SIL motor UDP port 127.0.0.1:9002");
    const int flags = fcntl(this->socketFd, F_GETFL, 0);
    fcntl(this->socketFd, F_SETFL, flags | O_NONBLOCK);
    fcntl(this->socketFd, F_SETFD, FD_CLOEXEC);
    this->stateAddress = LoopbackAddress(kStatePort);

    gzmsg << "PngBetaflightSilBridge exact ABI: motor=<4f, FDM=<18d, topic="
          << this->motorTopic << std::endl;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->socketFd < 0)
      return;
    this->ReceiveMotorPacket();
    this->PublishMotorCommand();
    this->SendFdm(_info, _ecm);
  }

  private: void ReceiveMotorPacket()
  {
    while (true)
    {
      MotorPacket packet{};
      sockaddr_in source{};
      socklen_t sourceLength = sizeof(source);
      const auto received = recvfrom(
          this->socketFd,
          &packet,
          sizeof(packet),
          0,
          reinterpret_cast<sockaddr *>(&source),
          &sourceLength);
      if (received < 0)
        break;
      if (received != static_cast<ssize_t>(sizeof(packet)) ||
          ntohl(source.sin_addr.s_addr) != INADDR_LOOPBACK)
        continue;
      for (std::size_t index = 0; index < this->motors.size(); ++index)
      {
        const float value = packet.normalized[index];
        this->motors[index] = std::isfinite(value)
            ? std::clamp(static_cast<double>(value), 0.0, 1.0)
            : 0.0;
      }
      this->receivedMotorPacket = true;
    }
  }

  private: void PublishMotorCommand()
  {
    gz::msgs::Actuators command;
    for (const double normalized : this->motors)
      command.add_velocity(normalized * this->maxRotVelocity);
    this->motorPublisher.Publish(command);
  }

  private: void SendFdm(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
  {
    const auto pose = this->link.WorldPose(_ecm);
    const auto velocity = this->link.WorldLinearVelocity(_ecm);
    const auto angularVelocity = this->link.WorldAngularVelocity(_ecm);
    if (!pose || !velocity || !angularVelocity)
      return;

    const double timestamp =
        std::chrono::duration<double>(_info.simTime).count();
    gz::math::Vector3d worldAcceleration = gz::math::Vector3d::Zero;
    if (this->previousVelocity && this->previousTimestamp)
    {
      const double dt = timestamp - *this->previousTimestamp;
      if (dt > 1.0e-6)
        worldAcceleration = (*velocity - *this->previousVelocity) / dt;
    }
    this->previousVelocity = *velocity;
    this->previousTimestamp = timestamp;

    const auto angularFlu = pose->Rot().RotateVectorReverse(*angularVelocity);
    const auto specificForceFlu = pose->Rot().RotateVectorReverse(
        worldAcceleration - gz::math::Vector3d(0.0, 0.0, -kGravityMps2));
    const gz::math::Vector3d specificForceFrd(
        specificForceFlu.X(), -specificForceFlu.Y(), -specificForceFlu.Z());

    const gz::math::Matrix3d nedFromEnu(
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, -1.0);
    const gz::math::Matrix3d fluFromFrd(
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0);
    // Official 2025.12.2 SITL consumes the body-to-earth quaternion directly;
    // its SIMULATOR_BUILD rotation-matrix branch applies the display-pitch fix.
    gz::math::Quaterniond bodyFrdToNed(
        nedFromEnu * gz::math::Matrix3d(pose->Rot()) * fluFromFrd);
    bodyFrdToNed.Normalize();

    const double latitudeRad = this->originLatitudeDeg * kPi / 180.0;
    const double latitudeDeg = this->originLatitudeDeg +
        pose->Pos().Y() / kEarthRadiusM * 180.0 / kPi;
    const double longitudeDeg = this->originLongitudeDeg +
        pose->Pos().X() /
            (kEarthRadiusM * std::max(0.01, std::cos(latitudeRad))) *
            180.0 / kPi;
    const double altitudeM = this->originAltitudeM + pose->Pos().Z();

    FdmPacket packet{};
    packet.timestamp = timestamp;
    // The flight candidate converts the board gyro back to body FRD with
    // [1,-1,1]. Feed FRD here so the official SITL Y/Z flip reproduces the
    // same virtual sensor convention before that runtime conversion.
    const std::array<double, 3> angular{
        angularFlu.X(), -angularFlu.Y(), -angularFlu.Z()};
    const std::array<double, 3> acceleration{
        specificForceFrd.X(), specificForceFrd.Y(), specificForceFrd.Z()};
    const std::array<double, 4> quaternion{
        bodyFrdToNed.W(), bodyFrdToNed.X(), bodyFrdToNed.Y(), bodyFrdToNed.Z()};
    const std::array<double, 3> velocityEnu{
        velocity->X(), velocity->Y(), velocity->Z()};
    const std::array<double, 3> position{
        longitudeDeg, latitudeDeg, altitudeM};
    std::copy(angular.begin(), angular.end(), packet.angularVelocityGazeboBodyFlu);
    std::copy(acceleration.begin(), acceleration.end(), packet.linearAccelerationBodyFrd);
    std::copy(
        quaternion.begin(), quaternion.end(), packet.orientationBodyFrdToNedWxyz);
    std::copy(velocityEnu.begin(), velocityEnu.end(), packet.velocityEnu);
    std::copy(
        position.begin(), position.end(), packet.longitudeLatitudeAltitude);
    packet.pressurePa = PressureAtAltitude(altitudeM);

    sendto(
        this->socketFd,
        &packet,
        sizeof(packet),
        0,
        reinterpret_cast<const sockaddr *>(&this->stateAddress),
        sizeof(this->stateAddress));
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Link link{gz::sim::kNullEntity};
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher motorPublisher;
  private: int socketFd{-1};
  private: sockaddr_in stateAddress{};
  private: std::string motorTopic;
  private: double maxRotVelocity{2200.0};
  private: double originLatitudeDeg{22.7991667};
  private: double originLongitudeDeg{113.86};
  private: double originAltitudeM{20.0};
  private: std::array<double, 4> motors{0.0, 0.0, 0.0, 0.0};
  private: bool receivedMotorPacket{false};
  private: std::optional<gz::math::Vector3d> previousVelocity;
  private: std::optional<double> previousTimestamp;
};

class DeterministicTargetMotion final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    const auto pose = _ecm.Component<gz::sim::components::Pose>(_entity);
    if (!this->model.Valid(_ecm) || pose == nullptr)
      throw std::runtime_error("DeterministicTargetMotion requires a model pose");
    this->basePose = pose->Data();
    this->eastAmplitudeM = SdfValue<double>(_sdf, "eastAmplitudeM", 1.2);
    this->northAmplitudeM = SdfValue<double>(_sdf, "northAmplitudeM", 0.6);
    this->periodS = SdfValue<double>(_sdf, "periodS", 8.0);
    this->verticalApproachStartS =
        SdfValue<double>(_sdf, "verticalApproachStartS", 8.0);
    this->verticalApproachSpeedMps =
        SdfValue<double>(_sdf, "verticalApproachSpeedMps", 2.0);
    this->maximumVerticalApproachM =
        SdfValue<double>(_sdf, "maximumVerticalApproachM", 3.5);
    this->horizontalApproachDecayS =
        SdfValue<double>(_sdf, "horizontalApproachDecayS", 1.5);
    this->cameraAlignmentStartS =
        SdfValue<double>(_sdf, "cameraAlignmentStartS", 5.8);
    this->cameraAlignmentBlendS =
        SdfValue<double>(_sdf, "cameraAlignmentBlendS", 0.5);
    this->interceptorModelName =
        SdfValue<std::string>(_sdf, "interceptorModelName", "interceptor");
    this->cameraOffsetBodyFluM =
        SdfValue<double>(_sdf, "cameraOffsetBodyFluM", 0.06);
    this->interceptorInitialHeightM =
        SdfValue<double>(_sdf, "interceptorInitialHeightM", 0.16);
    if (this->periodS <= 0.0 || this->verticalApproachStartS < 0.0 ||
        this->verticalApproachSpeedMps < 0.0 ||
        this->maximumVerticalApproachM < 0.0 ||
        this->horizontalApproachDecayS <= 0.0 ||
        this->cameraAlignmentStartS < 0.0 ||
        this->cameraAlignmentBlendS <= 0.0 ||
        this->interceptorModelName.empty() ||
        !std::isfinite(this->interceptorInitialHeightM))
      throw std::runtime_error("invalid deterministic target motion parameters");
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    const double elapsedS = std::chrono::duration<double>(_info.simTime).count();
    const double phase = 2.0 * kPi * elapsedS / this->periodS;
    const double approachElapsedS =
        std::max(0.0, elapsedS - this->verticalApproachStartS);
    const double horizontalScale = std::max(
        0.0, 1.0 - approachElapsedS / this->horizontalApproachDecayS);
    auto pose = this->basePose;
    pose.Pos().X() += horizontalScale * this->eastAmplitudeM * std::sin(phase);
    pose.Pos().Y() += horizontalScale * this->northAmplitudeM * std::sin(2.0 * phase);
    pose.Pos().Z() -= std::min(
        this->maximumVerticalApproachM,
        approachElapsedS * this->verticalApproachSpeedMps);
    const double alignmentBlend = std::clamp(
        (elapsedS - this->cameraAlignmentStartS) /
            this->cameraAlignmentBlendS,
        0.0,
        1.0);
    if (alignmentBlend > 0.0)
      this->AlignWithInterceptorCamera(_ecm, alignmentBlend, pose);
    this->model.SetWorldPoseCmd(_ecm, pose);
  }

  private: void AlignWithInterceptorCamera(
      const gz::sim::EntityComponentManager &_ecm,
      double _blend,
      gz::math::Pose3d &_targetPose)
  {
    if (this->interceptorEntity == gz::sim::kNullEntity)
    {
      this->interceptorEntity = _ecm.EntityByComponents(
          gz::sim::components::Name(this->interceptorModelName),
          gz::sim::components::Model());
    }
    if (this->interceptorEntity == gz::sim::kNullEntity)
      return;

    const auto interceptorPose =
        gz::sim::worldPose(this->interceptorEntity, _ecm);
    const auto opticalAxisWorld = interceptorPose.Rot().RotateVector(
        gz::math::Vector3d::UnitZ);
    if (opticalAxisWorld.Z() <= 0.1)
      return;
    const auto cameraOriginWorld = interceptorPose.Pos() +
        interceptorPose.Rot().RotateVector(
            gz::math::Vector3d(0.0, 0.0, this->cameraOffsetBodyFluM));
    const double blend = std::clamp(_blend, 0.0, 1.0);
    const double targetHeightAboveInitialInterceptor =
        _targetPose.Pos().Z() - this->interceptorInitialHeightM;
    const double alignedTargetHeight =
        interceptorPose.Pos().Z() + targetHeightAboveInitialInterceptor;
    _targetPose.Pos().Z() =
        (1.0 - blend) * _targetPose.Pos().Z() + blend * alignedTargetHeight;
    const double rayDistance = std::max(
        0.1,
        (_targetPose.Pos().Z() - cameraOriginWorld.Z()) / opticalAxisWorld.Z());
    const auto alignedPosition = cameraOriginWorld + rayDistance * opticalAxisWorld;
    _targetPose.Pos().X() =
        (1.0 - blend) * _targetPose.Pos().X() + blend * alignedPosition.X();
    _targetPose.Pos().Y() =
        (1.0 - blend) * _targetPose.Pos().Y() + blend * alignedPosition.Y();
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::math::Pose3d basePose;
  private: double eastAmplitudeM{1.2};
  private: double northAmplitudeM{0.6};
  private: double periodS{8.0};
  private: double verticalApproachStartS{8.0};
  private: double verticalApproachSpeedMps{2.0};
  private: double maximumVerticalApproachM{3.5};
  private: double horizontalApproachDecayS{1.5};
  private: double cameraAlignmentStartS{5.8};
  private: double cameraAlignmentBlendS{0.5};
  private: std::string interceptorModelName{"interceptor"};
  private: double cameraOffsetBodyFluM{0.06};
  private: double interceptorInitialHeightM{0.16};
  private: gz::sim::Entity interceptorEntity{gz::sim::kNullEntity};
};
}

GZ_ADD_PLUGIN(
    png::sitl::BetaflightSilBridge,
    gz::sim::System,
    png::sitl::BetaflightSilBridge::ISystemConfigure,
    png::sitl::BetaflightSilBridge::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    png::sitl::BetaflightSilBridge,
    "png::sitl::BetaflightSilBridge")

GZ_ADD_PLUGIN(
    png::sitl::DeterministicTargetMotion,
    gz::sim::System,
    png::sitl::DeterministicTargetMotion::ISystemConfigure,
    png::sitl::DeterministicTargetMotion::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    png::sitl::DeterministicTargetMotion,
    "png::sitl::DeterministicTargetMotion")
