/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Filesystem.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/GpuRays.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

// vertical range values seem to be less accurate
#define VERTICAL_LASER_TOL 1e-3

#define WAIT_TIME 0.02

using namespace ignition;
using namespace rendering;

void OnNewGpuRaysFrame(float *_scanDest, const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _channels,
                  PixelFormat /*_format*/)
{
  float f;
  int size =  _width * _height * _channels;
  memcpy(_scanDest, _scan, size * sizeof(f));
}

class GpuRaysTest: public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Test and verify gpu rays properties setters and getters
  public: void Configure(const std::string &_renderEngine);

  // Test boxes detection
  public: void RaysUnitBox(const std::string &_renderEngine);

  // Test vertical measurements
  public: void LaserVertical(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
/// \brief Test GPU rays configuraions
void GpuRaysTest::Configure(const std::string &_renderEngine)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  GpuRaysPtr gpuRays = scene->CreateGpuRays();
  ASSERT_TRUE(gpuRays != nullptr);
  root->AddChild(gpuRays);

  // set gpu rays caster initial pose
  math::Vector3d initPos(-2, 0.0, 5.0);
  math::Quaterniond initRot = math::Quaterniond::Identity;
  gpuRays->SetWorldPosition(initPos);
  EXPECT_EQ(initPos, gpuRays->WorldPosition());
  EXPECT_EQ(initRot, gpuRays->WorldRotation());

  // The following tests all the getters and setters
  {
    gpuRays->SetNearClipPlane(0.1);
    EXPECT_NEAR(gpuRays->NearClipPlane(), 0.1, 1e-6);

    gpuRays->SetFarClipPlane(100.0);
    EXPECT_NEAR(gpuRays->FarClipPlane(), 100, 1e-6);

    gpuRays->SetIsHorizontal(false);
    EXPECT_FALSE(gpuRays->IsHorizontal());

    gpuRays->SetNearClipPlane(0.04);
    EXPECT_NEAR(gpuRays->NearClipPlane(), 0.04, 1e-6);

    gpuRays->SetFarClipPlane(5.4);
    EXPECT_NEAR(gpuRays->FarClipPlane(), 05.4, 1e-6);

    gpuRays->SetAngleMin(-1.47);
    EXPECT_NEAR(gpuRays->AngleMin().Radian(), -1.47, 1e-6);

    gpuRays->SetAngleMax(1.56);
    EXPECT_NEAR(gpuRays->AngleMax().Radian(), 1.56, 1e-6);

    gpuRays->SetVerticalAngleMin(-0.32);
    EXPECT_NEAR(gpuRays->VerticalAngleMin().Radian(), -0.32, 1e-6);

    gpuRays->SetVerticalAngleMax(1.58);
    EXPECT_NEAR(gpuRays->VerticalAngleMax().Radian(), 1.58, 1e-6);

    gpuRays->SetRayCount(100);
    EXPECT_NEAR(gpuRays->RayCount(), 100, 1e-6);

    gpuRays->SetVerticalRayCount(67);
    EXPECT_NEAR(gpuRays->VerticalRayCount(), 67, 1e-6);
  }

  // Clean up
  engine->DestroyScene(scene);
}


/////////////////////////////////////////////////
/// \brief Test detection of different boxes
void GpuRaysTest::RaysUnitBox(const std::string &_renderEngine)
{
  // Test GPU rays with 3 boxes in the world.
  // First GPU rays at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values

  const double hMinAngle = -M_PI/2.0;
  const double hMaxAngle = M_PI/2.0;
  const double minRange = 0.1;
  const double maxRange = 10.0;
  const int hRayCount = 320;
  const int vRayCount = 1;

  common::Time waitTime = common::Time(WAIT_TIME);

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  // Create first ray caster
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);

  GpuRaysPtr gpuRays = scene->CreateGpuRays("gpu_rays_1");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetRayCount(hRayCount);
  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  // Create a second ray caster rotated
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond(M_PI/2.0, 0, 0));

  GpuRaysPtr gpuRays2 = scene->CreateGpuRays("gpu_rays_2");
  gpuRays2->SetWorldPosition(testPose2.Pos());
  gpuRays2->SetWorldRotation(testPose2.Rot());
  gpuRays2->SetNearClipPlane(minRange);
  gpuRays2->SetFarClipPlane(maxRange);
  gpuRays2->SetAngleMin(hMinAngle);
  gpuRays2->SetAngleMax(hMaxAngle);
  gpuRays2->SetRayCount(hRayCount);
  gpuRays2->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays2);

  // Create testing boxes
  // box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(3, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = scene->CreateVisual("UnitBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // box on the right of the first gpu rays caster
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -5, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox2 = scene->CreateVisual("UnitBox2");
  visualBox2->AddGeometry(scene->CreateBox());
  visualBox2->SetWorldPosition(box02Pose.Pos());
  visualBox2->SetWorldRotation(box02Pose.Rot());
  root->AddChild(visualBox2);

  // box on the left of the rays caster 1 but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, maxRange + 1, 0.5),
      ignition::math::Quaterniond::Identity);
  VisualPtr visualBox3 = scene->CreateVisual("UnitBox3");
  visualBox3->AddGeometry(scene->CreateBox());
  visualBox3->SetWorldPosition(box03Pose.Pos());
  visualBox3->SetWorldRotation(box03Pose.Rot());
  root->AddChild(visualBox3);

  // Verify rays caster 1 range readings
  // listen to new gpu rays frames
  unsigned int channels = 3;
  float *scan = new float[hRayCount * vRayCount * channels];
  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  gpuRays->Update();

  int mid = hRayCount * channels / 2;
  int last = (hRayCount - 1) * channels;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 = abs(box01Pose.Pos().X()) - unitBoxSize/2;
  double expectedRangeAtMidPointBox2 = abs(box02Pose.Pos().Y()) - unitBoxSize/2;

  // rays caster 1 should see box01 and box02
  EXPECT_NEAR(scan[mid], expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_NEAR(scan[0], expectedRangeAtMidPointBox2, LASER_TOL);
  EXPECT_DOUBLE_EQ(scan[last], ignition::math::INF_D);

  // Verify rays caster 2 range readings
  // listen to new gpu rays frames
  float *scan2 = new float[hRayCount * vRayCount * 3];
  common::ConnectionPtr c2 =
    gpuRays2->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  gpuRays2->Update();

  // Only box01 should be visible to rays caster 2
  EXPECT_DOUBLE_EQ(scan2[0], ignition::math::INF_D);
  EXPECT_NEAR(scan2[mid], expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(scan2[last], ignition::math::INF_D);

  // Move all boxes out of range
  visualBox1->SetWorldPosition(
      ignition::math::Vector3d(maxRange + 1, 0, 0));
  visualBox1->SetWorldRotation(box01Pose.Rot());
  visualBox2->SetWorldPosition(
      ignition::math::Vector3d(0, -(maxRange + 1), 0));
  visualBox2->SetWorldRotation(box02Pose.Rot());

  gpuRays->Update();
  gpuRays2->Update();

  for (unsigned int i = 0; i < gpuRays->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(scan[i * 3], ignition::math::INF_D);

  for (unsigned int i = 0; i < gpuRays2->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(scan2[i * 3], ignition::math::INF_D);

  c.reset();
  c2.reset();

  delete [] scan;
  delete [] scan2;

  scan = nullptr;
  scan2 = nullptr;

  // Clean up
  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
/// \brief Test GPU rays vertical component
void GpuRaysTest::LaserVertical(const std::string &_renderEngine)
{
  // Test a rays that has a vertical range component.
  // Place a box within range and verify range values,
  // then move the box out of range and verify range values

  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double vMinAngle = -M_PI/4.0;
  double vMaxAngle = M_PI/4.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  unsigned int hRayCount = 640;
  unsigned int vRayCount = 4;

  common::Time waitTime = common::Time(WAIT_TIME);

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  // Create first ray caster
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0, 0.5),
      ignition::math::Quaterniond::Identity);

  GpuRaysPtr gpuRays = scene->CreateGpuRays("vertical_gpu_rays");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetVerticalAngleMin(vMinAngle);
  gpuRays->SetVerticalAngleMax(vMaxAngle);
  gpuRays->SetRayCount(hRayCount);
  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  // Create testing boxes
  // box in front of ray sensor
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = scene->CreateVisual("VerticalTestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  unsigned int channels = 3;
  float *scan = new float[hRayCount * vRayCount * channels];
  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  gpuRays->Update();

  unsigned int mid = hRayCount * channels / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2
      - testPose.Pos().X();

  double vAngleStep = (vMaxAngle - vMinAngle) / (vRayCount-1);
  double angleStep = vMinAngle;

  // all vertical laser planes should sense box
  for (unsigned int i = 0; i < vRayCount; ++i)
  {
    double expectedRange = expectedRangeAtMidPoint / cos(angleStep);

    EXPECT_NEAR(scan[i * hRayCount * channels + mid],
        expectedRange, VERTICAL_LASER_TOL);

    angleStep += vAngleStep;

    // check that the values in the extremes are infinity
    EXPECT_DOUBLE_EQ(scan[i * hRayCount * channels],
        ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(scan[(i * hRayCount + (hRayCount - 1)) * channels],
        ignition::math::INF_D);
  }

  // Move box out of range
  visualBox1->SetWorldPosition(
      ignition::math::Vector3d(maxRange + 1, 0, 0));
  visualBox1->SetWorldRotation(
      ignition::math::Quaterniond::Identity);

  // wait for a few more laser scans
  gpuRays->Update();

  for (unsigned int j = 0; j < gpuRays->VerticalRayCount(); ++j)
  {
    for (unsigned int i = 0; i < gpuRays->RayCount(); ++i)
    {
      EXPECT_DOUBLE_EQ(scan[j * gpuRays->RayCount() * channels+ i * channels],
          ignition::math::INF_D);
    }
  }

  c.reset();

  delete [] scan;
  scan = nullptr;

  // Clean up
  engine->DestroyScene(scene);
}

/////////////////////////////////////////////////
TEST_P(GpuRaysTest, Configure)
{
  Configure(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GpuRaysTest, RaysUnitBox)
{
  RaysUnitBox(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GpuRaysTest, LaserVertical)
{
  LaserVertical(GetParam());
}

INSTANTIATE_TEST_CASE_P(GpuRays, GpuRaysTest,
    ::testing::Values("ogre"),
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
