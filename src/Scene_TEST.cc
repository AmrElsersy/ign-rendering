/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

using namespace ignition;
using namespace rendering;

class SceneTest : public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  public: void Scene(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void SceneTest::Scene(const std::string &_renderEngine)
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

  EXPECT_EQ(math::Color::Black, scene->BackgroundColor());
  scene->SetBackgroundColor(0, 1, 0, 1);
  EXPECT_EQ(math::Color(0, 1, 0, 1), scene->BackgroundColor());
  math::Color red(1, 0, 0, 1);
  scene->SetBackgroundColor(red);
  EXPECT_EQ(red, scene->BackgroundColor());
}

/////////////////////////////////////////////////
TEST_P(SceneTest, Scene)
{
  Scene(GetParam());
}

INSTANTIATE_TEST_CASE_P(Scene, SceneTest,
    ::testing::Values("ogre", "optix"));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
