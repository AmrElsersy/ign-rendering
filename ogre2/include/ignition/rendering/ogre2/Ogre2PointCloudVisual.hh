/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef IGNITION_RENDERING_OGRE2_OGREPOINTCLOUDVISUAL_HH_
#define IGNITION_RENDERING_OGRE2_OGREPOINTCLOUDVISUAL_HH_

#include <memory>
#include "ignition/rendering/base/BasePointCloudVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2PointCloudVisualPrivate;

    /// \brief Ogre 2.x implementation of a point cloud visual.
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2PointCloudVisual
    : public BasePointCloudVisual<Ogre2Visual>
    {
      /// \brief Constructor
      protected: Ogre2PointCloudVisual();

      /// \brief Destructor
      public: virtual ~Ogre2PointCloudVisual();

      // Documentation inherited.
      public: virtual void Init() override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void Destroy() override;

      // Documentation inherited.
      public: virtual void SetPoints(
                  const std::vector<math::Vector3d> &_points) override;

      // Documentation inherited.
      public: virtual void Update() override;

      // Documentation inherited.
      public: std::vector<math::Vector3d> Points() const;

      /// \brief Marker should only be created by scene.
      private: friend class Ogre2Scene;

      /// \brief Private data pointer.
      private: std::shared_ptr<Ogre2PointCloudVisualPrivate> dataPtr;
    };
    }
  }
}
#endif
