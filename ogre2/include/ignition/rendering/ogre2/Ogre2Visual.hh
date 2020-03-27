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
#ifndef IGNITION_RENDERING_OGRE2_OGRE2VISUAL_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2VISUAL_HH_

#include <vector>

#include "ignition/rendering/base/BaseVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Node.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Ogre2.x implementation of the visual class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Visual :
      public BaseVisual<Ogre2Node>
    {
      /// \brief Constructor
      protected: Ogre2Visual();

      /// \brief Destructor
      public: virtual ~Ogre2Visual();

      // Documentation inherited.
      public: virtual void SetVisible(bool _visible) override;

      // Documentation inherited.
      public: virtual void SetVisibilityFlags(uint32_t _flags) override;

      // Documentation inherited.
      public: virtual ignition::math::AxisAlignedBox BoundingBox()
                  const override;

      // Documentation inherited.
      public: virtual ignition::math::AxisAlignedBox LocalBoundingBox()
                  const override;

      /// \brief Recursively loop through this visual's children
      /// to obtain the bounding box.
      /// \param[in,out] _box The bounding box.
      /// \param[in] _local A flag indicating if the local bounding box is to
      /// be calculated.
      /// \param[in] _pose The top level visual pose used for local
      /// transformation, the pose will be disregarded if _local is false.
      protected: virtual void BoundsHelper(
                     ignition::math::AxisAlignedBox &_box, bool _local,
                     const ignition::math::Pose3d &_pose) const;

      /// \brief Transform a bounding box to the world space.
      /// \param[in] _bbox Input bounding box.
      /// \param[in] _worldPose Pose used to transform the bounding box.
      /// \param[out] _vertices Vertices of the transformed bounding box in
      /// world coordinates.
      protected: virtual void Transform(
                     const ignition::math::AxisAlignedBox &_bbox,
                     const ignition::math::Pose3d &_worldPose,
                     std::vector<ignition::math::Vector3d> &_vertices) const;

      /// \brief Get the minimum and maximum values of a list of vertices.
      /// \param[in] _vertices A list of input vertices.
      /// \param[out] _min Minimum x, y, z values.
      /// \param[out] _max Maximum x, y, z values.
      protected: virtual void MinMax(
                     const std::vector<ignition::math::Vector3d> &_vertices,
                     ignition::math::Vector3d &_min,
                     ignition::math::Vector3d &_max) const;

      // Documentation inherited.
      protected: virtual GeometryStorePtr Geometries() const override;

      // Documentation inherited.
      protected: virtual bool AttachGeometry(GeometryPtr _geometry) override;

      // Documentation inherited.
      protected: virtual bool DetachGeometry(GeometryPtr _geometry) override;

      /// \brief Initialize the visual
      protected: virtual void Init() override;

      /// \brief Get a shared pointer to this.
      /// \return Shared pointer to this
      private: Ogre2VisualPtr SharedThis();

      /// \brief Pointer to the attached geometries
      protected: Ogre2GeometryStorePtr geometries;

      /// \brief Make scene our friend so it can create ogre2 visuals
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
