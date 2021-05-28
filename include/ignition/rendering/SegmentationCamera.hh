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
#ifndef IGNITION_RENDERING_SEGMENTATIONCAMERA_HH_
#define IGNITION_RENDERING_SEGMENTATIONCAMERA_HH_

#include <string>

#include <ignition/common/Event.hh>
#include "ignition/rendering/Camera.hh"
#include "ignition/math/Color.hh"

// Segmentation types for semantic/instance segmentation
enum SegmentationType {
  Semantic,
  Panoptic
};

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \class Camera Camera.hh ignition/rendering/Camera.hh
    /// \brief Poseable Segmentation camera used for rendering the scene graph.
    /// This camera is designed to produced Segmentation data, instead of a 2D
    /// image.
    class IGNITION_RENDERING_VISIBLE SegmentationCamera :
      public virtual Camera,
      public virtual Sensor
    {
      /// \brief Destructor
      public: virtual ~SegmentationCamera() { }

      /// \brief Create a texture which will hold the segmentation data
      public: virtual void CreateSegmentationTexture() = 0;

      /// \brief All things needed to get back z buffer for Segmentation data
      /// \return The labels-buffer as a float array
      public: virtual uint8_t *SegmentationData() const = 0;

      /// \brief Connect to the new Segmentation image signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: virtual ignition::common::ConnectionPtr
        ConnectNewSegmentationFrame(
          std::function<void(const uint8_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) = 0;

      /// \brief Set Segmentation Type (Semantic / Instance)
      /// \param[in] _type Segmentation Type (Semantic / Instance)
      public: virtual void SetSegmentationType(SegmentationType _type) = 0;

      /// \brief Set Segmentation Type (Semantic / Instance)
      /// \param[in] _type Segmentation Type (Semantic / Instance)
      public: virtual void EnableColoredMap(bool _enable) = 0;

      /// \brief Set color for background & unlabeled items in the colored map
      /// \param[in] _color Color of background & unlabeled items
      public: virtual void SetBackgroundColor(math::Color _color) = 0;

      /// \brief Set label for background & unlabeled items in the semantic map
      /// \param[in] _color label of background & unlabeled items
      public: virtual void SetBackgroundLabel(int _label) = 0;

      /// \brief Get color for background & unlabeled items in the colored map
      /// \return Color of background & unlabeled items
      public: virtual math::Color BackgroundColor() = 0;

      /// \brief Set label for background & unlabeled items in the semantic map
      /// \return label of background & unlabeled items
      public: virtual int BackgroundLabel() = 0;
    };
  }
  }
}
#endif
