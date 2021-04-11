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
#ifndef IGNITION_RENDERING_BASE_BASESegmentationCAMERA_HH_
#define IGNITION_RENDERING_BASE_BASESegmentationCAMERA_HH_

#include <string>

#include <ignition/common/Event.hh>

#include "ignition/rendering/base/BaseCamera.hh"
#include "ignition/rendering/SegmentationCamera.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
      
    template <class T>
    class BaseSegmentationCamera :
      public virtual SegmentationCamera,
      public virtual BaseCamera<T>,
      public virtual T
    {
      protected: BaseSegmentationCamera();

      public: virtual ~BaseSegmentationCamera();

      public: virtual void CreateSegmentationTexture();

      public: virtual uint8_t *SegmentationData() const;

      public: virtual ignition::common::ConnectionPtr ConnectNewSegmentationFrame(
          std::function<void(const uint8_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber);
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseSegmentationCamera<T>::BaseSegmentationCamera()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseSegmentationCamera<T>::~BaseSegmentationCamera()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSegmentationCamera<T>::CreateSegmentationTexture()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    uint8_t *BaseSegmentationCamera<T>::SegmentationData() const
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::common::ConnectionPtr BaseSegmentationCamera<T>::ConnectNewSegmentationFrame(
          std::function<void(const uint8_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>)
    {
      return nullptr;
    }
  }
  }
}
#endif
