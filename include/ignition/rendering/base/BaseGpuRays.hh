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
#ifndef IGNITION_RENDERING_BASE_BASEGPURAYS_HH_
#define IGNITION_RENDERING_BASE_BASEGPURAYS_HH_

#include <string>

#include <ignition/common/Event.hh>
#include <ignition/common/Console.hh>

#include "ignition/rendering/GpuRays.hh"
#include "ignition/rendering/Image.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/Scene.hh"
#include "ignition/rendering/base/BaseRenderTarget.hh"
#include "ignition/rendering/base/BaseCamera.hh"
#include "ignition/rendering/Visual.hh"
#include "ignition/rendering/RenderTypes.hh"


namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    class IGNITION_RENDERING_VISIBLE BaseGpuRays :
      public virtual GpuRays,
      public virtual BaseCamera<T>,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseGpuRays();

      /// \brief Destructor
      public: virtual ~BaseGpuRays();

      // Documentation inherited.
      public: virtual const float *Data() const override;

      // Documentation inherited.
      public: virtual void CopyData(float *_data) override;

      // Documentation inherited.
      public: virtual void SetClamping(bool _value) override;

      // Documentation inherited.
      public: virtual bool Clamping() const override;

      // Documentation inherited.
      public: virtual common::ConnectionPtr ConnectNewGpuRaysFrame(
                  std::function<void(const float *_frame, unsigned int _width,
                  unsigned int _height, unsigned int _depth,
                  const std::string &_format)> _subscriber) override;

      // Documentation inherited.
      public: virtual RenderTargetPtr RenderTarget() const = 0;

      // Documentation inherited.
      public: virtual void SetIsHorizontal(const bool _horizontal) override;

      // Documentation inherited.
      public: virtual bool IsHorizontal() const override;

      // Documentation inherited.
      public: virtual void SetVFOV(const math::Angle &_vfov) override;

      // Documentation inherited.
      public: virtual math::Angle VFOV() const override;

      // Documentation inherited.
      public: virtual double RayCountRatio() const override;

      // Documentation inherited.
      public: virtual double RangeCountRatio() const override;

      // Documentation inherited.
      public: virtual void SetRayCountRatio(
                  const double _rayCountRatio) override;

      // Documentation inherited.
      public: virtual ignition::math::Angle AngleMin() const override;

      // Documentation inherited.
      public: virtual void SetAngleMin(double _angle) override;

      // Documentation inherited.
      public: virtual ignition::math::Angle AngleMax() const override;

      // Documentation inherited.
      public: virtual void SetAngleMax(double _angle) override;

      // Documentation inherited.
      public: virtual void SetVerticalRayCount(int _samples) override;

      // Documentation inherited.
      public: virtual void SetRayCount(int _samples) override;

      // Documentation inherited.
      public: virtual int RayCount() const override;

      // Documentation inherited.
      public: virtual int RangeCount() const override;

      // Documentation inherited.
      public: virtual int VerticalRayCount() const override;

      // Documentation inherited.
      public: virtual int VerticalRangeCount() const override;

      // Documentation inherited.
      public: virtual ignition::math::Angle VerticalAngleMin() const override;

      // Documentation inherited.
      public: virtual void SetVerticalAngleMin(const double _angle) override;

      // Documentation inherited.
      public: virtual ignition::math::Angle VerticalAngleMax() const override;

      // Documentation inherited.
      public: virtual void SetVerticalAngleMax(const double _angle) override;

      /// \brief maximum value used for data outside sensor range
      public: float dataMaxVal = ignition::math::INF_D;

      /// \brief minimum value used for data outside sensor range
      public: float dataMinVal = -ignition::math::INF_D;

      /// \brief True if clamping values are defined to camera values,
      // false if data outside range is +/- inf
      public: bool clamping = false;

      /// \brief Ray count ratio.
      protected: double rayCountRatio = 0;

      /// \brief Range count ratio.
      protected: double rangeCountRatio = 0;

      /// \brief Vertical field-of-view.
      protected: math::Angle vfov;

      /// \brief True if the sensor is horizontal only.
      protected: bool isHorizontal = true;

      /// \brief Horizontal minimal angle
      protected: double minAngle = 0;

      /// \brief Horizontal maximal angle
      protected: double maxAngle = 0;

      /// \brief Vertical minimal angle
      protected: double vMinAngle = 0;

      /// \brief Vertical maximal angle
      protected: double vMaxAngle = 0;

      /// \brief Quantity of horizontal rays
      protected: int hSamples = 0;

      /// \brief Quantity of verical rays
      protected: int vSamples = 0;

      /// \brief Resolution of horizontal rays
      protected: int hResolution = 1;

      /// \brief Resolution of vertical rays
      protected: int vResolution = 1;

      private: friend class OgreScene;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseGpuRays<T>::BaseGpuRays()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseGpuRays<T>::~BaseGpuRays()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    const float *BaseGpuRays<T>::Data() const
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::CopyData(float * /*_dataDest*/)
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetClamping(bool _value)
    {
      this->clamping = _value;

      if (this->clamping)
      {
        this->dataMinVal = this->NearClipPlane();
        this->dataMaxVal = this->FarClipPlane();
      }
      else
      {
        this->dataMinVal = -ignition::math::INF_D;
        this->dataMaxVal = ignition::math::INF_D;
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseGpuRays<T>::Clamping() const

    {
      return this->clamping;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::common::ConnectionPtr BaseGpuRays<T>::ConnectNewGpuRaysFrame(
          std::function<void(const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>)
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetIsHorizontal(const bool _horizontal)
    {
      this->isHorizontal = _horizontal;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseGpuRays<T>::IsHorizontal() const
    {
      return this->isHorizontal;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGpuRays<T>::RayCountRatio() const
    {
      return this->rayCountRatio;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetRayCountRatio(const double _rayCountRatio)
    {
      this->rayCountRatio = _rayCountRatio;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGpuRays<T>::RangeCountRatio() const
    {
      return this->rangeCountRatio;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Angle BaseGpuRays<T>::VFOV() const
    {
      return this->vfov;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGpuRays<T>::SetVFOV(const math::Angle &_vfov)
    {
      this->vfov = _vfov;
    }

    template <class T>
    //////////////////////////////////////////////////
    ignition::math::Angle BaseGpuRays<T>::AngleMin() const
    {
      return this->minAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetAngleMin(double _angle)
    {
      this->minAngle = _angle;
    }

    template <class T>
    //////////////////////////////////////////////////
    ignition::math::Angle BaseGpuRays<T>::AngleMax() const
    {
      return this->maxAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetAngleMax(double _angle)
    {
      this->maxAngle = _angle;
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::RayCount() const
    {
      return this->hSamples;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetRayCount(int _samples)
    {
      this->hSamples = _samples;
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::RangeCount() const
    {
      return this->RayCount() * this->hResolution;
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::VerticalRayCount() const
    {
      return this->vSamples;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetVerticalRayCount(int _samples)
    {
      this->vSamples = _samples;
    }

    template <class T>
    //////////////////////////////////////////////////
    int BaseGpuRays<T>::VerticalRangeCount() const
    {
      return this->VerticalRayCount() * this->vResolution;
    }

    template <class T>
    //////////////////////////////////////////////////
    ignition::math::Angle BaseGpuRays<T>::VerticalAngleMin() const
    {
      return this->vMinAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetVerticalAngleMin(const double _angle)
    {
        this->vMinAngle = _angle;
    }

    template <class T>
    //////////////////////////////////////////////////
    ignition::math::Angle BaseGpuRays<T>::VerticalAngleMax() const
    {
      return this->vMaxAngle;
    }

    template <class T>
    //////////////////////////////////////////////////
    void BaseGpuRays<T>::SetVerticalAngleMax(const double _angle)
    {
        this->vMaxAngle = _angle;
    }
    }
  }
}
#endif
