/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _IGNITION_RENDERING_RENDERTYPES_HH_
#define _IGNITION_RENDERING_RENDERTYPES_HH_

#include <memory>

namespace ignition
{
  namespace rendering
  {
    template <class T>
    using shared_ptr = std::shared_ptr<T>;

    class ArrowVisual;
    class AxisVisual;
    class Camera;
    class DirectionalLight;
    class Geometry;
    class JointVisual;
    class Image;
    class Light;
    class Material;
    class Mesh;
    class Node;
    class Object;
    class ObjectFactory;
    class PointLight;
    class RenderEngine;
    class Scene;
    class Sensor;
    class SpotLight;
    class SubMesh;
    class Visual;
    class RenderTarget;
    class RenderTexture;


    /// \def ArrowVisualPtr
    /// \brief Shared pointer to ArrowVisual
    typedef shared_ptr<ArrowVisual> ArrowVisualPtr;

    /// \def AxisVisualPtr
    /// \brief Shared pointer to AxisVisual
    typedef shared_ptr<AxisVisual> AxisVisualPtr;

    /// \def CameraPtr
    /// \brief Shared pointer to Camera
    typedef shared_ptr<Camera> CameraPtr;

    /// \def DirectionalLightPtr
    /// \brief Shared pointer to DirectionalLight
    typedef shared_ptr<DirectionalLight> DirectionalLightPtr;

    /// \def GeometryPtr
    /// \brief Shared pointer to Geometry
    typedef shared_ptr<Geometry> GeometryPtr;

    /// \def JointVisualPtr
    /// \brief Shared pointer to JointVisual
    typedef shared_ptr<JointVisual> JointVisualPtr;

    /// \def ImagePtr
    /// \brief Shared pointer to Image
    typedef shared_ptr<Image> ImagePtr;

    /// \def LightPtr
    /// \brief Shared pointer to Light
    typedef shared_ptr<Light> LightPtr;

    /// \def MaterialPtr
    /// \brief Shared pointer to Material
    typedef shared_ptr<Material> MaterialPtr;

    /// \def MeshPtr
    /// \brief Shared pointer to Mesh
    typedef shared_ptr<Mesh> MeshPtr;

    /// \def NodePtr
    /// \brief Shared pointer to Node
    typedef shared_ptr<Node> NodePtr;

    /// \def ObjectPtr
    /// \brief Shared pointer to Object
    typedef shared_ptr<Object> ObjectPtr;

    /// \def ObjectFactoryPtr
    /// \brief Shared pointer to ObjectFactory
    typedef shared_ptr<ObjectFactory> ObjectFactoryPtr;

    /// \def PointLightPtr
    /// \brief Shared pointer to PointLight
    typedef shared_ptr<PointLight> PointLightPtr;

    /// \def ScenePtr
    /// \brief Shared pointer to Scene
    typedef shared_ptr<Scene> ScenePtr;

    /// \def SensorPtr
    /// \brief Shared pointer to Sensor
    typedef shared_ptr<Sensor> SensorPtr;

    /// \def SpotLightPtr
    /// \brief Shared pointer to SpotLight
    typedef shared_ptr<SpotLight> SpotLightPtr;

    /// \def SubMeshPtr
    /// \brief Shared pointer to SubMesh
    typedef shared_ptr<SubMesh> SubMeshPtr;

    /// \def VisualPtr
    /// \brief Shared pointer to Visual
    typedef shared_ptr<Visual> VisualPtr;

    /// \def RenderTargetPtr
    /// \brief Shared pointer to RenderTarget
    typedef shared_ptr<RenderTarget> RenderTargetPtr;

    /// \def RenderTexturePtr
    /// \brief Shared pointer to RenderTexture
    typedef shared_ptr<RenderTexture> RenderTexturePtr;

    /// \def const ArrowVisualPtr
    /// \brief Shared pointer to const ArrowVisual
    typedef shared_ptr<const ArrowVisual> ConstArrowVisualPtr;

    /// \def const AxisVisualPtr
    /// \brief Shared pointer to const AxisVisual
    typedef shared_ptr<const AxisVisual> ConstAxisVisualPtr;

    /// \def const CameraPtr
    /// \brief Shared pointer to const Camera
    typedef shared_ptr<const Camera> ConstCameraPtr;

    /// \def const DirectionalLightPtr
    /// \brief Shared pointer to const DirectionalLight
    typedef shared_ptr<const DirectionalLight> ConstDirectionalLightPtr;

    /// \def const GeometryPtr
    /// \brief Shared pointer to const Geometry
    typedef shared_ptr<const Geometry> ConstGeometryPtr;

    /// \def const JointVisualPtr
    /// \brief Shared pointer to const JointVisual
    typedef shared_ptr<const JointVisual> ConstJointVisualPtr;

    /// \def const ImagePtr
    /// \brief Shared pointer to const Image
    typedef shared_ptr<const Image> ConstImagePtr;

    /// \def const LightPtr
    /// \brief Shared pointer to const Light
    typedef shared_ptr<const Light> ConstLightPtr;

    /// \def const MaterialPtr
    /// \brief Shared pointer to const Material
    typedef shared_ptr<const Material> ConstMaterialPtr;

    /// \def const MeshPtr
    /// \brief Shared pointer to const Mesh
    typedef shared_ptr<const Mesh> ConstMeshPtr;

    /// \def const NodePtr
    /// \brief Shared pointer to const Node
    typedef shared_ptr<const Node> ConstNodePtr;

    /// \def const ObjectPtr
    /// \brief Shared pointer to const Object
    typedef shared_ptr<const Object> ConstObjectPtr;

    /// \def const ObjectFactoryPtr
    /// \brief Shared pointer to const ObjectFactory
    typedef shared_ptr<const ObjectFactory> ConstObjectFactoryPtr;

    /// \def const PointLightPtr
    /// \brief Shared pointer to const PointLight
    typedef shared_ptr<const PointLight> ConstPointLightPtr;

    /// \def const ScenePtr
    /// \brief Shared pointer to const Scene
    typedef shared_ptr<const Scene> ConstScenePtr;

    /// \def const SensorPtr
    /// \brief Shared pointer to const Sensor
    typedef shared_ptr<const Sensor> ConstSensorPtr;

    /// \def const SpotLightPtr
    /// \brief Shared pointer to const SpotLight
    typedef shared_ptr<const SpotLight> ConstSpotLightPtr;

    /// \def const SubMeshPtr
    /// \brief Shared pointer to const SubMesh
    typedef shared_ptr<const SubMesh> ConstSubMeshPtr;

    /// \def const VisualPtr
    /// \brief Shared pointer to const Visual
    typedef shared_ptr<const Visual> ConstVisualPtr;

    /// \def const RenderTargetPtr
    /// \brief Shared pointer to const RenderTarget
    typedef shared_ptr<const RenderTarget> ConstRenderTargetPtr;

    /// \def const RenderTexturePtr
    /// \brief Shared pointer to const RenderTexture
    typedef shared_ptr<const RenderTexture> ConstRenderTexturePtr;
  }
}
#endif
