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
#ifndef IGNITION_RENDERING_OGRERENDERTYPES_HH_
#define IGNITION_RENDERING_OGRERENDERTYPES_HH_

#include "ignition/rendering/base/BaseRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    class OgreArrowVisual;
    class OgreAxisVisual;
    class OgreCamera;
    class OgreDirectionalLight;
    class OgreGeometry;
    class OgreJointVisual;
    class OgreLight;
    class OgreMaterial;
    class OgreMesh;
    class OgreMeshFactory;
    class OgreNode;
    class OgreObject;
    class OgrePointLight;
    class OgreRenderEngine;
    class OgreScene;
    class OgreSensor;
    class OgreSpotLight;
    class OgreSubMesh;
    class OgreVisual;
    class OgreRenderTarget;
    class OgreRenderTexture;

    typedef BaseSceneStore<OgreScene>       OgreSceneStore;
    typedef BaseNodeStore<OgreNode>         OgreNodeStore;
    typedef BaseLightStore<OgreLight>       OgreLightStore;
    typedef BaseSensorStore<OgreSensor>     OgreSensorStore;
    typedef BaseVisualStore<OgreVisual>     OgreVisualStore;
    typedef BaseGeometryStore<OgreGeometry> OgreGeometryStore;
    typedef BaseSubMeshStore<OgreSubMesh>   OgreSubMeshStore;
    typedef BaseMaterialMap<OgreMaterial>   OgreMaterialMap;

    typedef shared_ptr<OgreArrowVisual>          OgreArrowVisualPtr;
    typedef shared_ptr<OgreAxisVisual>           OgreAxisVisualPtr;
    typedef shared_ptr<OgreCamera>               OgreCameraPtr;
    typedef shared_ptr<OgreDirectionalLight>     OgreDirectionalLightPtr;
    typedef shared_ptr<OgreGeometry>             OgreGeometryPtr;
    typedef shared_ptr<OgreJointVisual>          OgreJointVisualPtr;
    typedef shared_ptr<OgreLight>                OgreLightPtr;
    typedef shared_ptr<OgreMaterial>             OgreMaterialPtr;
    typedef shared_ptr<OgreMesh>                 OgreMeshPtr;
    typedef shared_ptr<OgreMeshFactory>          OgreMeshFactoryPtr;
    typedef shared_ptr<OgreNode>                 OgreNodePtr;
    typedef shared_ptr<OgreObject>               OgreObjectPtr;
    typedef shared_ptr<OgrePointLight>           OgrePointLightPtr;
    typedef shared_ptr<OgreScene>                OgreScenePtr;
    typedef shared_ptr<OgreSensor>               OgreSensorPtr;
    typedef shared_ptr<OgreSpotLight>            OgreSpotLightPtr;
    typedef shared_ptr<OgreSubMesh>              OgreSubMeshPtr;
    typedef shared_ptr<OgreVisual>               OgreVisualPtr;
    typedef shared_ptr<OgreSceneStore>           OgreSceneStorePtr;
    typedef shared_ptr<OgreNodeStore>            OgreNodeStorePtr;
    typedef shared_ptr<OgreLightStore>           OgreLightStorePtr;
    typedef shared_ptr<OgreSensorStore>          OgreSensorStorePtr;
    typedef shared_ptr<OgreVisualStore>          OgreVisualStorePtr;
    typedef shared_ptr<OgreGeometryStore>        OgreGeometryStorePtr;
    typedef shared_ptr<OgreSubMeshStore>         OgreSubMeshStorePtr;
    typedef shared_ptr<OgreMaterialMap>          OgreMaterialMapPtr;
    typedef shared_ptr<OgreRenderTarget>         OgreRenderTargetPtr;
    typedef shared_ptr<OgreRenderTexture>        OgreRenderTexturePtr;
  }
}
#endif
