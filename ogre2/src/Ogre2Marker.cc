/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>

#include "ignition/rendering/ogre2/Ogre2Marker.hh"
#include "ignition/rendering/ogre2/Ogre2Material.hh"
#include "ignition/rendering/ogre2/Ogre2Mesh.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"

class ignition::rendering::Ogre2MarkerPrivate
{
  /// \brief Marker material
  public: Ogre2MaterialPtr material = nullptr;

  /// \brief Mesh Object for primitive shapes
  public: Ogre2MeshPtr mesh = nullptr;

  /// \brief MarkerType of render operation
  public: MarkerType markerType;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Marker::Ogre2Marker()
  : dataPtr(new Ogre2MarkerPrivate)
{
}

//////////////////////////////////////////////////
Ogre2Marker::~Ogre2Marker()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2Marker::PreRender()
{
}

//////////////////////////////////////////////////
void Ogre2Marker::Destroy()
{
  if (this->dataPtr->mesh)
  {
    this->dataPtr->mesh->Destroy();
    this->dataPtr->mesh.reset();
  }

  if (this->dataPtr->material && this->Scene())
  {
    this->Scene()->DestroyMaterial(this->dataPtr->material);
    this->dataPtr->material.reset();
  }
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2Marker::OgreObject() const
{
  switch (markerType)
  {
    case NONE:
    case BOX:
    case CYLINDER:
    case SPHERE:
      return this->dataPtr->mesh->OgreObject();
    case LINE_STRIP:
    case LINE_LIST:
    case POINTS:
    case TRIANGLE_FAN:
    case TRIANGLE_LIST:
    case TRIANGLE_STRIP:
    {
      ignerr << "Failed to create marker of type " << markerType
             << ". Dynamic renderables not supported yet\n";
      return this->dataPtr->mesh->OgreObject();
    }
    default:
      ignerr << "Invalid Marker type " << markerType << "\n";
      return nullptr;
  }
}

//////////////////////////////////////////////////
void Ogre2Marker::Init()
{
  this->Create();
}

//////////////////////////////////////////////////
void Ogre2Marker::Create()
{
  this->dataPtr->markerType = BOX;
  if (!this->dataPtr->mesh)
  {
    this->dataPtr->mesh =
      std::dynamic_pointer_cast<Ogre2Mesh>(this->scene->CreateBox());
  }
}

//////////////////////////////////////////////////
void Ogre2Marker::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  Ogre2MaterialPtr derived =
    std::dynamic_pointer_cast<Ogre2Material>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign material created by another render-engine"
      << std::endl;

    return;
  }

  std::string materialName = derived->Name();
  Ogre::MaterialPtr ogreMaterial = derived->Material();
  this->dataPtr->material = derived;

  this->dataPtr->material->SetReceiveShadows(false);
  this->dataPtr->material->SetLightingEnabled(false);

  switch (markerType)
  {
    case NONE:
    case BOX:
    case CYLINDER:
    case SPHERE:
      this->dataPtr->mesh->SetMaterial(derived, false);
      break;
    case LINE_STRIP:
    case LINE_LIST:
    case POINTS:
    case TRIANGLE_FAN:
    case TRIANGLE_LIST:
    case TRIANGLE_STRIP:
      break;
    default:
      ignerr << "Invalid Marker type " << markerType << "\n";
      break;
  }
}

//////////////////////////////////////////////////
MaterialPtr Ogre2Marker::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
void Ogre2Marker::SetPoint(unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_value*/)
{
}

//////////////////////////////////////////////////
void Ogre2Marker::AddPoint(const ignition::math::Vector3d &/*_pt*/,
    const ignition::math::Color &/*_color*/)
{
}

//////////////////////////////////////////////////
void Ogre2Marker::ClearPoints()
{
}

//////////////////////////////////////////////////
void Ogre2Marker::SetType(MarkerType _markerType)
{
  if (_markerType == this->markerType)
    return;

  this->markerType = _markerType;

  auto visual = std::dynamic_pointer_cast<Ogre2Visual>(this->Parent());

  // clear geom if needed
  if (this->dataPtr->mesh)
  {
    if (visual)
    {
      visual->RemoveGeometry(
          std::dynamic_pointer_cast<Geometry>(shared_from_this()));
    }
    this->dataPtr->mesh->Destroy();
  }


  GeometryPtr newMesh;
  switch (_markerType)
  {
    case NONE:
      newMesh = this->scene->CreateBox();
      break;
    case BOX:
      newMesh = this->scene->CreateBox();
      break;
    case CYLINDER:
      newMesh = this->scene->CreateCylinder();
      break;
    case SPHERE:
      newMesh = this->scene->CreateSphere();
      break;
    case LINE_STRIP:
    case LINE_LIST:
    case POINTS:
    case TRIANGLE_FAN:
    case TRIANGLE_LIST:
    case TRIANGLE_STRIP:
      // todo(anyone) support dynamic renderables
      break;
    default:
      ignerr << "Invalid Marker type\n";
      break;
  }

  if (newMesh)
  {
    this->dataPtr->mesh = std::dynamic_pointer_cast<Ogre2Mesh>(newMesh);
    if (visual)
    {
      visual->AddGeometry(
          std::dynamic_pointer_cast<Geometry>(shared_from_this()));
    }
  }
}

//////////////////////////////////////////////////
MarkerType Ogre2Marker::Type() const
{
  return this->dataPtr->markerType;
}
