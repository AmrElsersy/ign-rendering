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


#include <sstream>

#include <ignition/common/Console.hh>
#include <ignition/common/Material.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Skeleton.hh>
#include <ignition/common/SubMesh.hh>

#include <ignition/math/Matrix4.hh>

#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2Mesh.hh"
#include "ignition/rendering/ogre2/Ogre2MeshFactory.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Storage.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2MeshFactory::Ogre2MeshFactory(Ogre2ScenePtr _scene) :
  scene(_scene)
{
}

//////////////////////////////////////////////////
Ogre2MeshFactory::~Ogre2MeshFactory()
{
}

//////////////////////////////////////////////////
Ogre2MeshPtr Ogre2MeshFactory::Create(const MeshDescriptor &_desc)
{
  // create ogre entity
  Ogre2MeshPtr mesh(new Ogre2Mesh);
  MeshDescriptor normDesc = _desc;
  normDesc.Load();
  mesh->ogreItem = this->OgreItem(normDesc);

  // check if invalid mesh
  if (!mesh->ogreItem)
  {
    return nullptr;
  }

  // create sub-mesh store
  Ogre2SubMeshStoreFactory subMeshFactory(this->scene, mesh->ogreItem);
  mesh->subMeshes = subMeshFactory.Create();
  return mesh;
}

//////////////////////////////////////////////////
Ogre::Item *Ogre2MeshFactory::OgreItem(const MeshDescriptor &_desc)
{
  if (!this->Load(_desc))
  {
    return nullptr;
  }

  std::string name = this->MeshName(_desc);
  Ogre::SceneManager *sceneManager = this->scene->OgreSceneManager();

  Ogre::MeshPtr mesh =
      Ogre::MeshManager::getSingleton().getByName(name);
  if (!mesh)
  {
    std::cerr << "no mesh  creating one " << name << std::endl;
    Ogre::v1::MeshPtr v1Mesh =
        Ogre::v1::MeshManager::getSingleton().getByName(name);
    if (!v1Mesh)
    {
      ignerr << "Ogre v1 mesh was not created: " << name << std::endl;
      return nullptr;
    }

    mesh = Ogre::MeshManager::getSingleton().createManual(
        name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    mesh->importV1(v1Mesh.get(), true, true, true);
  }
  else
  {
    std::cerr << "got mesh  creating item " << name << std::endl;
  }

  return sceneManager->createItem(mesh, Ogre::SCENE_DYNAMIC);
}

//////////////////////////////////////////////////
bool Ogre2MeshFactory::Load(const MeshDescriptor &_desc)
{
  if (!this->Validate(_desc))
  {
    return false;
  }

  if (this->IsLoaded(_desc))
  {
    return true;
  }

  return this->LoadImpl(_desc);
}

//////////////////////////////////////////////////
bool Ogre2MeshFactory::IsLoaded(const MeshDescriptor &_desc)
{
  std::string name = this->MeshName(_desc);
  return Ogre::MeshManager::getSingleton().resourceExists(name);
}

//////////////////////////////////////////////////
bool Ogre2MeshFactory::LoadImpl(const MeshDescriptor &_desc)
{
  Ogre::v1::MeshPtr ogreMesh;
  std::string name;
  std::string group;

  Ogre2RenderEngine::Instance()->AddResourcePath(_desc.mesh->Path());

  try
  {
    name = this->MeshName(_desc);
    group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
    ogreMesh = Ogre::v1::MeshManager::getSingleton().createManual(name, group);

    Ogre::v1::SkeletonPtr ogreSkeleton;

    if (_desc.mesh->HasSkeleton())
    {
      common::SkeletonPtr skel = _desc.mesh->MeshSkeleton();
      ogreSkeleton = Ogre::v1::OldSkeletonManager::getSingleton().create(
        _desc.mesh->Name() + "_skeleton",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        true);

      for (unsigned int i = 0; i < skel->NodeCount(); i++)
      {
        common::SkeletonNode *node = skel->NodeByHandle(i);
        Ogre::v1::OldBone *bone = ogreSkeleton->createBone(node->Name());

        if (node->Parent())
          ogreSkeleton->getBone(node->Parent()->Name())->addChild(bone);

        math::Matrix4d trans = node->Transform();
        math::Vector3d pos = trans.Translation();
        math::Quaterniond q = trans.Rotation();
        bone->setPosition(Ogre::Vector3(pos.X(), pos.Y(), pos.Z()));
        bone->setOrientation(Ogre::Quaternion(q.W(), q.X(), q.Y(), q.Z()));
        bone->setInheritOrientation(true);
        bone->setManuallyControlled(true);
        bone->setInitialState();
      }
      ogreMesh->setSkeletonName(_desc.mesh->Name() + "_skeleton");
    }

    for (unsigned int i = 0; i < _desc.mesh->SubMeshCount(); i++)
    {
      // if submesh is specified then load only that particualr submesh
      auto s = _desc.mesh->SubMeshByIndex(i).lock();
      if (!_desc.subMeshName.empty() && s &&
          s->Name() != _desc.subMeshName)
      {
        continue;
      }

      Ogre::v1::SubMesh *ogreSubMesh;
      Ogre::v1::VertexData *vertexData;
      Ogre::v1::VertexDeclaration* vertexDecl;
      Ogre::v1::HardwareVertexBufferSharedPtr vBuf;
      Ogre::v1::HardwareIndexBufferSharedPtr iBuf;
      float *vertices;
      uint32_t *indices;

      size_t currOffset = 0;

      // Copy the original submesh. We may need to modify the vertices, and
      // we don't want to change the original.
      common::SubMesh subMesh(*s.get());

      // Recenter the vertices if requested.
      if (_desc.centerSubMesh)
        subMesh.Center(math::Vector3d::Zero);

      // ogreSubMesh = ogreMesh->createSubMesh(subMesh.Name());
      ogreSubMesh = ogreMesh->createSubMesh("asdf");
      ogreSubMesh->useSharedVertices = false;
      if (subMesh.SubMeshPrimitiveType() == common::SubMesh::TRIANGLES)
      {
        ogreSubMesh->operationType =
            Ogre::OT_TRIANGLE_LIST;
      }
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::LINES)
      {
        ogreSubMesh->operationType = Ogre::OT_LINE_LIST;
      }
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::LINESTRIPS)
      {
        ogreSubMesh->operationType = Ogre::OT_LINE_STRIP;
      }
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::TRIFANS)
      {
        ogreSubMesh->operationType = Ogre::OT_TRIANGLE_FAN;
      }
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::TRISTRIPS)
      {
        ogreSubMesh->operationType =
            Ogre::OT_TRIANGLE_STRIP;
      }
      else if (subMesh.SubMeshPrimitiveType() == common::SubMesh::POINTS)
      {
        ogreSubMesh->operationType = Ogre::OT_POINT_LIST;
      }
      else
      {
        ignerr << "Unknown primitive type["
              << subMesh.SubMeshPrimitiveType() << "]\n";
      }

      ogreSubMesh->vertexData[Ogre::VpNormal] = new Ogre::v1::VertexData();
      ogreSubMesh->vertexData[Ogre::VpShadow] =
          ogreSubMesh->vertexData[Ogre::VpNormal];
      vertexData = ogreSubMesh->vertexData[Ogre::VpNormal];
      vertexDecl = vertexData->vertexDeclaration;

      // The vertexDecl should contain positions, blending weights, normals,
      // diffiuse colors, specular colors, tex coords. In that order.
      vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                             Ogre::VES_POSITION);
      currOffset += Ogre::v1::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

      // TODO: blending weights

      // normals
      if (subMesh.NormalCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3,
                               Ogre::VES_NORMAL);
        currOffset += Ogre::v1::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
      }

      // TODO: diffuse colors

      // TODO: specular colors

      // two dimensional texture coordinates
      if (subMesh.TexCoordCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
            Ogre::VES_TEXTURE_COORDINATES, 0);
        currOffset += Ogre::v1::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
      }

      // allocate the vertex buffer
      vertexData->vertexCount = subMesh.VertexCount();

      vBuf = Ogre::v1::HardwareBufferManager::getSingleton().createVertexBuffer(
                 vertexDecl->getVertexSize(0),
                 vertexData->vertexCount,
                 Ogre::v1::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                 false);

      vertexData->vertexBufferBinding->setBinding(0, vBuf);
      vertices = static_cast<float*>(vBuf->lock(
                      Ogre::v1::HardwareBuffer::HBL_DISCARD));

      if (_desc.mesh->HasSkeleton())
      {
        common::SkeletonPtr skel = _desc.mesh->MeshSkeleton();
        for (unsigned int j = 0; j < subMesh.NodeAssignmentsCount(); j++)
        {
          common::NodeAssignment na = subMesh.NodeAssignmentByIndex(j);
          Ogre::v1::VertexBoneAssignment vba;
          vba.vertexIndex = na.vertexIndex;
          vba.boneIndex = ogreSkeleton->getBone(skel->NodeByHandle(
                              na.nodeIndex)->Name())->getHandle();
          vba.weight = na.weight;
          ogreSubMesh->addBoneAssignment(vba);
        }
      }

      // allocate index buffer
      ogreSubMesh->indexData[Ogre::VpShadow] =
          ogreSubMesh->indexData[Ogre::VpNormal];
      ogreSubMesh->indexData[Ogre::VpNormal]->indexCount = subMesh.IndexCount();

      ogreSubMesh->indexData[Ogre::VpNormal]->indexBuffer =
        Ogre::v1::HardwareBufferManager::getSingleton().createIndexBuffer(
            Ogre::v1::HardwareIndexBuffer::IT_32BIT,
            ogreSubMesh->indexData[Ogre::VpNormal]->indexCount,
            Ogre::v1::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
            false);

      iBuf = ogreSubMesh->indexData[Ogre::VpNormal]->indexBuffer;
      indices = static_cast<uint32_t*>(
          iBuf->lock(Ogre::v1::HardwareBuffer::HBL_DISCARD));

      unsigned int j;

      // Add all the vertices
      for (j = 0; j < subMesh.VertexCount(); j++)
      {
        *vertices++ = subMesh.Vertex(j).X();
        *vertices++ = subMesh.Vertex(j).Y();
        *vertices++ = subMesh.Vertex(j).Z();

        if (subMesh.NormalCount() > 0)
        {
          *vertices++ = subMesh.Normal(j).X();
          *vertices++ = subMesh.Normal(j).Y();
          *vertices++ = subMesh.Normal(j).Z();
        }

        if (subMesh.TexCoordCount() > 0)
        {
          *vertices++ = subMesh.TexCoord(j).X();
          *vertices++ = subMesh.TexCoord(j).Y();
        }
      }

      // Add all the indices
      for (j = 0; j < subMesh.IndexCount(); j++)
        *indices++ = subMesh.Index(j);

      common::MaterialPtr material;
      material = _desc.mesh->MaterialByIndex(subMesh.MaterialIndex());

      if (material)
      {
        this->scene->CreateMaterial(*material);
        ogreSubMesh->setMaterialName(material->Name());
      }
      else
      {
        ogreSubMesh->setMaterialName("Default/White");
      }

      // Unlock
      vBuf->unlock();
      iBuf->unlock();
    }

    math::Vector3d max = _desc.mesh->Max();
    math::Vector3d min = _desc.mesh->Min();

    if (_desc.mesh->HasSkeleton())
    {
      min = math::Vector3d(-1, -1, -1);
      max = math::Vector3d(1, 1, 1);
    }

    if (!max.IsFinite())
    {
      ignerr << "Max bounding box is not finite[" << max << "]" << std::endl;
      return false;
    }

    if (!min.IsFinite())
    {
      ignerr << "Min bounding box is not finite[" << min << "]" << std::endl;
      return false;
    }

    ogreMesh->_setBounds(Ogre::AxisAlignedBox(
          Ogre::Vector3(min.X(), min.Y(), min.Z()),
          Ogre::Vector3(max.X(), max.Y(), max.Z())),
          false);

    // this line makes clear the mesh is loaded (avoids memory leaks)
    // ogreMesh->load();
  }
  catch(Ogre::Exception &e)
  {
    ignerr << "Unable to insert mesh[" << e.getDescription() << "]"
        << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
std::string Ogre2MeshFactory::MeshName(const MeshDescriptor &_desc)
{
  std::stringstream ss;
  ss << _desc.meshName << "::";
  ss << _desc.subMeshName << "::";
  ss << ((_desc.centerSubMesh) ? "CENTERED" : "ORIGINAL");
  return ss.str();
}

//////////////////////////////////////////////////
bool Ogre2MeshFactory::Validate(const MeshDescriptor &_desc)
{
  if (!_desc.mesh && _desc.meshName.empty())
  {
    ignerr << "Invalid mesh-descriptor, no mesh specified" << std::endl;
    return false;
  }

  if (!_desc.mesh)
  {
    ignerr << "Cannot load null mesh" << std::endl;
    return false;
  }

  if (_desc.mesh->SubMeshCount() == 0)
  {
    ignerr << "Cannot load mesh with zero sub-meshes" << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
Ogre2SubMeshStoreFactory::Ogre2SubMeshStoreFactory(Ogre2ScenePtr _scene,
    Ogre::Item *_item) :
  scene(_scene),
  ogreItem(_item)
{
  this->CreateNameList();
}

//////////////////////////////////////////////////
Ogre2SubMeshStoreFactory::~Ogre2SubMeshStoreFactory()
{
}

//////////////////////////////////////////////////
Ogre2SubMeshStorePtr Ogre2SubMeshStoreFactory::Create()
{
  Ogre2SubMeshStorePtr subMeshes(new Ogre2SubMeshStore);
  unsigned int count = this->ogreItem->getNumSubItems();

  for (unsigned int i = 0; i < count; ++i)
  {
    Ogre2SubMeshPtr subMesh = this->CreateSubMesh(i);
    subMeshes->Add(subMesh);
  }

  return subMeshes;
}

//////////////////////////////////////////////////
Ogre2SubMeshPtr Ogre2SubMeshStoreFactory::CreateSubMesh(unsigned int _index)
{
  Ogre2SubMeshPtr subMesh(new Ogre2SubMesh);

  subMesh->id = _index;
  subMesh->name = this->names[_index];
  subMesh->scene = this->scene;
  subMesh->ogreSubItem = this->ogreItem->getSubItem(_index);

  MaterialPtr mat;
  Ogre::HlmsDatablock *ogreDatablock = subMesh->ogreSubItem->getDatablock();
  if (ogreDatablock)
  {
    mat = this->scene->Material(
        std::string(*ogreDatablock->getNameStr()));
    std::cerr << "subMesh " << subMesh->name << " block exists: "
        << std::string(*ogreDatablock->getNameStr()) << std::endl;
  }
  else
  {
    Ogre::MaterialPtr ogreMat = subMesh->ogreSubItem->getMaterial();
    if (ogreMat)
    {

      std::cerr << "subMesh " << subMesh->name << " mat exists: "
         << ogreMat->getName() << std::endl;
      mat = this->scene->Material(ogreMat->getName());
    }
    else

      std::cerr << "subMesh " << subMesh->name << " mat does not exists" << std::endl;
  }
  if (!mat)
  {
    mat = this->scene->CreateMaterial();
    std::cerr << "subMesh " << subMesh->name << " no mat" << std::endl;
  }
  subMesh->SetMaterial(mat);
/*
  MaterialPtr mat;
  Ogre::MaterialPtr ogreMat = subMesh->ogreSubItem->getMaterial();
  if (ogreMat)
    mat = this->scene->Material(ogreMat->getName());
  if (!mat)
  {
    mat = this->scene->CreateMaterial();
  }
  subMesh->SetMaterial(mat);
*/

  subMesh->Load();
  subMesh->Init();

  return subMesh;
}

//////////////////////////////////////////////////
void Ogre2SubMeshStoreFactory::CreateNameList()
{
  this->PopulateDefaultNames();
//  this->PopulateGivenNames();
}

//////////////////////////////////////////////////
void Ogre2SubMeshStoreFactory::PopulateDefaultNames()
{
  unsigned int count = this->ogreItem->getNumSubItems();
  this->names.reserve(count);

  for (unsigned int i = 0; i < count; ++i)
  {
    this->names.push_back("SubMesh(" + std::to_string(i) + ")");
  }
}

//////////////////////////////////////////////////
void Ogre2SubMeshStoreFactory::PopulateGivenNames()
{
  const Ogre::MeshPtr ogreMesh = this->ogreItem->getMesh();

  for (auto pair : ogreMesh->getSubMeshNameMap())
  {
    std::string name = pair.first;
    unsigned int index = pair.second;
    this->names[index] = name;
  }
}
