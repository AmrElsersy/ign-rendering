/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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


#include <string>

#include "ignition/common/Console.hh"
#include "ignition/math/Color.hh"
#include "ignition/rendering/ogre2/Ogre2Camera.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2ParticleEmitter.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2SegmentationCamera.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Utils.hh"

#include "Ogre2SegmentationMaterialSwitcher.hh"

/// \brief Private data for the Ogre2SegmentationCamera class
class ignition::rendering::Ogre2SegmentationCameraPrivate
{
  /// \brief buffer to store render texture data & to be sent to listeners
  public: uint8_t *buffer {nullptr};

  /// \brief Workspace Definition
  public: std::string ogreCompositorWorkspaceDef;

  /// \brief Final pass compositor node definition
  public: std::string ogreCompositorNodeDef;

  /// \brief 1st pass compositor workspace
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace {nullptr};

  /// \brief Output texture
  public: Ogre::TextureGpu *ogreSegmentationTexture {nullptr};

  /// \brief Dummy render texture for the depth data
  public: RenderTexturePtr segmentationTexture {nullptr};

  /// \brief The segmentation material
  public: Ogre::MaterialPtr segmentationMaterial;

  /// \brief New Segmentation Frame Event to notify listeners with new data
  /// \param[in] _data Segmentation buffer data
  /// \param[in] _width Width of the image
  /// \param[in] _height Height of the image
  /// \param[in] _channels Number of channels
  /// \param[in] _format Image Format
  public: ignition::common::EventT<void(const uint8_t *_data,
    unsigned int _width, unsigned int _height, unsigned int _channels,
    const std::string &_format)> newSegmentationFrame;

  /// \brief Material Switcher to switch item's material
  /// with colored version for segmentation
  public: std::unique_ptr<Ogre2SegmentationMaterialSwitcher>
          materialSwitcher {nullptr};

};

using namespace ignition;
using namespace rendering;

/////////////////////////////////////////////////
Ogre2SegmentationCamera::Ogre2SegmentationCamera() :
  dataPtr(new Ogre2SegmentationCameraPrivate())
{
}

/////////////////////////////////////////////////
Ogre2SegmentationCamera::~Ogre2SegmentationCamera()
{
  this->Destroy();
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::Init()
{
  BaseCamera::Init();

  this->CreateCamera();

  this->CreateRenderTexture();

  this->dataPtr->materialSwitcher.reset(
      new Ogre2SegmentationMaterialSwitcher(this->scene));
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::Destroy()
{
  if (this->dataPtr->buffer)
  {
    delete [] this->dataPtr->buffer;
    this->dataPtr->buffer = nullptr;
  }

  if (!this->ogreCamera)
    return;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  auto ogreCompMgr = ogreRoot->getCompositorManager2();

  this->dataPtr->materialSwitcher.reset();

  if (this->dataPtr->ogreSegmentationTexture)
  {
    ogreRoot->getRenderSystem()->getTextureGpuManager()->destroyTexture(
      this->dataPtr->ogreSegmentationTexture);
    this->dataPtr->ogreSegmentationTexture = nullptr;

  }
  if (this->dataPtr->ogreCompositorWorkspace)
  {
    ogreCompMgr->removeWorkspace(
        this->dataPtr->ogreCompositorWorkspace);
  }
  if (this->dataPtr->segmentationMaterial)
  {
    Ogre::MaterialManager::getSingleton().remove(
        this->dataPtr->segmentationMaterial->getName());
  }

  if (!this->dataPtr->ogreCompositorWorkspaceDef.empty())
  {
    ogreCompMgr->removeWorkspaceDefinition(
        this->dataPtr->ogreCompositorWorkspaceDef);
    ogreCompMgr->removeNodeDefinition(
        this->dataPtr->ogreCompositorNodeDef);
  }

  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
  }
  else
  {
    if (ogreSceneManager->findCameraNoThrow(this->name) != nullptr)
    {
      ogreSceneManager->destroyCamera(this->ogreCamera);
      this->ogreCamera = nullptr;
    }
  }
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::PreRender()
{
  if (!this->dataPtr->ogreSegmentationTexture)
    this->CreateSegmentationTexture();
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateCamera()
{
  auto ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->ogreCamera = ogreSceneManager->createCamera(this->Name());
  if (this->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  this->ogreCamera->detachFromParent();
  this->ogreNode->attachObject(this->ogreCamera);

  // rotate to ignition gazebo coord.
  this->ogreCamera->yaw(Ogre::Degree(-90));
  this->ogreCamera->roll(Ogre::Degree(-90));
  this->ogreCamera->setFixedYawAxis(false);

  this->ogreCamera->setAutoAspectRatio(true);
  this->ogreCamera->setRenderingDistance(100);
  this->ogreCamera->setProjectionType(Ogre::ProjectionType::PT_PERSPECTIVE);
  this->ogreCamera->setCustomProjectionMatrix(false);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateSegmentationTexture()
{
  // Camera Parameters
  this->ogreCamera->setNearClipDistance(this->NearClipPlane());
  this->ogreCamera->setFarClipDistance(this->FarClipPlane());
  this->ogreCamera->setAspectRatio(this->AspectRatio());
  double vfov = 2.0 * atan(tan(this->HFOV().Radian() / 2.0) /
    this->AspectRatio());
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  this->SetImageFormat(PixelFormat::PF_R8G8B8);
  Ogre::PixelFormatGpu ogrePF = Ogre::PFG_RGB8_UNORM;

  std::string wsDefName = "SegmentationCameraWorkspace_" + this->Name();
  this->dataPtr->ogreCompositorWorkspaceDef = wsDefName;
  if(!ogreCompMgr->hasWorkspaceDefinition(wsDefName))
  {
    std::string nodeDefName = wsDefName + "/Node";
    this->dataPtr->ogreCompositorNodeDef = nodeDefName;
    Ogre::CompositorNodeDef *nodeDef =
        ogreCompMgr->addNodeDefinition(nodeDefName);
    // Input texture
    nodeDef->addTextureSourceName("rt_input", 0,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    Ogre::TextureDefinitionBase::TextureDefinition *segmentationTexDef =
        nodeDef->addTextureDefinition("depthTexture");
    segmentationTexDef->textureType = Ogre::TextureTypes::Type2D;
    segmentationTexDef->width = 0;
    segmentationTexDef->height = 0;
    segmentationTexDef->depthOrSlices = 1;
    segmentationTexDef->numMipmaps = 0;
    segmentationTexDef->widthFactor = 1;
    segmentationTexDef->heightFactor = 1;
    segmentationTexDef->format = Ogre::PFG_D32_FLOAT;
    segmentationTexDef->textureFlags &= ~Ogre::TextureFlags::Uav;
    // set to default pool so that when the colorTexture pass is rendered, its
    // depth data get populated to depthTexture
    segmentationTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    segmentationTexDef->depthBufferFormat = Ogre::PFG_UNKNOWN;

    Ogre::RenderTargetViewDef *rtv =
      nodeDef->addRenderTextureView("depthTexture");
    rtv->setForTextureDefinition("depthTexture", segmentationTexDef);

    Ogre::TextureDefinitionBase::TextureDefinition *colorTexDef =
        nodeDef->addTextureDefinition("colorTexture");
    colorTexDef->textureType = Ogre::TextureTypes::Type2D;
    colorTexDef->width = 0;
    colorTexDef->height = 0;
    colorTexDef->depthOrSlices = 1;
    colorTexDef->numMipmaps = 0;
    colorTexDef->widthFactor = 1;
    colorTexDef->heightFactor = 1;
    colorTexDef->format = Ogre::PFG_RGBA8_UNORM;
    colorTexDef->textureFlags &= ~Ogre::TextureFlags::Uav;
    colorTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    colorTexDef->depthBufferFormat = Ogre::PFG_D32_FLOAT;
    colorTexDef->preferDepthTexture = true;

    Ogre::RenderTargetViewDef *rtv2 =
      nodeDef->addRenderTextureView("colorTexture");
    rtv2->setForTextureDefinition("colorTexture", colorTexDef);

    nodeDef->setNumTargetPass(2);
    Ogre::CompositorTargetDef *colorTargetDef =
        nodeDef->addTargetPass("colorTexture");
    colorTargetDef->setNumPasses(1);
    {
      // scene pass
      Ogre::CompositorPassSceneDef *passScene =
          static_cast<Ogre::CompositorPassSceneDef *>(
          colorTargetDef->addPass(Ogre::PASS_SCENE));
      passScene->setAllLoadActions(Ogre::LoadAction::Clear);
      passScene->setAllClearColours(Ogre::ColourValue(0, 0, 0));
      // segmentation camera should not see particles
      passScene->mVisibilityMask = IGN_VISIBILITY_ALL &
          ~Ogre2ParticleEmitter::kParticleVisibilityFlags;
    }

    // rt_input target - converts to thermal
    Ogre::CompositorTargetDef *inputTargetDef =
        nodeDef->addTargetPass("rt_input");
    inputTargetDef->setNumPasses(1);
    {
      // quad pass
      Ogre::CompositorPassQuadDef *passQuad =
          static_cast<Ogre::CompositorPassQuadDef *>(
          inputTargetDef->addPass(Ogre::PASS_QUAD));
      passQuad->setAllLoadActions(Ogre::LoadAction::Clear);
      passQuad->setAllClearColours(Ogre::ColourValue(0, 0, 0));

      passQuad->mMaterialName = this->dataPtr->segmentationMaterial->getName();
      passQuad->addQuadTextureSource(0, "depthTexture");
      passQuad->addQuadTextureSource(1, "colorTexture");
      passQuad->mFrustumCorners =
          Ogre::CompositorPassQuadDef::VIEW_SPACE_CORNERS;
    }
    nodeDef->mapOutputChannel(0, "rt_input");
    Ogre::CompositorWorkspaceDef *workDef =
        ogreCompMgr->addWorkspaceDefinition(wsDefName);
    workDef->connectExternal(0, nodeDef->getName(), 0);
  }

  Ogre::CompositorWorkspaceDef *wsDef =
      ogreCompMgr->getWorkspaceDefinition(wsDefName);

  if (!wsDef)
  {
    ignerr << "Unable to add workspace definition [" << wsDefName << "] "
           << " for " << this->Name();
  }

  Ogre::TextureGpuManager *textureMgr =
    ogreRoot->getRenderSystem()->getTextureGpuManager();
  // create render texture - these textures pack the thermal data
  this->dataPtr->ogreSegmentationTexture =
    textureMgr->createOrRetrieveTexture(this->Name() + "_segmentation",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::RenderToTexture,
      Ogre::TextureTypes::Type2D);

  this->dataPtr->ogreSegmentationTexture->setResolution(
      this->ImageWidth(), this->ImageHeight());
  this->dataPtr->ogreSegmentationTexture->setNumMipmaps(1u);
  this->dataPtr->ogreSegmentationTexture->setPixelFormat(ogrePF);
  this->dataPtr->ogreSegmentationTexture->scheduleTransitionTo(
    Ogre::GpuResidency::Resident);

  // create compositor worksspace
  this->dataPtr->ogreCompositorWorkspace =
      ogreCompMgr->addWorkspace(
        this->scene->OgreSceneManager(),
        this->dataPtr->ogreSegmentationTexture,
        this->ogreCamera,
        wsDefName,
        false);

  // add segmentaiton material switcher to render target listener
  Ogre::CompositorNode *node =
      this->dataPtr->ogreCompositorWorkspace->getNodeSequence()[0];
  auto channels = node->getLocalTextures();
  for (auto c : channels)
  {
    if (c->getPixelFormat() == Ogre::PFG_RGBA8_UNORM)
    {
      this->ogreCamera->addListener(
        this->dataPtr->materialSwitcher.get());
      break;
    }
  }
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::PostRender()
{
  // return if no one is listening to the new frame
  if (this->dataPtr->newSegmentationFrame.ConnectionCount() == 0)
    return;

  const auto width = this->ImageWidth();
  const auto height = this->ImageHeight();
  PixelFormat format = this->ImageFormat();

  const auto len = width * height;
  const auto channelCount = PixelUtil::ChannelCount(format);
  const auto bytesPerChannel = PixelUtil::BytesPerChannel(format);
  const auto bufferSize = len * channelCount * bytesPerChannel;

  Ogre::Image2 image;
  image.convertFromTexture(this->dataPtr->ogreSegmentationTexture, 0u, 0u);
  Ogre::TextureBox box = image.getData(0);

  if (!this->dataPtr->buffer)
  {
    this->dataPtr->buffer = new uint8_t[bufferSize];
  }

  uint8_t *segmentationBufferTmp = static_cast<uint8_t*>(box.data);
  memcpy(&this->dataPtr->buffer, segmentationBufferTmp, bufferSize);

  this->dataPtr->newSegmentationFrame(
    this->dataPtr->buffer,
    width, height, channelCount,
    PixelUtil::Name(format));
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr
  Ogre2SegmentationCamera::ConnectNewSegmentationFrame(
  std::function<void(const uint8_t *, unsigned int, unsigned int,
  unsigned int, const std::string &)>  _subscriber)
{
  return this->dataPtr->newSegmentationFrame.Connect(_subscriber);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::Render()
{
  // update the compositors
  this->scene->StartRendering();

  this->dataPtr->ogreCompositorWorkspace->_validateFinalTarget();
  this->dataPtr->ogreCompositorWorkspace->_beginUpdate(false);
  this->dataPtr->ogreCompositorWorkspace->_update();
  this->dataPtr->ogreCompositorWorkspace->_endUpdate(false);

  Ogre::vector<Ogre::TextureGpu*>::type swappedTargets;
  swappedTargets.reserve(2u);
  this->dataPtr->ogreCompositorWorkspace->_swapFinalTarget(swappedTargets);

  this->scene->FlushGpuCommandsAndStartNewFrame(1u, false);
}

/////////////////////////////////////////////////
RenderTargetPtr Ogre2SegmentationCamera::RenderTarget() const
{
  return this->dataPtr->segmentationTexture;
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->segmentationTexture =
    std::dynamic_pointer_cast<Ogre2RenderTexture>(base);
  this->dataPtr->segmentationTexture->SetWidth(1);
  this->dataPtr->segmentationTexture->SetHeight(1);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::SetBackgroundColor(
  const math::Color &_color)
{
  this->dataPtr->materialSwitcher->backgroundColor.Set(
    _color.R(), _color.G(), _color.B()
  );
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::SetBackgroundLabel(int _label)
{
  this->dataPtr->materialSwitcher->backgroundLabel = _label;
  this->SetBackgroundColor(
    math::Color(_label / 255.0, _label / 255.0, _label / 255.0));
}

/////////////////////////////////////////////////
math::Color Ogre2SegmentationCamera::BackgroundColor() const
{
  return this->dataPtr->materialSwitcher->backgroundColor;
}

/////////////////////////////////////////////////
int Ogre2SegmentationCamera::BackgroundLabel() const
{
  return this->dataPtr->materialSwitcher->backgroundLabel;
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::SetSegmentationType(SegmentationType _type)
{
  this->dataPtr->materialSwitcher->type = _type;
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::EnableColoredMap(bool _enable)
{
  this->dataPtr->materialSwitcher->isColoredMap = _enable;
}

/////////////////////////////////////////////////
SegmentationType Ogre2SegmentationCamera::Type() const
{
  return this->dataPtr->materialSwitcher->type;
}

/////////////////////////////////////////////////
bool Ogre2SegmentationCamera::IsColoredMap() const
{
  return this->dataPtr->materialSwitcher->isColoredMap;
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::LabelMapFromColoredBuffer(
  uint8_t * _labelBuffer) const
{
  if (!this->dataPtr->materialSwitcher->isColoredMap)
    return;

  if (!this->dataPtr->buffer)
    return;

  const auto &colorToLabel = this->dataPtr->materialSwitcher->colorToLabel;

  auto width = this->ImageWidth();
  auto height = this->ImageHeight();

  for (uint32_t i = 0; i < height; i++)
  {
    for (uint32_t j = 0; j < width; j++)
    {
      auto index = (i * width + j) * 3;
      auto r = this->dataPtr->buffer[index + 2];
      auto g = this->dataPtr->buffer[index + 1];
      auto b = this->dataPtr->buffer[index];

      // get color 24 bit unique id, we don't multiply it by 255 like before
      // as they are not normalized we read it from the buffer in
      // range [0-255] already
      int64_t colorId = r * 256 * 256 + g * 256 + b;

      // initialize the pixel with the background label value
      {
        uint8_t label8bit = this->dataPtr->materialSwitcher->backgroundLabel;

        _labelBuffer[index] =     label8bit;
        _labelBuffer[index + 1] = label8bit;
        _labelBuffer[index + 2] = label8bit;
      }

      // skip if not exist
      auto it = colorToLabel.find(colorId);
      if (it == colorToLabel.end())
        continue;

      int64_t label = it->second;

      if (this->dataPtr->materialSwitcher->type ==
          SegmentationType::ST_SEMANTIC)
      {
        uint8_t label8bit = label % 256;

        _labelBuffer[index] =     label8bit;
        _labelBuffer[index + 1] = label8bit;
        _labelBuffer[index + 2] = label8bit;
      }
      else if (this->dataPtr->materialSwitcher->type ==
        SegmentationType::ST_PANOPTIC)
      {
        // get the label and instance counts from the composite label id
        uint8_t label8bit = label / (256 * 256);
        // get the rest 16 bit
        uint16_t instanceCount = label % (256 * 256);
        // composite that 16 bit count to two 8 bit channels
        uint8_t instanceCount1 = instanceCount / 256;
        uint8_t instanceCount2 = instanceCount % 256;

        _labelBuffer[index + 2] = label8bit;
        _labelBuffer[index + 1] = instanceCount1;
        _labelBuffer[index] = instanceCount2;
      }
    }
  }
}
