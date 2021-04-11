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

#include "ignition/rendering/ogre2/Ogre2Camera.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2SelectionBuffer.hh"
#include "ignition/rendering/Utils.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"

#include "ignition/rendering/ogre2/Ogre2SegmentationCamera.hh"
#include "ignition/rendering/ogre2/Ogre2MaterialSwitcher.hh"
// #include "stdlib.h"
using namespace ignition;
using namespace rendering;


class SegmentationMaterialSwitcher : public Ogre2MaterialSwitcher
{
  /// \brief Constructor
  /// \param[in] _scene pointer to the scene
  public: SegmentationMaterialSwitcher(Ogre2ScenePtr _scene);

  /// \brief Destructor
  public: ~SegmentationMaterialSwitcher();

  /// \brief set the currentColor with random color for movable object
  public: virtual void NextColor() override;

  /// \brief Reset the pseudo number generator that generates colors  
  public: virtual void Reset() override;

  /// \brief seed(initial value) of the pseudo number generator
  /// used to generate the same sequence 
  protected: uint seed = 1;
};

/////////////////////////////////////////////////
SegmentationMaterialSwitcher::SegmentationMaterialSwitcher(Ogre2ScenePtr _scene) : 
  Ogre2MaterialSwitcher(_scene)
{
  srand(this->seed);
}

/////////////////////////////////////////////////
SegmentationMaterialSwitcher::~SegmentationMaterialSwitcher()
{
}

/////////////////////////////////////////////////
void SegmentationMaterialSwitcher::NextColor()
{
  // random color
  int r = (rand()) % 255;
  int g = (rand()) % 255;
  int b = (rand()) % 255;

  this->currentColor.Set(r, g, b);
}

/////////////////////////////////////////////////
void SegmentationMaterialSwitcher::Reset()
{
  // reset the pseudo num generator with the same seed(initial value)
  // to get random values with the same sequence
  srand(this->seed);

  this->currentColor = ignition::math::Color(0.0, 0.0, 0.0);
  this->colorDict.clear();
}

/////////////////////////////////////////////////
class ignition::rendering::Ogre2SegmentationCameraPrivate
{
  /// \brief Material Switcher to switch item's material 
  /// with colored version for segmentation
  public: SegmentationMaterialSwitcher *materialSwitcher;

  /// \brief Compositor Manager to create workspace
  public: Ogre::CompositorManager2 *ogreCompositorManager;

  /// \brief Workspace to interface with render texture 
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace;

  /// \brief Workspace Definition 
  public: std::string workspaceDefinition;

  /// \brief Render Texture to store the final segmentation data
  public: Ogre::RenderTexture *ogreRenderTexture;

  /// \brief Texture to create the render texture from.
  public: Ogre::TexturePtr ogreTexture;

  /// \brief Pixel Box to copy render texture data to a buffer
  public: Ogre::PixelBox *pixelBox;

  /// \brief buffer to store render texture data & to be sent to listeners
  public: uint8_t *buffer = nullptr;

  /// \brief dummy render texture to set image dims
  public: Ogre2RenderTexturePtr dummyTexture;

  /// \brief New Segmentation Frame Event to notify listeners with new data
  public: ignition::common::EventT<void(const uint8_t *, unsigned int, unsigned int,
        unsigned int, const std::string &)> newSegmentationFrame;

  /// \brief Image / Render Texture Format
  public: Ogre::PixelFormat format = Ogre::PF_R8G8B8;
};

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
  this->Reset();
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateCamera()
{
  auto ogreScene = this->scene->OgreSceneManager();
  if (ogreScene == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->ogreCamera = ogreScene->createCamera(this->Name());
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
void Ogre2SegmentationCamera::Destroy()
{
  BaseCamera::Destroy();
  Camera::Destroy();
  
  delete this->dataPtr->materialSwitcher;
  delete this->dataPtr->ogreCompositorWorkspace;
  delete this->dataPtr->ogreCompositorManager;

  auto &manager = Ogre::TextureManager::getSingleton();
  manager.unload(this->dataPtr->ogreTexture->getName());
  manager.remove(this->dataPtr->ogreTexture->getName());
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::PreRender()
{
  if (!this->dataPtr->ogreRenderTexture)
    this->CreateSegmentationTexture();
  
  this->dataPtr->materialSwitcher->Reset();
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateSegmentationTexture()
{
  this->dataPtr->materialSwitcher = new SegmentationMaterialSwitcher(this->scene);

  // Camera Parameters
  this->ogreCamera->setNearClipDistance(this->NearClipPlane());
  this->ogreCamera->setFarClipDistance(this->FarClipPlane());
  this->ogreCamera->setAspectRatio(this->AspectRatio());
  double vfov = 2.0 * atan(tan(this->HFOV().Radian() / 2.0) / this->AspectRatio());
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));

  auto width = this->ImageWidth();
  auto height = this->ImageHeight();

  // texture
  this->dataPtr->ogreTexture = Ogre::TextureManager::getSingleton().createManual(
    "SegmentationCameraTexture",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D, width, height, 0, this->dataPtr->format, Ogre::TU_RENDERTARGET
  );

  // render texture
  auto hardwareBuffer = this->dataPtr->ogreTexture->getBuffer();
  this->dataPtr->ogreRenderTexture = hardwareBuffer->getRenderTarget();

  // add material switcher to switch the material to a unique color for each object
  // in the pre render & get the original material again in the post render
  this->dataPtr->ogreRenderTexture->addListener(this->dataPtr->materialSwitcher);

  // workspace
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  this->dataPtr->ogreCompositorManager = ogreRoot->getCompositorManager2();

  this->dataPtr->workspaceDefinition = "SegmentationCameraWorkspace_" + this->Name();
  auto backgroundColor = Ogre2Conversions::Convert(this->scene->BackgroundColor());

  // basic workspace consist of clear pass with the givin color & 
  // a render scene pass to the givin render texture
  this->dataPtr->ogreCompositorManager->createBasicWorkspaceDef(
    this->dataPtr->workspaceDefinition,
    backgroundColor
    );

  // connect the compositor with the render texture to render the final output to it
  this->dataPtr->ogreCompositorWorkspace = 
    this->dataPtr->ogreCompositorManager->addWorkspace(
      this->scene->OgreSceneManager(),
      this->dataPtr->ogreRenderTexture,
      this->ogreCamera,
      this->dataPtr->workspaceDefinition,
      false
    );

  // set visibility mask
  auto node = this->dataPtr->ogreCompositorWorkspace->getNodeSequence()[0];
  auto pass = node->_getPasses()[1]->getDefinition();
  auto renderScenePass = dynamic_cast<const Ogre::CompositorPassSceneDef *>(pass);
  const_cast<Ogre::CompositorPassSceneDef *>(renderScenePass)->setVisibilityMask(
    IGN_VISIBILITY_ALL
  );

  // buffer to store render texture data
  auto bufferSize = Ogre::PixelUtil::getMemorySize(width, height, 1, this->dataPtr->format);
  this->dataPtr->buffer = new uint8_t[bufferSize];
  this->dataPtr->pixelBox = new Ogre::PixelBox(width , height ,1, 
    this->dataPtr->format, this->dataPtr->buffer);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::Render()
{
  this->dataPtr->ogreCompositorWorkspace->setEnabled(true);
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  ogreRoot->renderOneFrame();
  this->dataPtr->ogreCompositorWorkspace->setEnabled(false);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::PostRender()
{
  // copy render texture data to the pixel box & its buffer
  this->dataPtr->ogreRenderTexture->copyContentsToMemory(
    *this->dataPtr->pixelBox,
    Ogre::RenderTarget::FB_FRONT
  );

  // return if no one is listening to the new frame
  if (this->dataPtr->newSegmentationFrame.ConnectionCount() == 0)
    return;

  uint width = this->ImageWidth();
  uint height = this->ImageHeight();
  uint channelCount = 3;

  this->dataPtr->newSegmentationFrame(
    this->dataPtr->buffer, 
    width, height, channelCount, Ogre::PixelUtil::getFormatName(this->dataPtr->format)
  );
}

/////////////////////////////////////////////////
uint8_t *Ogre2SegmentationCamera::SegmentationData() const
{
    return this->dataPtr->buffer;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr Ogre2SegmentationCamera::ConnectNewSegmentationFrame(
  std::function<void(const uint8_t *, unsigned int, unsigned int,
  unsigned int, const std::string &)>  _subscriber)
{
  return this->dataPtr->newSegmentationFrame.Connect(_subscriber);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->dummyTexture = std::dynamic_pointer_cast<Ogre2RenderTexture>(base);
  this->dataPtr->dummyTexture->SetWidth(1);
  this->dataPtr->dummyTexture->SetHeight(1);
}

/////////////////////////////////////////////////
RenderTargetPtr Ogre2SegmentationCamera::RenderTarget() const
{
  return this->dataPtr->dummyTexture;
}
