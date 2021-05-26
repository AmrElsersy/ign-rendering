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

#include "ignition/common/Console.hh"
#include "ignition/rendering/ogre2/Ogre2Camera.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2SelectionBuffer.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/Utils.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2SegmentationCamera.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"
#include "ignition/math/Color.hh"

// #include "stdlib.h"
using namespace ignition;
using namespace rendering;


namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \brief Helper class to assign unique colors to renderables
    class SegmentationMaterialSwitcher : public Ogre::RenderTargetListener
    {
      /// \brief Constructor
      public: explicit SegmentationMaterialSwitcher(Ogre2ScenePtr _scene);

      /// \brief Destructor
      public: ~SegmentationMaterialSwitcher();

      /// \brief Ogre's pre render update callback
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      public: virtual void preRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      /// \brief Ogre's post render update callback
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      public: virtual void postRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      /// \brief Get color for background & unlabeled items in the colored map
      /// \return Color of background & unlabeled items 
      public: math::Color BackgroundColor();

      /// \brief Set color for background & unlabeled items in the colored map
      /// \param[in] _color Color of background & unlabeled items 
      public: void SetBackgroundColor(math::Color _color);

      /// \brief Set label for background & unlabeled items in the semantic map
      /// \param[in] _color label of background & unlabeled items 
      public: void SetBackgroundLabel(int _label);

      /// \brief Set label for background & unlabeled items in the semantic map
      /// \return label of background & unlabeled items 
      public: int BackgroundLabel();

      /// \brief Convert label of semantic map to a unique color for colored map
      /// \param[in] _label id of the semantic map
      /// \return _color unique color in the colored map for that label
      private: math::Color LabelToColor(int _label);

      /// \brief A map of ogre sub item pointer to their original hlms material
      private: std::map<Ogre::SubItem *, Ogre::HlmsDatablock *> datablockMap;

      /// \brief Ogre v1 material consisting of a shader that changes the
      /// appearance of item to use a unique color for mouse picking
      private: Ogre::MaterialPtr plainMaterial;

      /// \brief Ogre v1 material consisting of a shader that changes the
      /// appearance of item to use a unique color for mouse picking. In
      /// addition, the depth check and depth write properties disabled.
      private: Ogre::MaterialPtr plainOverlayMaterial;

      /// \brief User Data Key to set the label
      private: std::string labelKey = "label";

      /// \brief Background & unlabeled objects label id in semantic map
      private: int backgroundLabel = 0;

      /// \brief Background & unlabeled objects color in the colored map
      private: math::Color backgroundColor;

      /// \brief Ogre2 Scene 
      private: Ogre2ScenePtr scene;
    };
    }
  }
}

/////////////////////////////////////////////////
SegmentationMaterialSwitcher::SegmentationMaterialSwitcher(Ogre2ScenePtr _scene)
{  
  this->scene = _scene;
  this->backgroundColor.Set(0, 0, 0);

  // plain material to switch item's material
  Ogre::ResourcePtr res =
    Ogre::MaterialManager::getSingleton().load("ign-rendering/plain_color",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  this->plainMaterial = res.staticCast<Ogre::Material>();
  this->plainMaterial->load();

  // plain overlay material
  this->plainOverlayMaterial =
      this->plainMaterial->clone("plain_color_overlay");
  if (!this->plainOverlayMaterial->getTechnique(0) ||
      !this->plainOverlayMaterial->getTechnique(0)->getPass(0))
  {
    ignerr << "Problem creating selection buffer overlay material"
        << std::endl;
    return;
  }
  Ogre::Pass *overlayPass =
      this->plainOverlayMaterial->getTechnique(0)->getPass(0);
  Ogre::HlmsMacroblock macroblock(*overlayPass->getMacroblock());
  macroblock.mDepthCheck = false;
  macroblock.mDepthWrite = false;
  overlayPass->setMacroblock(macroblock);
}

/////////////////////////////////////////////////
SegmentationMaterialSwitcher::~SegmentationMaterialSwitcher()
{
}

/////////////////////////////////////////////////
math::Color SegmentationMaterialSwitcher::BackgroundColor()
{
  return this->backgroundColor;
}

/////////////////////////////////////////////////
void SegmentationMaterialSwitcher::SetBackgroundColor(math::Color _color)
{
  this->backgroundColor = _color;
}

/////////////////////////////////////////////////
int SegmentationMaterialSwitcher::BackgroundLabel()
{
  return this->backgroundLabel;
}

/////////////////////////////////////////////////
void SegmentationMaterialSwitcher::SetBackgroundLabel(int _label)
{
  this->backgroundLabel = _label;
}

/////////////////////////////////////////////////
math::Color SegmentationMaterialSwitcher::LabelToColor(int _label)
{
  if (_label == this->backgroundLabel)
    return this->backgroundColor;

  // use label as seed to generate the same color for the label
  srand(_label);  

  // random color
  int r = (rand()) % 255;
  int g = (rand()) % 255;
  int b = (rand()) % 255;

  return math::Color(r, g, b);
}

////////////////////////////////////////////////
void SegmentationMaterialSwitcher::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  this->datablockMap.clear();
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);

  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);

    // get visual from ogre item
    Ogre::Any userAny = item->getUserObjectBindings().getUserAny();

    if (!userAny.isEmpty() && userAny.getType() == typeid(unsigned int))
    {
      VisualPtr visual;
      try
      {
        visual = this->scene->VisualById(Ogre::any_cast<unsigned int>(userAny));
      }
      catch(Ogre::Exception &e)
      {
        ignerr << "Ogre Error:" << e.getFullDescription() << "\n";
      }
      Ogre2VisualPtr ogreVisual = std::dynamic_pointer_cast<Ogre2Visual>(visual);

      // get class user data
      Variant classAny = ogreVisual->UserData(this->labelKey);  

      int label;
      try
      {
        label = std::get<int>(classAny);
      }
      catch(std::bad_variant_access &e)
      {
        // items with no class are considered background
        label = this->backgroundLabel;
      }

      // item->setVisibilityFlags(Ogre::VisibilityFlags::LAYER_VISIBILITY);
      // // item->setVisibilityFlags(Ogre::VisibilityFlags::RESERVED_VISIBILITY_FLAGS);
      for (unsigned int i = 0; i < item->getNumSubItems(); i++)
      {
        // save subitems material 
        Ogre::SubItem *subItem = item->getSubItem(i);
        Ogre::HlmsDatablock *datablock = subItem->getDatablock();
        this->datablockMap[subItem] = datablock;

        // switch to semantic material
        // subItem->setCustomParameter(1, Ogre::Vector4(
        //   label, label, label, 1.0));

        math::Color color = this->LabelToColor(label);
        subItem->setCustomParameter(1, Ogre::Vector4(
          color.R(), color.G(), color.B(), 1.0));

        // check if it's an overlay material by assuming the
        // depth check and depth write properties are off.
        if (!datablock->getMacroblock()->mDepthWrite &&
            !datablock->getMacroblock()->mDepthCheck)
          subItem->setMaterial(this->plainOverlayMaterial);
        else
          subItem->setMaterial(this->plainMaterial);
      }      
    }
    itor.moveNext();
  }
}

/////////////////////////////////////////////////
void SegmentationMaterialSwitcher::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent &/*_evt*/)
{
  // restore item to use hlms material
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);
    for (unsigned int i = 0; i < item->getNumSubItems(); i++)
    {
      Ogre::SubItem *subItem = item->getSubItem(i);
      auto it = this->datablockMap.find(subItem);
      if (it != this->datablockMap.end())
        subItem->setDatablock(it->second);
    }
    itor.moveNext();
  }
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
  auto backgroundColor = Ogre2Conversions::Convert(
    this->dataPtr->materialSwitcher->BackgroundColor());

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
  auto bufferSize = Ogre::PixelUtil::getMemorySize(
    width, height, 1, this->dataPtr->format);
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

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::SetBackgroundColor(math::Color _color)
{
  return this->dataPtr->materialSwitcher->SetBackgroundColor(_color);
}

/////////////////////////////////////////////////
void Ogre2SegmentationCamera::SetBackgroundLabel(int _label)
{
  return this->dataPtr->materialSwitcher->SetBackgroundLabel(_label);
}

/////////////////////////////////////////////////
math::Color Ogre2SegmentationCamera::BackgroundColor()
{
  return this->dataPtr->materialSwitcher->BackgroundColor();
}

/////////////////////////////////////////////////
int Ogre2SegmentationCamera::BackgroundLabel()
{
  return this->dataPtr->materialSwitcher->BackgroundLabel();
}
