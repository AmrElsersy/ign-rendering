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

// leave this out of OgreIncludes as it conflicts with other files requiring
// gl.h
#include <GL/glew.h>
#include <OGRE/RenderSystems/GL/OgreGLFBORenderTexture.h>


#include <ignition/common/Console.hh>

#include "ignition/rendering/Material.hh"

#include "ignition/rendering/ogre/OgreRenderEngine.hh"
#include "ignition/rendering/ogre/OgreConversions.hh"
#include "ignition/rendering/ogre/OgreMaterial.hh"
#include "ignition/rendering/ogre/OgreRenderTarget.hh"
#include "ignition/rendering/ogre/OgreRTShaderSystem.hh"
#include "ignition/rendering/ogre/OgreScene.hh"
#include "ignition/rendering/ogre/OgreIncludes.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
// OgreRenderTarget
//////////////////////////////////////////////////
OgreRenderTarget::OgreRenderTarget()
{
  this->ogreBackgroundColor = Ogre::ColourValue::Black;
}

//////////////////////////////////////////////////
OgreRenderTarget::~OgreRenderTarget()
{
  // TODO(anyone): clean up check null

  OgreRTShaderSystem::Instance()->DetachViewport(this->ogreViewport,
      this->scene);
}

//////////////////////////////////////////////////
void OgreRenderTarget::Copy(Image &_image) const
{
  // TODO(anyone): handle Bayer conversions
  // TODO(anyone): handle ogre version differences

  if (_image.Width() != this->width || _image.Height() != this->height)
  {
    ignerr << "Invalid image dimensions" << std::endl;
    return;
  }

  void* data = _image.Data();
  Ogre::PixelFormat imageFormat = OgreConversions::Convert(_image.Format());
  Ogre::PixelBox ogrePixelBox(this->width, this->height, 1, imageFormat, data);
  this->RenderTarget()->copyContentsToMemory(ogrePixelBox);
}

//////////////////////////////////////////////////
Ogre::Camera *OgreRenderTarget::Camera() const
{
  return this->ogreCamera;
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetCamera(Ogre::Camera *_camera)
{
  this->ogreCamera = _camera;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
math::Color OgreRenderTarget::BackgroundColor() const
{
  return OgreConversions::Convert(this->ogreBackgroundColor);
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetBackgroundColor(math::Color _color)
{
  this->ogreBackgroundColor = OgreConversions::Convert(_color);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
unsigned int OgreRenderTarget::AntiAliasing() const
{
  return this->antiAliasing;
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetAntiAliasing(unsigned int _aa)
{
  this->antiAliasing = _aa;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void OgreRenderTarget::PreRender()
{
  BaseRenderTarget::PreRender();
  this->UpdateBackgroundColor();

  if (this->material)
  {
    this->material->PreRender();
  }
}

//////////////////////////////////////////////////
void OgreRenderTarget::PostRender()
{
  // do nothing by default
}

//////////////////////////////////////////////////
void OgreRenderTarget::Render()
{
  this->RenderTarget()->update();
}

//////////////////////////////////////////////////
void OgreRenderTarget::UpdateBackgroundColor()
{
  if (this->colorDirty && this->ogreViewport)
  {
    this->ogreViewport->setBackgroundColour(this->ogreBackgroundColor);
    this->colorDirty = false;
  }
}

//////////////////////////////////////////////////
void OgreRenderTarget::RebuildImpl()
{
  this->RebuildTarget();
  this->RebuildViewport();
  this->RebuildMaterial();
}

//////////////////////////////////////////////////
void OgreRenderTarget::RebuildViewport()
{
  Ogre::RenderTarget *ogreRenderTarget = this->RenderTarget();
  ogreRenderTarget->removeAllViewports();

  this->ogreViewport = ogreRenderTarget->addViewport(this->ogreCamera);
  this->ogreViewport->setBackgroundColour(this->ogreBackgroundColor);
  this->ogreViewport->setClearEveryFrame(true);
  this->ogreViewport->setShadowsEnabled(true);
  this->ogreViewport->setOverlaysEnabled(false);

  OgreRTShaderSystem::Instance()->AttachViewport(this->ogreViewport,
      this->scene);
}

//////////////////////////////////////////////////
void OgreRenderTarget::SetMaterial(MaterialPtr _material)
{
  this->material = _material;

  // Have to rebuild the target so there is something to apply the applicator to
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void OgreRenderTarget::RebuildMaterial()
{
  if (this->material)
  {
    OgreMaterial *ogreMaterial = dynamic_cast<OgreMaterial*>(
        this->material.get());
    Ogre::MaterialPtr matPtr = ogreMaterial->Material();

    Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
    Ogre::RenderTarget *target = this->RenderTarget();
    this->materialApplicator.reset(new OgreRenderTargetMaterial(
        sceneMgr, target, matPtr.get()));
  }
}

//////////////////////////////////////////////////
// OgreRenderTexture
//////////////////////////////////////////////////
OgreRenderTexture::OgreRenderTexture()
{
}

//////////////////////////////////////////////////
OgreRenderTexture::~OgreRenderTexture()
{
}

//////////////////////////////////////////////////
void OgreRenderTexture::Destroy()
{
  std::string ogreName = this->ogreTexture->getName();
  Ogre::TextureManager::getSingleton().remove(ogreName);
}

//////////////////////////////////////////////////
Ogre::RenderTarget *OgreRenderTexture::RenderTarget() const
{
  return this->ogreTexture->getBuffer()->getRenderTarget();
}

//////////////////////////////////////////////////
void OgreRenderTexture::RebuildTarget()
{
  this->DestroyTarget();
  this->BuildTarget();
}

//////////////////////////////////////////////////
void OgreRenderTexture::DestroyTarget()
{
  // TODO(anyone): implement
}

//////////////////////////////////////////////////
void OgreRenderTexture::BuildTarget()
{
  Ogre::TextureManager &manager = Ogre::TextureManager::getSingleton();
  Ogre::PixelFormat ogreFormat = OgreConversions::Convert(this->format);

  this->ogreTexture = (manager.createManual(this->name, "General",
      Ogre::TEX_TYPE_2D, this->width, this->height, 0, ogreFormat,
      Ogre::TU_RENDERTARGET, 0, false, this->antiAliasing)).getPointer();
}

//////////////////////////////////////////////////
GLuint OgreRenderTexture::GLId()
{
  if (!this->ogreTexture)
    return GLuint(0);

  GLuint texId;
  this->ogreTexture->getCustomAttribute("GLID", &texId);

  return texId;
}

//////////////////////////////////////////////////
void OgreRenderTexture::PreRender()
{
  OgreRenderTarget::PreRender();
  if (!this->ogreTexture)
    return;


  Ogre::RenderTarget *rt = this->RenderTarget();

  Ogre::GLFrameBufferObject *ogreFbo = nullptr;
  rt->getCustomAttribute("FBO", &ogreFbo);
  Ogre::GLFBOManager *manager = ogreFbo->getManager();
  manager->bind(rt);
}

//////////////////////////////////////////////////
void OgreRenderTexture::PostRender()
{
  if (!this->ogreTexture)
    return;

  Ogre::RenderTarget *rt = this->RenderTarget();

  Ogre::GLFrameBufferObject *ogreFbo = nullptr;
  rt->getCustomAttribute("FBO", &ogreFbo);
  Ogre::GLFBOManager *manager = ogreFbo->getManager();
  manager->unbind(rt);
}


//////////////////////////////////////////////////
// OgreRenderWindow
//////////////////////////////////////////////////
OgreRenderWindow::OgreRenderWindow()
{
}

//////////////////////////////////////////////////
OgreRenderWindow::~OgreRenderWindow()
{
}

//////////////////////////////////////////////////
Ogre::RenderTarget *OgreRenderWindow::RenderTarget() const
{
  return this->ogreRenderWindow;
}

//////////////////////////////////////////////////
void OgreRenderWindow::Destroy()
{
  // if (this->ogreRenderWindow)
  //  this->ogreRenderWindow->destroy();
}

//////////////////////////////////////////////////
void OgreRenderWindow::RebuildTarget()
{
  // TODO(anyone) determine when to rebuild
  // ie. only when ratio or handle changes!
  // e.g. sizeDirty?
  if (!this->ogreRenderWindow)
    this->BuildTarget();

  Ogre::RenderWindow *window =
      dynamic_cast<Ogre::RenderWindow *>(this->ogreRenderWindow);
  window->resize(this->width, this->height);
  window->windowMovedOrResized();
}

//////////////////////////////////////////////////
void OgreRenderWindow::BuildTarget()
{
  auto engine = OgreRenderEngine::Instance();
  std::string renderTargetName =
      engine->CreateWindow(this->handle,
          this->width,
          this->height,
          this->ratio,
          this->antiAliasing);
  this->ogreRenderWindow =
      engine->OgreRoot()->getRenderTarget(renderTargetName);
}
