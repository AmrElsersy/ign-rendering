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

#ifndef IGNITION_RENDERING_OGRE2_OGRE2MATERIALSWITCHER_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2MATERIALSWITCHER_HH_

#include <map>
#include <string>
#include <random>
#include <unordered_map>
#include <unordered_set>

#include <ignition/math/Color.hh>
#include "ignition/rendering/config.hh"
#include "ignition/rendering/ogre2/Export.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2SegmentationCamera.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
// forward declarations
class Ogre2SelectionBuffer;

/// \brief Helper class to assign unique colors to renderables
class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2SegmentationMaterialSwitcher :
  public Ogre::RenderTargetListener
{
  /// \brief Constructor
  public: explicit Ogre2SegmentationMaterialSwitcher(Ogre2ScenePtr _scene);

  /// \brief Destructor
  public: ~Ogre2SegmentationMaterialSwitcher() = default;

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

  /// \brief Convert label of semantic map to a unique color for colored map and
  /// add the color of the label to the taken colors if it doesn't exist
  /// \param[in] _label id of the semantic map or encoded id of panoptic map
  /// \param[in] _isMultiLink bool used to skip the taken color check if the
  /// label is for a multi link model, as all links should have the same color
  /// \return _color unique color in the colored map for that label
  private: math::Color LabelToColor(int64_t _label,
    bool _isMultiLink = false);

  /// \brief Get the top level model visual of a particular visual
  /// \param[in] _visual The visual who's top level model visual we are
  /// interested in
  /// \return The top level model visual of _visual
  private: VisualPtr TopLevelModelVisual(VisualPtr _visual) const;

  /// \brief Check if the color is already taken and add it to taken colors
  /// if it does not exist
  /// \param[in] _color Color to be checked
  /// \return True if taken, False otherwise
  private: bool IsTakenColor(const math::Color &_color);

  /// \brief A map of ogre sub item pointer to its original hlms material
  private: std::unordered_map<Ogre::SubItem *,
    Ogre::HlmsDatablock *> datablockMap;

  /// \brief Ogre v1 material consisting of a shader that changes the
  /// appearance of item to use a unique color for mouse picking
  private: Ogre::MaterialPtr plainMaterial;

  /// \brief Ogre v1 material consisting of a shader that changes the
  /// appearance of item to use a unique color for mouse picking. In
  /// addition, the depth check and depth write properties disabled.
  private: Ogre::MaterialPtr plainOverlayMaterial;

  /// \brief Background & unlabeled objects label id in semantic map
  private: int backgroundLabel = 0;

  /// \brief Background & unlabeled objects color in the colored map
  private: math::Color backgroundColor {0, 0, 0};

  /// \brief Segmentation Type
  private: SegmentationType type {SegmentationType::ST_SEMANTIC};

  /// \brief True to generate colored map
  /// False to generate labels ids map
  private: bool isColoredMap {false};

  /// \brief Keep track of num of instances of the same label
  /// Key: label id, value: num of instances
  private: std::unordered_map<unsigned int, unsigned int> instancesCount;

  /// \brief keep track of the random colors, stores encoded id of r,g,b
  private: std::unordered_set<int64_t> takenColors;

  /// \brief keep track of the labels that are already colored.
  /// Usful for coloring items in semantic mode in LabelToColor()
  private: std::unordered_set<int64_t> coloredLabel;

  /// \brief Mapping from the colorId to the label id, used in converting
  /// the colored map to label ids map
  /// Key: colorId label id, value: label in case of semantic segmentation
  /// or composite id (8 bit label + 16 bit instances) in instance type
  private: std::unordered_map<int64_t, int64_t> colorToLabel;

  /// \brief Pseudo num generator to generate colors from label id
  private: std::default_random_engine generator;

  /// \brief Ogre2 Scene
  private: Ogre2ScenePtr scene = nullptr;

  friend class Ogre2SegmentationCamera;
};
} 
}  // namespace rendering
}  // namespace ignition
#endif
