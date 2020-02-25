/*
    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include "ShadowLight.h"

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Frustum.h>
#include <Magnum/SceneGraph/FeatureGroup.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <algorithm>

#include "esp/geo/geo.h"
#include "esp/gfx/shadows/ShadowCasterDrawable.h"
#include "esp/scene/SceneNode.h"

namespace Mn = Magnum;
namespace esp {
namespace gfx {

ShadowLight::ShadowLight(scene::SceneNode& parent)
    : Mn::SceneGraph::Camera3D{parent}, _shadowTexture{Mn::NoCreate} {
  setAspectRatioPolicy(Mn::SceneGraph::AspectRatioPolicy::NotPreserved);
}

void ShadowLight::setupShadowmaps(Mn::Int numShadowLevels,
                                  const Mn::Vector2i& size) {
  _layers.clear();

  (_shadowTexture = Mn::GL::Texture2DArray{})
      .setImage(0, Mn::GL::TextureFormat::DepthComponent,
                Mn::ImageView3D{Mn::GL::PixelFormat::DepthComponent,
                                Mn::GL::PixelType::Float,
                                {size, numShadowLevels}})
      .setMaxLevel(0)
      .setCompareFunction(Mn::GL::SamplerCompareFunction::LessOrEqual)
      .setCompareMode(Mn::GL::SamplerCompareMode::CompareRefToTexture)
      .setMinificationFilter(Mn::GL::SamplerFilter::Linear,
                             Mn::GL::SamplerMipmap::Base)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear);

  for (std::int_fast32_t i = 0; i < numShadowLevels; ++i) {
    _layers.emplace_back(size);
    Mn::GL::Framebuffer& shadowFramebuffer = _layers.back().shadowFramebuffer;
    shadowFramebuffer
        .attachTextureLayer(Mn::GL::Framebuffer::BufferAttachment::Depth,
                            _shadowTexture, 0, i)
        .mapForDraw(Mn::GL::Framebuffer::DrawAttachment::None)
        .bind();
    CORRADE_INTERNAL_ASSERT(
        shadowFramebuffer.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
        Mn::GL::Framebuffer::Status::Complete);
  }
}

ShadowLight::ShadowLayerData::ShadowLayerData(const Mn::Vector2i& size)
    : shadowFramebuffer{{{}, size}} {}

void ShadowLight::setTarget(const Mn::Vector3& lightDirection,
                            const Mn::Vector3& screenDirection,
                            MagnumCamera& mainCamera) {
  Mn::Matrix4 cameraMatrix =
      Mn::Matrix4::lookAt({}, -lightDirection, screenDirection);
  const Mn::Matrix3x3 cameraRotationMatrix = cameraMatrix.rotation();
  const Mn::Matrix3x3 inverseCameraRotationMatrix =
      cameraRotationMatrix.inverted();

  for (std::size_t layerIndex = 0; layerIndex != _layers.size(); ++layerIndex) {
    std::vector<Mn::Vector3> mainCameraFrustumCorners =
        layerFrustumCorners(mainCamera, Mn::Int(layerIndex));
    ShadowLayerData& layer = _layers[layerIndex];

    /* Calculate the AABB in shadow-camera space */
    Mn::Vector3 min{std::numeric_limits<Mn::Float>::max()},
        max{std::numeric_limits<Mn::Float>::lowest()};
    for (Mn::Vector3 worldPoint : mainCameraFrustumCorners) {
      Mn::Vector3 cameraPoint = inverseCameraRotationMatrix * worldPoint;
      min = Mn::Math::min(min, cameraPoint);
      max = Mn::Math::max(max, cameraPoint);
    }

    /* Place the shadow camera at the mid-point of the camera box */
    const Mn::Vector3 mid = (min + max) * 0.5f;
    const Mn::Vector3 cameraPosition = cameraRotationMatrix * mid;

    const Mn::Vector3 range = max - min;
    /* Set up the initial extends of the shadow map's render volume. Note
       we will adjust this later when we render. */
    layer.orthographicSize = range.xy();
    layer.orthographicNear = -0.5f * range.z();
    layer.orthographicFar = 0.5f * range.z();
    cameraMatrix.translation() = cameraPosition;
    layer.shadowCameraMatrix = cameraMatrix;
  }
}

Mn::Float ShadowLight::cutZ(const Mn::Int layer) const {
  return _layers[layer].cutPlane;
}

void ShadowLight::setupSplitDistances(const Mn::Float zNear,
                                      const Mn::Float zFar,
                                      const Mn::Float power) {
  /* props http://stackoverflow.com/a/33465663 */
  for (std::size_t i = 0; i != _layers.size(); ++i) {
    const Mn::Float linearDepth =
        zNear +
        std::pow(Mn::Float(i + 1) / _layers.size(), power) * (zFar - zNear);
    const Mn::Float nonLinearDepth =
        (zFar + zNear - 2.0f * zNear * zFar / linearDepth) / (zFar - zNear);
    _layers[i].cutPlane = (nonLinearDepth + 1.0f) / 2.0f;
  }
}

Mn::Float ShadowLight::cutDistance(const Mn::Float zNear,
                                   const Mn::Float zFar,
                                   const Mn::Int layer) const {
  const Mn::Float depthSample = 2.0f * _layers[layer].cutPlane - 1.0f;
  const Mn::Float zLinear =
      2.0f * zNear * zFar / (zFar + zNear - depthSample * (zFar - zNear));
  return zLinear;
}

std::vector<Mn::Vector3> ShadowLight::layerFrustumCorners(
    MagnumCamera& mainCamera,
    const Mn::Int layer) {
  const Mn::Float z0 = layer == 0 ? 0 : _layers[layer - 1].cutPlane;
  const Mn::Float z1 = _layers[layer].cutPlane;
  return cameraFrustumCorners(mainCamera, z0, z1);
}

std::vector<Mn::Vector3> ShadowLight::cameraFrustumCorners(
    MagnumCamera& mainCamera,
    const Mn::Float z0,
    const Mn::Float z1) {
  const Mn::Matrix4 imvp =
      (mainCamera.projectionMatrix() * mainCamera.cameraMatrix()).inverted();
  return frustumCorners(imvp, z0, z1);
}

std::vector<Mn::Vector3> ShadowLight::frustumCorners(const Mn::Matrix4& imvp,
                                                     const Mn::Float z0,
                                                     const Mn::Float z1) {
  return {imvp.transformPoint({-1, -1, z0}), imvp.transformPoint({1, -1, z0}),
          imvp.transformPoint({-1, 1, z0}),  imvp.transformPoint({1, 1, z0}),
          imvp.transformPoint({-1, -1, z1}), imvp.transformPoint({1, -1, z1}),
          imvp.transformPoint({-1, 1, z1}),  imvp.transformPoint({1, 1, z1})};
}

std::vector<Mn::Vector4> ShadowLight::calculateClipPlanes() {
  const Mn::Matrix4 pm = projectionMatrix();
  std::vector<Mn::Vector4> clipPlanes{
      {pm[3][0] + pm[2][0], pm[3][1] + pm[2][1], pm[3][2] + pm[2][2],
       pm[3][3] + pm[2][3]}, /* near */
      {pm[3][0] - pm[2][0], pm[3][1] - pm[2][1], pm[3][2] - pm[2][2],
       pm[3][3] - pm[2][3]}, /* far */
      {pm[3][0] + pm[0][0], pm[3][1] + pm[0][1], pm[3][2] + pm[0][2],
       pm[3][3] + pm[0][3]}, /* left */
      {pm[3][0] - pm[0][0], pm[3][1] - pm[0][1], pm[3][2] - pm[0][2],
       pm[3][3] - pm[0][3]}, /* right */
      {pm[3][0] + pm[1][0], pm[3][1] + pm[1][1], pm[3][2] + pm[1][2],
       pm[3][3] + pm[1][3]}, /* bottom */
      {pm[3][0] - pm[1][0], pm[3][1] - pm[1][1], pm[3][2] - pm[1][2],
       pm[3][3] - pm[1][3]}}; /* top */
  for (Mn::Vector4& plane : clipPlanes)
    plane *= plane.xyz().lengthInverted();
  return clipPlanes;
}

void ShadowLight::render(MagnumDrawableGroup& drawables) {
  /* Compute transformations of all objects in the group relative to the camera
   */
  std::vector<std::reference_wrapper<MagnumObject>> objects;
  objects.reserve(drawables.size());
  for (std::size_t i = 0; i != drawables.size(); ++i)
    objects.push_back(static_cast<MagnumObject&>(drawables[i].object()));
  std::vector<ShadowCasterDrawable*> filteredDrawables;

  /* Projecting world points normalized device coordinates means they range
     -1 -> 1. Use this bias matrix so we go straight from world -> texture
     space */
  constexpr const Mn::Matrix4 bias{{0.5f, 0.0f, 0.0f, 0.0f},
                                   {0.0f, 0.5f, 0.0f, 0.0f},
                                   {0.0f, 0.0f, 0.5f, 0.0f},
                                   {0.5f, 0.5f, 0.5f, 1.0f}};

  Mn::GL::Renderer::setDepthMask(true);

  for (std::size_t layer = 0; layer != _layers.size(); ++layer) {
    ShadowLayerData& d = _layers[layer];
    Mn::Float orthographicNear = d.orthographicNear;
    const Mn::Float orthographicFar = d.orthographicFar;

    /* Move this whole object to the right place to render each layer */
    object().setTransformation(d.shadowCameraMatrix).setClean();
    setProjectionMatrix(Mn::Matrix4::orthographicProjection(
        d.orthographicSize, orthographicNear, orthographicFar));

    // const std::vector<Mn::Vector4> clipPlanes = calculateClipPlanes();
    // camera frustum relative to world origin
    const Mn::Frustum frustum =
        Mn::Frustum::fromMatrix(projectionMatrix() * cameraMatrix());
    // Skip out the near plane because we need to include shadow casters
    // traveling the direction the camera is facing.
    const std::vector<Mn::Vector4> clipPlanes{frustum.left(), frustum.right(),
                                              frustum.top(), frustum.bottom(),
                                              frustum.far()};
    std::vector<Mn::Matrix4> transformations =
        object().scene()->transformationMatrices(objects, cameraMatrix());

    /* Rebuild the list of objects we will draw by clipping them with the
       shadow camera's planes */
    std::size_t transformationsOutIndex = 0;
    filteredDrawables.clear();
    for (std::size_t drawableIndex = 0; drawableIndex != drawables.size();
         ++drawableIndex) {
      bool drawnByLayer = true;
      auto& drawable =
          static_cast<ShadowCasterDrawable&>(drawables[drawableIndex]);
      auto& node = static_cast<scene::SceneNode&>(objects[drawableIndex].get());
      const Mn::Matrix4 transform = transformations[drawableIndex];

      /* If your centre is offset, inject it here */
      const Mn::Vector4 localCentre{0.0f, 0.0f, 0.0f, 1.0f};
      const Mn::Vector4 drawableCentre = transform * localCentre;

      Corrade::Containers::Optional<Mn::Range3D> aabb = node.getAbsoluteAABB();

      if (aabb) {
        auto& range = *aabb;
        const Mn::Vector3 center = range.min() + range.max();
        const Mn::Vector3 extent = range.max() - range.min();
        for (const auto& plane : clipPlanes) {
          const Mn::Vector3 absPlaneNormal = Mn::Math::abs(plane.xyz());

          const float d = Mn::Math::dot(center, plane.xyz());
          const float r = Mn::Math::dot(extent, absPlaneNormal);
          if (d + r < -2.0 * plane.w()) {
            // useless side of plane, we can skip
            drawnByLayer = false;
            break;
          }
        }

        if (!drawnByLayer) {
          continue;
        }

        {
          /* If this object extends in front of the near plane, extend
             the near plane. We negate the z because the negative z is
             forward away from the camera, but the near/far planes are
             measured forwards. */
          // TODO: need to implement distance to AABB
          const Mn::Float nearestPoint = 0;
          orthographicNear = Mn::Math::min(orthographicNear, nearestPoint);
          filteredDrawables.push_back(&drawable);
          transformations[transformationsOutIndex++] = transform;
        }
      }

      /* Start at 1, not 0 to skip out the near plane because we need to
         include shadow casters traveling the direction the camera is
         facing. */
      //   for (std::size_t clipPlaneIndex = 1; clipPlaneIndex !=
      //   clipPlanes.size();
      //        ++clipPlaneIndex) {
      //     const Mn::Float distance =
      //         Mn::Math::dot(clipPlanes[clipPlaneIndex], drawableCentre);

      //     /* If the object is on the useless side of any one plane, we can
      //     skip it
      //      */
      //     if (distance < -drawable.radius())
      //       goto next;
      //   }
    }

    /* Recalculate the projection matrix with new near plane. */
    const Mn::Matrix4 shadowCameraProjectionMatrix =
        Mn::Matrix4::orthographicProjection(d.orthographicSize,
                                            orthographicNear, orthographicFar);
    d.shadowMatrix = bias * shadowCameraProjectionMatrix * cameraMatrix();
    setProjectionMatrix(shadowCameraProjectionMatrix);

    d.shadowFramebuffer.clear(Mn::GL::FramebufferClear::Depth).bind();
    for (std::size_t i = 0; i != transformationsOutIndex; ++i)
      filteredDrawables[i]->draw(transformations[i], *this);
  }

  Mn::GL::defaultFramebuffer.bind();
}

}  // namespace gfx
}  // namespace esp
