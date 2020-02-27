#pragma once
/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Shaders/Generic.h>

namespace esp {
namespace gfx {

/** @brief Shader that can synthesize shadows on an object */
class ShadowReceiverShader : public Magnum::GL::AbstractShaderProgram {
 public:
  typedef Magnum::Shaders::Generic3D::Position Position;
  typedef Magnum::Shaders::Generic3D::Normal Normal;

  explicit ShadowReceiverShader(Magnum::NoCreateT)
      : Magnum::GL::AbstractShaderProgram{Magnum::NoCreate} {}

  explicit ShadowReceiverShader(Magnum::UnsignedInt numShadowLevels);

  /**
   * @brief Set transformation and projection matrix
   *
   * Matrix that transforms from local model space -> world space ->
   * camera space -> clip coordinates (aka model-view-projection matrix).
   */
  ShadowReceiverShader& setTransformationProjectionMatrix(
      const Magnum::Matrix4& matrix);

  /**
   * @brief Set model matrix
   *
   * Matrix that transforms from local model space -> world space (used
   * for lighting).
   */
  ShadowReceiverShader& setModelMatrix(const Magnum::Matrix4& matrix);

  /**
   * @brief Set shadowmap matrices
   *
   * Matrix that transforms from world space -> shadow texture space.
   */
  ShadowReceiverShader& setShadowmapMatrices(
      Corrade::Containers::ArrayView<const Magnum::Matrix4> matrices);

  /** @brief Set world-space direction to the light source */
  ShadowReceiverShader& setLightDirection(const Magnum::Vector3& vector3);

  /** @brief Set shadow map texture array */
  ShadowReceiverShader& setShadowmapTexture(
      Magnum::GL::Texture2DArray& texture);

  Magnum::UnsignedInt getNumLayers() const { return _shadowMapNumLayers; }

  /**
   * @brief Set thadow bias uniform
   *
   * Normally it wants to be something from 0.0001 -> 0.001.
   */
  ShadowReceiverShader& setShadowBias(Magnum::Float bias);

 private:
  Magnum::UnsignedInt _shadowMapNumLayers;
  enum : Magnum::Int { ShadowmapTextureLayer = 0 };

  Magnum::Int _modelMatrixUniform, _transformationProjectionMatrixUniform,
      _shadowmapMatrixUniform, _lightDirectionUniform, _shadowBiasUniform;
};

}  // namespace gfx
}  // namespace esp
