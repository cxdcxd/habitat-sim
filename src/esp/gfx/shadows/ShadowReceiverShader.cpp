/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2016 — Bill Robinson <airbaggins@gmail.com>
*/

#include "ShadowReceiverShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/TextureArray.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Math/Matrix4.h>

// This is to import the "resources" at runtime.
// When the resource is compiled into static library,
// it must be explicitly initialized via this macro, and should be called
// *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace esp {
namespace gfx {

ShadowReceiverShader::ShadowReceiverShader(Magnum::UnsignedInt numShadowLevels)
    : _shadowMapNumLayers{numShadowLevels} {
  MAGNUM_ASSERT_GL_VERSION_SUPPORTED(Magnum::GL::Version::GL330);

  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

  const Magnum::Utility::Resource rs{"default-shaders"};

  Magnum::GL::Shader vert{Magnum::GL::Version::GL330,
                          Magnum::GL::Shader::Type::Vertex};
  Magnum::GL::Shader frag{Magnum::GL::Version::GL330,
                          Magnum::GL::Shader::Type::Fragment};

  std::string preamble =
      "#define NUM_SHADOW_MAP_LEVELS " + std::to_string(numShadowLevels) + "\n";
  vert.addSource(preamble);
  vert.addSource(rs.get("ShadowReceiver.vert"));
  frag.addSource(preamble);
  frag.addSource(rs.get("ShadowReceiver.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Magnum::GL::Shader::compile({vert, frag}));

  bindAttributeLocation(Position::Location, "position");
  bindAttributeLocation(Normal::Location, "normal");

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  _modelMatrixUniform = uniformLocation("modelMatrix");
  _transformationProjectionMatrixUniform =
      uniformLocation("transformationProjectionMatrix");
  _shadowmapMatrixUniform = uniformLocation("shadowmapMatrix");
  _lightDirectionUniform = uniformLocation("lightDirection");
  _shadowBiasUniform = uniformLocation("shadowBias");

  setUniform(uniformLocation("shadowmapTexture"), ShadowmapTextureLayer);
}

ShadowReceiverShader& ShadowReceiverShader::setTransformationProjectionMatrix(
    const Magnum::Matrix4& matrix) {
  setUniform(_transformationProjectionMatrixUniform, matrix);
  return *this;
}

ShadowReceiverShader& ShadowReceiverShader::setModelMatrix(
    const Magnum::Matrix4& matrix) {
  setUniform(_modelMatrixUniform, matrix);
  return *this;
}

ShadowReceiverShader& ShadowReceiverShader::setShadowmapMatrices(
    const Corrade::Containers::ArrayView<const Magnum::Matrix4> matrices) {
  setUniform(_shadowmapMatrixUniform, matrices);
  return *this;
}

ShadowReceiverShader& ShadowReceiverShader::setLightDirection(
    const Magnum::Vector3& vector) {
  setUniform(_lightDirectionUniform, vector);
  return *this;
}

ShadowReceiverShader& ShadowReceiverShader::setShadowmapTexture(
    Magnum::GL::Texture2DArray& texture) {
  texture.bind(ShadowmapTextureLayer);
  return *this;
}

ShadowReceiverShader& ShadowReceiverShader::setShadowBias(
    const Magnum::Float bias) {
  setUniform(_shadowBiasUniform, bias);
  return *this;
}

}  // namespace gfx
}  // namespace esp
