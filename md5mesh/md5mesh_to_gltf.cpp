/*  MD5Mesh2GLTF
    Copyright(C) 2022 Lukas Cone

    This program is free software : you can redistribute it and / or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.If not, see <https://www.gnu.org/licenses/>.
*/
#include "project.h"
#include "spike/app_context.hpp"
#include "spike/except.hpp"
#include "spike/gltf.hpp"
#include "spike/io/binreader_stream.hpp"
#include "spike/io/binwritter_stream.hpp"
#include <map>

std::string_view filters[]{".md5mesh$"};

static AppInfo_s appInfo{
    .header = MD5Mesh2GLTF_DESC " v" MD5Mesh2GLTF_VERSION
                                ", " MD5Mesh2GLTF_COPYRIGHT "Lukas Cone",
    .filters = filters,
};

AppInfo_s *AppInitModule() { return &appInfo; }

struct Vertex {
  Vector position;
  Vector2 uv;
  UCVector4 boneIds;
  UCVector4 boneWeights;
  SVector4 normal;
};

struct GLTFMain : GLTF {
  GLTFStream &SkinStream() {
    if (ibmStream < 0) {
      auto &newStream = NewStream("ibms");
      ibmStream = newStream.slot;
      return newStream;
    }
    return Stream(ibmStream);
  }

  GLTFStream &GetIndexStream() {
    if (indexStream < 0) {
      auto &str = NewStream("indices");
      str.target = gltf::BufferView::TargetType::ElementArrayBuffer;
      indexStream = str.slot;
      return str;
    }

    return Stream(indexStream);
  }

  GLTFStream &GetVertexStream() {
    if (vtStream < 0) {
      auto &str = NewStream("vertices", sizeof(Vertex));
      str.target = gltf::BufferView::TargetType::ArrayBuffer;
      vtStream = str.slot;
      return str;
    }

    return Stream(vtStream);
  }

private:
  int32 ibmStream = -1;
  int32 indexStream = -1;
  int32 vtStream = -1;
};

static const es::Matrix44 corMat({0, 0, -1, 0}, {-1, 0, 0, 0}, {0, 1, 0, 0},
                                 {0, 0, 0, 1});

inline bool fltcmp(float f0, float f1, float epsilon = 0.0001) {
  return (f1 <= f0 + epsilon) && (f1 >= f0 - epsilon);
}
bool operator<(const Vector &v0, const Vector &v1) {
  if (v0.x == v1.x) {
    if (v0.y == v1.y) {
      return v0.z < v1.z;
    }
    return v0.y < v1.y;
  }
  return v0.x < v1.x;
}

void AppProcessFile(AppContext *ctx) {
  auto &istr = ctx->GetStream();
  char buffer[0x1000]{};

  auto GetToken = [&](bool ignoreNewLine = true) {
    uint32 bufferPos = 0;
    bool isString = false;
    while (!istr.eof()) {
      char c;
      istr.read(&c, 1);

      if (c == '"') {
        if (isString) {
          buffer[bufferPos] = 0;
          return std::string_view(buffer, bufferPos);
        }

        isString = !isString;
        continue;
      }

      if (isString) {
        buffer[bufferPos++] = c;
        continue;
      }

      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        if (bufferPos > 0) {
          buffer[bufferPos] = 0;
          return std::string_view(buffer, bufferPos);
        }
        if (!ignoreNewLine && c == '\n') {
          return std::string_view();
        }
        continue;
      }

      buffer[bufferPos++] = c;
    }

    if (bufferPos > 0) {
      buffer[bufferPos] = 0;
      return std::string_view(buffer, bufferPos);
    }

    return std::string_view();
  };

  auto GetInt = [&] {
    auto token = GetToken();
    return std::atol(token.data());
  };

  auto ExpectToken = [&](const char *what) {
    auto token = GetToken();
    if (token != what) {
      throw std::runtime_error("Expected " + std::string(what) +
                               ", got: " + std::string(token));
    }
  };

  auto GetGroup = [&] { ExpectToken("{"); };

  auto GetFloat = [&] {
    auto token = GetToken();
    return std::atof(token.data());
  };

  auto GetVector3 = [&] {
    ExpectToken("(");
    Vector retVal;
    retVal.x = GetFloat();
    retVal.y = GetFloat();
    retVal.z = GetFloat();
    ExpectToken(")");

    return retVal;
  };

  auto GetVector2 = [&] {
    ExpectToken("(");
    Vector2 retVal;
    retVal.x = GetFloat();
    retVal.y = GetFloat();
    ExpectToken(")");

    return retVal;
  };

  struct Bone {
    std::string name;
    int32 parentId;
    Vector position;
    Vector rotation;
    es::Matrix44 tm;
  };

  std::vector<Bone> bones;

  struct Vert {
    Vector2 uv;
    uint32 weightId;
    uint32 weightElem;
  };

  struct Weight {
    uint8 boneId;
    float weight;
    Vector position;
  };

  struct Mesh {
    std::vector<Vert> verts;
    std::vector<Weight> weights;
    std::vector<UIVector> tris;
    std::string shader;
  };

  std::vector<Mesh> meshes;

  while (!istr.eof()) {
    auto token = GetToken();

    if (token == "MD5Version") {
      if (auto version = GetInt(); version != 10) {
        throw es::InvalidVersionError(version);
      }
    } else if (token == "numJoints" || token == "numMeshes") {
      GetToken();
    } else if (token == "joints") {
      GetGroup();

      while (!istr.eof()) {
        auto boneName = GetToken();
        if (boneName == "}") {
          break;
        }

        Bone &nBone = bones.emplace_back();
        nBone.name = boneName;
        nBone.parentId = GetInt();
        nBone.position = GetVector3();
        nBone.rotation = GetVector3();
        ExpectToken("//");
        GetToken(false); // skip parent bone name
      }
    } else if (token == "commandline") {
      GetToken();
    } else if (token == "mesh") {
      GetGroup();
      auto &nMesh = meshes.emplace_back();

      while (!istr.eof()) {
        auto token = GetToken();
        if (token == "}") {
          break;
        } else if (token == "numverts") {
          nMesh.verts.resize(GetInt());
        } else if (token == "numtris") {
          nMesh.tris.resize(GetInt());
        } else if (token == "numweights") {
          nMesh.weights.resize(GetInt());
        } else if (token == "vert") {
          Vert &vert = nMesh.verts.at(GetInt());
          vert.uv = GetVector2();
          vert.weightId = GetInt();
          vert.weightElem = GetInt();
        } else if (token == "tri") {
          UIVector &tri = nMesh.tris.at(GetInt());
          tri.x = GetInt();
          tri.y = GetInt();
          tri.z = GetInt();
        } else if (token == "weight") {
          Weight &weight = nMesh.weights.at(GetInt());
          weight.boneId = GetInt();
          weight.weight = GetFloat();
          weight.position = GetVector3();
        } else if (token == "shader") {
          nMesh.shader = GetToken();
        } else if (token == "//") {
          while (!istr.eof()) {
            auto tkn = GetToken(false);
            if (tkn.empty()) {
              break;
            }
          }
        } else {
          throw std::runtime_error("Invalid mesh token: " + std::string(token));
        }
      }
    } else if (!token.empty()) {
      throw std::runtime_error("Invalid token: " + std::string(token));
    }
  }

  GLTFMain main;
  auto &gSkin = main.skins.emplace_back();
  auto &ibms = main.SkinStream();
  {
    auto [acc, accId] = main.NewAccessor(ibms, 16);
    acc.type = gltf::Accessor::Type::Mat4;
    acc.componentType = gltf::Accessor::ComponentType::Float;
    acc.count = bones.size();
    gSkin.inverseBindMatrices = accId;
  }

  for (auto &b : bones) {
    const size_t boneIndex = main.nodes.size();
    gSkin.joints.emplace_back(boneIndex);
    auto &gNode = main.nodes.emplace_back();
    gNode.name = b.name;
    Vector4A16 quat(b.rotation);
    quat.QComputeElement();
    Vector4A16 pos(b.position, 1);
    b.tm = quat;
    b.tm.r4() = pos;
    b.tm.Transpose();
    b.tm = corMat * b.tm;
    Vector4A16 dummy;

    if (b.parentId < 0) {
      b.tm.Decompose(pos, quat, dummy);
      main.scenes.front().nodes.emplace_back(boneIndex);
    } else {
      auto &parentBone = bones.at(b.parentId);
      es::Matrix44 localTm = -parentBone.tm * b.tm;
      localTm.Decompose(pos, quat, dummy);

      main.nodes.at(b.parentId).children.emplace_back(boneIndex);
    }

    memcpy(gNode.translation.data(), &pos, 12);
    memcpy(gNode.rotation.data(), &quat, 16);
    ibms.wr.Write(-b.tm);
  }

  main.scenes.front().nodes.emplace_back(main.nodes.size());
  auto &gNode = main.nodes.emplace_back();
  gNode.mesh = main.meshes.size();
  gNode.skin = 0;
  auto &prims = main.meshes.emplace_back().primitives;

  for (auto &m : meshes) {
    std::vector<Vertex> vertices;

    struct PositionEdges {
      std::vector<uint16> edges{};
      size_t index;
    };

    std::map<Vector, Vector> positionNormals;

    for (auto &t : m.tris) {
      static const uint32 fids[]{0, 2, 1};
      Vector4A16 positions[3];

      for (uint32 i = 0; i < 3; i++) {
        uint32 idx = t[fids[i]];
        auto &vert = vertices.emplace_back();
        Vert &mVert = m.verts.at(idx);
        vert.uv = mVert.uv;

        for (uint32 w = 0; w < mVert.weightElem; w++) {
          Weight &mWeight = m.weights.at(mVert.weightId + w);
          Bone &bone = bones.at(mWeight.boneId);
          vert.boneIds[w] = mWeight.boneId;
          vert.boneWeights[w] = std::round(mWeight.weight * 0xff);
          Vector4A16 tpos(Vector4A16(mWeight.position, 1) * bone.tm);
          Vector bPos(Vector4A16(tpos * mWeight.weight));
          vert.position += bPos;
        }

        positions[i] = vert.position;

        auto found = positionNormals.find(vert.position);

        if (found == positionNormals.end()) {
          found = positionNormals.emplace(vert.position, Vector{}).first;
        }
      }

      // Poor mans normal generator
      Vector4A16 u = positions[1] - positions[0];
      Vector4A16 v = positions[2] - positions[0];
      Vector4A16 normal = u.Cross(v);

      for (uint32 i = 0; i < 3; i++) {
        positionNormals.at(positions[i]) += normal;
      }
    }

    auto &prim = prims.emplace_back();
    prim.material = main.materials.size();
    main.materials.emplace_back().name = m.shader;

    auto &gVertices = main.GetVertexStream();

    {
      auto [acc, accId] = main.NewAccessor(gVertices, 4);
      acc.componentType = gltf::Accessor::ComponentType::Float;
      acc.type = gltf::Accessor::Type::Vec3;
      acc.count = vertices.size();
      prim.attributes["POSITION"] = accId;
    }
    {
      auto [acc, accId] = main.NewAccessor(gVertices, 4, 12);
      acc.componentType = gltf::Accessor::ComponentType::Float;
      acc.type = gltf::Accessor::Type::Vec2;
      acc.count = vertices.size();
      prim.attributes["TEXCOORD_0"] = accId;
    }
    {
      auto [acc, accId] = main.NewAccessor(gVertices, 4, 20);
      acc.componentType = gltf::Accessor::ComponentType::UnsignedByte;
      acc.type = gltf::Accessor::Type::Vec4;
      acc.count = vertices.size();
      prim.attributes["JOINTS_0"] = accId;
    }
    {
      auto [acc, accId] = main.NewAccessor(gVertices, 4, 24);
      acc.componentType = gltf::Accessor::ComponentType::UnsignedByte;
      acc.type = gltf::Accessor::Type::Vec4;
      acc.count = vertices.size();
      acc.normalized = true;
      prim.attributes["WEIGHTS_0"] = accId;
    }
    {
      auto [acc, accId] = main.NewAccessor(gVertices, 4, 28);
      acc.componentType = gltf::Accessor::ComponentType::Short;
      acc.type = gltf::Accessor::Type::Vec3;
      acc.count = vertices.size();
      acc.normalized = true;
      prim.attributes["NORMAL"] = accId;
    }

    for (auto &v : vertices) {
      Vector4A16 normal = positionNormals.at(v.position);
      normal.Normalize() *= 0x7fff;
      normal = Vector4A16(_mm_round_ps(normal._data, _MM_ROUND_NEAREST));
      v.normal = normal.Convert<int16>();
      gVertices.wr.Write(v);
    }
  }

  BinWritterRef wr(ctx->NewFile(ctx->workingFile.ChangeExtension(".glb")).str);
  main.extensionsRequired.emplace_back("KHR_mesh_quantization");
  main.extensionsUsed.emplace_back("KHR_mesh_quantization");
  main.FinishAndSave(wr, std::string(ctx->workingFile.GetFolder()));
}
