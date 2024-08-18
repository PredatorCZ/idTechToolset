/*  MD5Anim2GLTF
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
#include "spike/type/flags.hpp"

std::string_view filters[]{".md5anim$"};

std::string_view controlFilters[]{
    ".glb$",
    ".gltf$",
};

static AppInfo_s appInfo{
    .header = MD5Anim2GLTF_DESC " v" MD5Anim2GLTF_VERSION
                                ", " MD5Anim2GLTF_COPYRIGHT "Lukas Cone",
    .filters = filters,
    .batchControlFilters = controlFilters,
};

AppInfo_s *AppInitModule() { return &appInfo; }

struct GLTFAni : GLTF {
  using GLTF::GLTF;

  GLTFStream &AnimStream() {
    if (aniStream < 0) {
      auto &newStream = NewStream("anims");
      aniStream = newStream.slot;
      return newStream;
    }
    return Stream(aniStream);
  }

private:
  int32 aniStream = -1;
};

static const float IN_TO_M = 0.0254 * 0.9144; // inch to yard to meter
static const es::Matrix44 corMat({0, 0, 1, 0}, {1, 0, 0, 0}, {0, 1, 0, 0},
                                 {0, 0, 0, 1});

void LoadAnim(GLTFAni &main, std::istream &istr, std::string animName) {
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

  auto GetVector3Open = [&] {
    Vector retVal;
    retVal.x = GetFloat();
    retVal.y = GetFloat();
    retVal.z = GetFloat();
    ExpectToken(")");

    return retVal;
  };

  enum BoneFlag {
    Tx,
    Ty,
    Tz,
    Ri,
    Rj,
    Rk,
  };

  struct Bone {
    std::string name;
    int32 parentId;
    es::Flags<BoneFlag> flags;
    int32 startFrame;
    Vector pos;
    Vector rotation;

    std::vector<Vector> posTrack;
    std::vector<Vector> rotationTrack;
    std::vector<float> posFrames;
    std::vector<float> rotationFrames;
  };

  std::vector<Bone> bones;

  float fpsInv = 1.f / 30;

  while (!istr.eof()) {
    auto token = GetToken();

    if (token == "MD5Version") {
      if (auto version = GetInt(); version != 10) {
        throw es::InvalidVersionError(version);
      }
    } else if (token == "numFrames" || token == "numJoints" ||
               token == "numAnimatedComponents") {
      GetToken();
    } else if (token == "frameRate") {
      fpsInv = 1.f / GetInt();
    } else if (token == "hierarchy") {
      GetGroup();

      while (!istr.eof()) {
        auto boneName = GetToken();
        if (boneName == "}") {
          break;
        }

        Bone &nBone = bones.emplace_back();
        nBone.name = boneName;
        nBone.parentId = GetInt();
        nBone.flags = GetInt();
        nBone.startFrame = GetInt();
        ExpectToken("//");
        while (!istr.eof()) {
          if (GetToken(false).empty()) {
            break;
          }
        }
      }
    } else if (token == "commandline") {
      GetToken();
    } else if (token == "baseframe") {
      GetGroup();
      uint32 curBone = 0;

      while (!istr.eof()) {
        auto boneName = GetToken();
        if (boneName == "}") {
          break;
        }

        if (boneName != "(") {
          throw std::runtime_error("Expected (, got: " + std::string(token));
        }
        Bone &nBone = bones.at(curBone++);
        nBone.pos = GetVector3Open();
        nBone.rotation = GetVector3();
      }
    } else if (token == "frame") {
      int32 frameIndex = GetInt();
      GetGroup();
      std::vector<float> data;

      while (!istr.eof()) {
        auto boneName = GetToken();
        if (boneName == "}") {
          break;
        }

        data.emplace_back(atof(boneName.data()));
      }

      for (uint32 curValue = 0; auto &b : bones) {
        Vector pos(b.pos);
        Vector rot(b.rotation);

        if (b.flags == BoneFlag::Tx) {
          pos.x = data.at(curValue++);
        }
        if (b.flags == BoneFlag::Ty) {
          pos.y = data.at(curValue++);
        }
        if (b.flags == BoneFlag::Tz) {
          pos.z = data.at(curValue++);
        }
        if (b.flags == BoneFlag::Ri) {
          rot.x = data.at(curValue++);
        }
        if (b.flags == BoneFlag::Rj) {
          rot.y = data.at(curValue++);
        }
        if (b.flags == BoneFlag::Rk) {
          rot.z = data.at(curValue++);
        }

        if (pos != b.pos) {
          b.posTrack.emplace_back(pos);
          b.posFrames.emplace_back(frameIndex * fpsInv);
        }

        if (rot != b.rotation) {
          b.rotationTrack.emplace_back(rot);
          b.rotationFrames.emplace_back(frameIndex * fpsInv);
        }
      }

    } else if (token == "bounds") {
      while (!istr.eof()) {
        if (GetToken() == "}") {
          break;
        }
      }
    } else if (!token.empty()) {
      throw std::runtime_error("Invalid token: " + std::string(token));
    }
  }

  auto &stream = main.AnimStream();
  size_t nullInput = 0;
  {
    auto [acc, accid] = main.NewAccessor(stream, 4);
    acc.componentType = gltf::Accessor::ComponentType::Float;
    acc.type = gltf::Accessor::Type::Scalar;
    acc.count = 1;
    acc.min.emplace_back(0);
    acc.max.emplace_back(0);
    nullInput = accid;
    stream.wr.Write(0);
  }

  auto NewFrameAcc = [&](const std::vector<float> &frameTimes) {
    auto [acc, accid] = main.NewAccessor(stream, 4);
    acc.componentType = gltf::Accessor::ComponentType::Float;
    acc.type = gltf::Accessor::Type::Scalar;
    acc.count = frameTimes.size();
    acc.min.emplace_back(0);
    acc.max.emplace_back(frameTimes.back());

    if (frameTimes.front() != 0) {
      acc.count++;
      stream.wr.Write(0);
    }

    stream.wr.WriteContainer(frameTimes);
    return accid;
  };

  auto &anim = main.animations.emplace_back();
  anim.name = animName;

  for (auto &b : bones) {
    int32 foundNode = -1;
    for (int32 nodeIndex = 0; auto &n : main.nodes) {
      if (n.name == b.name) {
        foundNode = nodeIndex;
        break;
      }

      nodeIndex++;
    }

    {
      auto &chan = anim.channels.emplace_back();
      chan.target.path = "translation";
      chan.target.node = foundNode;
      chan.sampler = anim.samplers.size();

      auto &sampl = anim.samplers.emplace_back();
      sampl.input = b.posFrames.empty() ? nullInput : NewFrameAcc(b.posFrames);

      auto [acc, accid] = main.NewAccessor(stream, 4);
      acc.componentType = gltf::Accessor::ComponentType::Float;
      acc.type = gltf::Accessor::Type::Vec3;
      sampl.output = accid;

      auto WriteTranslation = [&](Vector pos_) {
        Vector4A16 pos(pos_ * IN_TO_M);

        if (b.parentId < 0) {
          pos = pos * corMat;
        }

        stream.wr.Write<Vector>(pos);
      };

      if (b.posTrack.empty()) {
        acc.count = 1;
        WriteTranslation(b.pos);
      } else {
        acc.count = b.posTrack.size();
        if (b.posFrames.front() != 0) {
          acc.count++;
          WriteTranslation(b.pos);
        }

        for (auto &p : b.posTrack) {
          WriteTranslation(p);
        }
      }
    }

    {
      auto &chan = anim.channels.emplace_back();
      chan.target.path = "rotation";
      chan.target.node = foundNode;
      chan.sampler = anim.samplers.size();

      auto &sampl = anim.samplers.emplace_back();
      sampl.input =
          b.rotationTrack.empty() ? nullInput : NewFrameAcc(b.rotationFrames);

      auto [acc, accid] = main.NewAccessor(stream, 4);
      acc.componentType = gltf::Accessor::ComponentType::Short;
      acc.type = gltf::Accessor::Type::Vec4;
      acc.normalized = true;
      sampl.output = accid;

      auto WriteRotatation = [&](Vector4A16 rot) {
        rot.QComputeElement();
        es::Matrix44 mtx(rot.Normalize());
        mtx.Transpose();

        if (b.parentId < 0) {
          mtx = corMat * mtx;
        }

        rot = mtx.ToQuat();

        rot *= 0x7fff;
        rot = Vector4A16(_mm_round_ps(rot._data, _MM_ROUND_NEAREST));
        stream.wr.Write(rot.Convert<int16>());
      };

      if (b.rotationTrack.empty()) {
        acc.count = 1;
        WriteRotatation(b.rotation);
      } else {
        acc.count = b.rotationTrack.size();
        if (b.rotationFrames.front() != 0) {
          acc.count++;
          WriteRotatation(b.rotation);
        }

        for (auto r : b.rotationTrack) {
          WriteRotatation(r);
        }
      }
    }
  }
}

void AppProcessFile(AppContext *ctx) {
  GLTFAni main(gltf::LoadFromBinary(ctx->GetStream(), ""));
  auto &anims = ctx->SupplementalFiles();

  for (auto &animFile : anims) {
    auto animStream = ctx->RequestFile(animFile);
    LoadAnim(main, *animStream.Get(),
             std::string(AFileInfo(animFile).GetFilename()));
  }

  BinWritterRef wr(
      ctx->NewFile(std::string(ctx->workingFile.GetFullPathNoExt()) +
                   "_out.glb")
          .str);
  main.FinishAndSave(wr, std::string(ctx->workingFile.GetFolder()));
}
