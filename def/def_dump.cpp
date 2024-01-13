/*  DEFDump
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
#include "spike/io/binreader_stream.hpp"
#include "spike/io/binwritter_stream.hpp"
#include <cmath>
#include <map>

std::string_view filters[]{".def$"};

static AppInfo_s appInfo{
    .header =
        DEFDump_DESC " v" DEFDump_VERSION ", " DEFDump_COPYRIGHT "Lukas Cone",
    .filters = filters,
};

AppInfo_s *AppInitModule() { return &appInfo; }

void AppProcessFile(AppContext *ctx) {
  auto &istr = ctx->GetStream();
  char buffer[0x1000]{};
  size_t curLine = 0;

  auto GetToken = [&](bool ignoreNewLine = true) {
    uint32 bufferPos = 0;
    bool isString = false;
    bool isComment = false;
    bool skipToNewLine = false;

    while (!istr.eof()) {
      char c;
      istr.read(&c, 1);

      if (skipToNewLine) {
        skipToNewLine = c != '\n';
        curLine += c == '\n';
        continue;
      }

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

      if (c == '/') {
        if (!isComment) {
          isComment = true;
        } else {
          if (bufferPos > 0) {
            bufferPos--;
          }
          skipToNewLine = true;
          isComment = false;
          continue;
        }
      } else if (isComment) {
        isComment = false;
      }

      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        if (bufferPos > 0) {
          buffer[bufferPos] = 0;
          return std::string_view(buffer, bufferPos);
        }
        if (c == '\n') {
          curLine++;
          if (!ignoreNewLine) {
            return std::string_view();
          }
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

  struct Event {
    int32 frame;
    std::string function;
    std::vector<std::string> params;
  };

  struct Animation {
    std::string name;
    std::string path;
    std::vector<Event> events;
  };

  struct Model {
    std::string meshPath;
    std::vector<Animation> anims;
  };

  std::vector<Model> models;

  while (!istr.eof()) {
    auto token = GetToken();

    if (token == "export" || token == "entityDef") {
      token = GetToken();
      GetGroup();
      while (!istr.eof()) {
        auto endGroup = GetToken();
        if (endGroup == "}") {
          break;
        }
      }
    } else if (token == "model") {
      auto &nModel = models.emplace_back();
      nModel.meshPath = GetToken();
      GetGroup();

      while (!istr.eof()) {
        auto token = GetToken();
        if (token == "}") {
          break;
        } else if (token == "mesh") {
          nModel.meshPath = GetToken();
        } else if (token == "anim") {
          auto &nAnim = nModel.anims.emplace_back();
          nAnim.name = GetToken();
          nAnim.path = GetToken();
          auto nextToken = GetToken(false);

          if (nextToken == "{") {
            while (!istr.eof()) {
              auto token = GetToken();
              if (token == "}") {
                break;
              } else if (token == "frame") {
                auto &nEvent = nAnim.events.emplace_back();
                nEvent.frame = GetInt();
                nEvent.function = GetToken();

                while (!istr.eof() && token.size() > 0) {
                  token = GetToken(false);
                  nEvent.params.emplace_back(token);
                }

              } else {
                throw std::runtime_error(
                    "line:" + std::to_string(curLine) +
                    " Invalid anim token: " + std::string(token));
              }
            }
          }
        } else {
          throw std::runtime_error(
              "line:" + std::to_string(curLine) +
              " Invalid mesh token: " + std::string(token));
        }
      }
    } else if (!token.empty()) {
      throw std::runtime_error("line:" + std::to_string(curLine) +
                               " Invalid model token: " + std::string(token));
    }
  }

  auto &ostr = ctx->NewFile(ctx->workingFile.ChangeExtension(".qcd")).str;

  for (auto &m : models) {
    for (auto &a : m.anims) {
      ostr << "// " << a.path << '\n';

      for (auto &e : a.events) {
        ostr << "{ event ";

        if (e.function.starts_with("sound")) {
          ostr << "5004";
        } else {
          ostr << e.function;
        }

        ostr << ' ' << std::round(((e.frame - 1) / 24.f) * 30) << ' ';

        for (auto &p : e.params) {
          ostr << p << ' ';
        }

        ostr << "}\n";
      }
    }
  }
}
