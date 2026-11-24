// REQ-DEV-42: the glTF binary the field generator writes is a valid,
// self-contained GLB of closed, outward-facing primitives, and every geom of
// the MuJoCo field model but the fuel and the carpet planes (a viewer draws
// its own floor) is a node of it with the geom's pose, size and colour, once
// Studio's viewport has turned the Y-up scene to its Z-up frame.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"

namespace {

using Vec = std::array<double, 3>;
using Mat = std::array<double, 9>;  // row-major 3x3

// A rigid transform with a linear part that may scale: p -> L p + t.
struct Affine {
  Mat linear{1, 0, 0, 0, 1, 0, 0, 0, 1};
  Vec translation{0, 0, 0};

  Vec Apply(const Vec& p) const {
    Vec out{};
    for (size_t i = 0; i < 3; ++i) {
      out[i] = translation[i] + linear[3 * i] * p[0] +
               linear[3 * i + 1] * p[1] + linear[3 * i + 2] * p[2];
    }
    return out;
  }
  Affine Then(const Affine& child) const {  // this * child
    Affine out;
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        double sum = 0;
        for (size_t k = 0; k < 3; ++k) {
          sum += linear[3 * i + k] * child.linear[3 * k + j];
        }
        out.linear[3 * i + j] = sum;
      }
    }
    out.translation = Apply(child.translation);
    return out;
  }
};

// Studio's viewport loads the GLB under `scene.rotation.x = Math.PI / 2`
// (loadFieldModel, studio/src/panels/pose3d/field_model.ts): +90 degrees
// about x. //studio:field_glb_test loads the built GLB through that function
// and holds the panel to the same frame.
const Affine kStudioRotation{{1, 0, 0, 0, 0, -1, 0, 1, 0}, {0, 0, 0}};

// A glTF node's local transform: translation, rotation (x, y, z, w), scale.
Affine NodeTransform(const nlohmann::json& node) {
  Affine a;
  const std::vector<double> t =
      node.value("translation", std::vector<double>{0, 0, 0});
  const std::vector<double> q =
      node.value("rotation", std::vector<double>{0, 0, 0, 1});
  const std::vector<double> s =
      node.value("scale", std::vector<double>{1, 1, 1});
  const double x = q[0];
  const double y = q[1];
  const double z = q[2];
  const double w = q[3];
  const Mat r{1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
              2 * (x * z + y * w),     2 * (x * y + z * w),
              1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
              2 * (x * z - y * w),     2 * (y * z + x * w),
              1 - 2 * (x * x + y * y)};
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) a.linear[3 * i + j] = r[3 * i + j] * s[j];
    a.translation[i] = t[i];
  }
  return a;
}

std::string Runfile(const char* variable) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* path = std::getenv(variable);
  if (!runfiles || path == nullptr) {
    throw std::runtime_error("runfiles: " + error + ", " + variable + " unset");
  }
  return runfiles->Rlocation(path);
}

uint32_t ReadU32(const std::string& bytes, size_t at) {
  uint32_t value = 0;
  std::memcpy(&value, bytes.data() + at, sizeof(value));
  return value;
}

// The GLB's JSON chunk and BIN chunk; throws on a malformed container.
struct Glb {
  nlohmann::json json;
  std::string bin;
};

Glb ReadGlb(const std::string& bytes) {
  if (bytes.size() < 28) throw std::runtime_error("shorter than a GLB");
  if (bytes.compare(0, 4, "glTF") != 0) throw std::runtime_error("magic");
  if (ReadU32(bytes, 4) != 2) throw std::runtime_error("not version 2");
  if (ReadU32(bytes, 8) != bytes.size()) throw std::runtime_error("length");
  const uint32_t json_length = ReadU32(bytes, 12);
  if (bytes.compare(16, 4, "JSON") != 0 || json_length % 4 != 0 ||
      20 + json_length + 8 > bytes.size()) {
    throw std::runtime_error("JSON chunk");
  }
  const size_t bin_at = 20 + json_length;
  const uint32_t bin_length = ReadU32(bytes, bin_at);
  if (bytes.compare(bin_at + 4, 4, std::string("BIN\0", 4)) != 0 ||
      bin_length % 4 != 0 || bin_at + 8 + bin_length != bytes.size()) {
    throw std::runtime_error("BIN chunk");
  }
  return {nlohmann::json::parse(bytes.substr(20, json_length)),
          bytes.substr(bin_at + 8, bin_length)};
}

class FieldGltfTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    std::ifstream file(Runfile("FIELD_GLB"), std::ios::binary);
    Bytes() = std::string(std::istreambuf_iterator<char>(file), {});
    std::array<char, 1024> error{};
    Model().reset(mj_loadXML(Runfile("FIELD_XML").c_str(), nullptr,
                             error.data(), static_cast<int>(error.size())));
    ASSERT_TRUE(Model()) << error.data();
  }
  static std::string& Bytes() {
    static std::string bytes;
    return bytes;
  }
  struct ModelDeleter {
    void operator()(mjModel* m) const { mj_deleteModel(m); }
  };
  static std::unique_ptr<mjModel, ModelDeleter>& Model() {
    static std::unique_ptr<mjModel, ModelDeleter> model;
    return model;
  }

  // An accessor's floats, checked to lie inside its buffer view.
  static std::vector<float> Floats(const Glb& glb, int accessor) {
    const auto& a = glb.json.at("accessors").at(accessor);
    const auto& v =
        glb.json.at("bufferViews").at(a.at("bufferView").get<int>());
    const int width = a.at("type") == "VEC3" ? 3 : 1;
    const size_t count = a.at("count").get<size_t>() * width;
    const size_t offset = v.value("byteOffset", 0) + a.value("byteOffset", 0);
    if (a.at("componentType") != 5126 || a.value("byteOffset", 0) + count * 4 >
                                             v.at("byteLength").get<size_t>()) {
      throw std::runtime_error("accessor outside its view");
    }
    std::vector<float> out(count);
    std::memcpy(out.data(), glb.bin.data() + offset, count * 4);
    return out;
  }
};

// A valid, self-contained GLB: header, chunks and 4-byte alignment; one
// buffer, the BIN chunk, with no URI anywhere; every buffer view inside it;
// every position accessor with bounds that hold; every index in range.
TEST_F(FieldGltfTest, GlbIsValidAndSelfContained) {
  const Glb glb = ReadGlb(Bytes());
  const auto& j = glb.json;
  EXPECT_EQ(j.at("asset").at("version"), "2.0");
  ASSERT_EQ(j.at("buffers").size(), 1U);
  EXPECT_FALSE(j.at("buffers")[0].contains("uri"));
  EXPECT_FALSE(j.contains("images"));
  EXPECT_LE(j.at("buffers")[0].at("byteLength").get<size_t>(), glb.bin.size());
  for (const auto& v : j.at("bufferViews")) {
    EXPECT_EQ(v.at("buffer"), 0);
    EXPECT_EQ(v.value("byteOffset", 0) % 4, 0);
    EXPECT_LE(v.value("byteOffset", 0) + v.at("byteLength").get<size_t>(),
              glb.bin.size());
  }
  for (const auto& mesh : j.at("meshes")) {
    for (const auto& p : mesh.at("primitives")) {
      const int position = p.at("attributes").at("POSITION");
      const auto& a = j.at("accessors").at(position);
      ASSERT_TRUE(a.contains("min") && a.contains("max"));
      const std::vector<float> values = Floats(glb, position);
      for (size_t i = 0; i < values.size(); ++i) {
        EXPECT_GE(values[i], a.at("min")[i % 3].get<float>());
        EXPECT_LE(values[i], a.at("max")[i % 3].get<float>());
      }
      const auto& ia = j.at("accessors").at(p.at("indices").get<int>());
      const auto& iv = j.at("bufferViews").at(ia.at("bufferView").get<int>());
      ASSERT_EQ(ia.at("componentType"), 5123);
      ASSERT_LE(ia.at("count").get<size_t>() * 2,
                iv.at("byteLength").get<size_t>());
      std::vector<uint16_t> indices(ia.at("count").get<size_t>());
      std::memcpy(indices.data(), glb.bin.data() + iv.value("byteOffset", 0),
                  indices.size() * 2);
      for (const uint16_t index : indices) {
        EXPECT_LT(index, a.at("count").get<size_t>()) << mesh.at("name");
      }
    }
  }
}

// Every geom of the compiled field model but the fuel and the carpet planes
// is exactly one node, and nothing else is: in the field frame
// (Studio's rotation applied) its translation is the geom's position, its
// linear part the geom's rotation times its size on the unit primitive
// (a mesh: the same vertices in the world), and its colour the geom's rgba
// (sRGB, as MuJoCo shows it) as glTF's linear colour.
// The comparison is made in the field frame after Studio's rotation, so it
// also proves the root node's frame change.
TEST_F(FieldGltfTest, EveryGeomIsANodeWithItsPoseSizeAndColour) {
  const Glb glb = ReadGlb(Bytes());
  const auto& j = glb.json;
  const mjModel* m = Model().get();
  std::unique_ptr<mjData, void (*)(mjData*)> d(mj_makeData(m), mj_deleteData);
  mj_forward(m, d.get());
  // World transforms of the scene's nodes, under Studio's rotation.
  std::map<std::string, std::pair<Affine, nlohmann::json>> world;
  std::vector<std::pair<int, Affine>> stack;
  for (const auto& root :
       j.at("scenes").at(j.at("scene").get<int>()).at("nodes")) {
    stack.emplace_back(root.get<int>(), kStudioRotation);
  }
  while (!stack.empty()) {
    const auto [index, parent] = stack.back();
    stack.pop_back();
    const auto& node = j.at("nodes").at(index);
    const Affine here = parent.Then(NodeTransform(node));
    for (const auto& child : node.value("children", nlohmann::json::array())) {
      stack.emplace_back(child.get<int>(), here);
    }
    if (node.contains("mesh")) {
      const std::string name = node.at("name");
      EXPECT_EQ(world.count(name), 0U) << "two nodes named " << name;
      world[name] = {here, node};
    }
  }
  constexpr double kPose = 1e-6;  // metres, and for unit rotation entries
  constexpr double kMesh = 1e-5;  // metres; MuJoCo keeps vertices as float
  int drawn = 0;
  for (int g = 0; g < m->ngeom; ++g) {
    const char* raw = mj_id2name(m, mjOBJ_GEOM, g);
    const std::string name = raw == nullptr ? "" : raw;
    if (name.rfind("fuel_", 0) == 0 || m->geom_type[g] == mjGEOM_PLANE) {
      continue;
    }
    SCOPED_TRACE(name);
    ++drawn;
    const auto found = world.find(name);
    ASSERT_NE(found, world.end()) << "no node";
    const Affine& a = found->second.first;
    const auto& node = found->second.second;
    const auto& primitive =
        j.at("meshes").at(node.at("mesh").get<int>()).at("primitives")[0];
    const mjtNum* xpos = d->geom_xpos + 3 * static_cast<ptrdiff_t>(g);
    const mjtNum* xmat = d->geom_xmat + 9 * static_cast<ptrdiff_t>(g);
    const mjtNum* size = m->geom_size + 3 * static_cast<ptrdiff_t>(g);
    if (m->geom_type[g] == mjGEOM_MESH) {
      // The node's vertices in the world against MuJoCo's, both ways.
      const std::vector<float> local =
          Floats(glb, primitive.at("attributes").at("POSITION"));
      std::vector<Vec> ours;
      for (size_t i = 0; i < local.size(); i += 3) {
        ours.push_back(a.Apply({local[i], local[i + 1], local[i + 2]}));
      }
      const int mesh = m->geom_dataid[g];
      std::vector<Vec> theirs;
      for (int v = 0; v < m->mesh_vertnum[mesh]; ++v) {
        const float* f = m->mesh_vert +
                         3 * static_cast<ptrdiff_t>(m->mesh_vertadr[mesh] + v);
        Vec w{};
        for (size_t i = 0; i < 3; ++i) {
          w[i] = xpos[i] + xmat[3 * i] * f[0] + xmat[3 * i + 1] * f[1] +
                 xmat[3 * i + 2] * f[2];
        }
        theirs.push_back(w);
      }
      auto near = [](const Vec& p, const std::vector<Vec>& set) {
        double best = 1e9;
        for (const Vec& q : set) {
          best =
              std::min(best, std::hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]));
        }
        return best;
      };
      for (const Vec& p : theirs) EXPECT_LT(near(p, ours), kMesh);
      for (const Vec& p : ours) EXPECT_LT(near(p, theirs), kMesh);
    } else {
      // The unit primitive spans -1..1, so the node's scale is MuJoCo's
      // half size.
      const auto& accessor =
          j.at("accessors")
              .at(primitive.at("attributes").at("POSITION").get<int>());
      for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(accessor.at("min")[i].get<double>(), -1, 1e-12);
        EXPECT_NEAR(accessor.at("max")[i].get<double>(), 1, 1e-12);
      }
      Vec half{size[0], size[1], size[2]};
      if (m->geom_type[g] == mjGEOM_CYLINDER) {
        half = {size[0], size[0], size[1]};
        // MuJoCo's cylinder axis is its local z: the unit cylinder lies
        // within radius 1 of it.
        const std::vector<float> local =
            Floats(glb, primitive.at("attributes").at("POSITION"));
        for (size_t i = 0; i < local.size(); i += 3) {
          EXPECT_LE(std::hypot(local[i], local[i + 1]), 1 + 1e-6);
        }
      } else {
        ASSERT_EQ(m->geom_type[g], mjGEOM_BOX);
      }
      for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(a.translation[i], xpos[i], kPose) << i;
        for (size_t k = 0; k < 3; ++k) {
          EXPECT_NEAR(a.linear[3 * i + k], xmat[3 * i + k] * half[k], kPose)
              << i << ", " << k;
        }
      }
    }
    const auto& colour = j.at("materials")
                             .at(primitive.at("material").get<int>())
                             .at("pbrMetallicRoughness")
                             .at("baseColorFactor");
    for (int i = 0; i < 4; ++i) {
      const double c = m->geom_rgba[4 * static_cast<ptrdiff_t>(g) + i];
      // The sRGB transfer function's inverse (IEC 61966-2-1); alpha as is.
      const double expected =
          i == 3
              ? c
              : (c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4));
      EXPECT_NEAR(colour[i].get<double>(), expected, 1e-6) << i;
    }
  }
  EXPECT_EQ(static_cast<int>(world.size()), drawn) << "extra nodes";
}

// Every primitive is a closed surface facing outward, as a viewer that
// culls back faces needs: each edge (by position, since flat faces repeat
// their corners) is shared by exactly two triangles, the enclosed signed
// volume is positive, normals are unit length, and each triangle's face
// normal agrees with its corners' normals.
TEST_F(FieldGltfTest, PrimitivesAreClosedAndFaceOutward) {
  const Glb glb = ReadGlb(Bytes());
  const auto& j = glb.json;
  for (const auto& mesh : j.at("meshes")) {
    for (const auto& p : mesh.at("primitives")) {
      SCOPED_TRACE(mesh.at("name").get<std::string>());
      EXPECT_FALSE(j.at("materials")
                       .at(p.at("material").get<int>())
                       .value("doubleSided", false));
      const std::vector<float> position =
          Floats(glb, p.at("attributes").at("POSITION"));
      const std::vector<float> normal =
          Floats(glb, p.at("attributes").at("NORMAL"));
      ASSERT_EQ(position.size(), normal.size());
      const auto& ia = j.at("accessors").at(p.at("indices").get<int>());
      const auto& iv = j.at("bufferViews").at(ia.at("bufferView").get<int>());
      std::vector<uint16_t> index(ia.at("count").get<size_t>());
      ASSERT_EQ(index.size() % 3, 0U);
      std::memcpy(index.data(), glb.bin.data() + iv.value("byteOffset", 0),
                  index.size() * 2);
      auto at = [&](const std::vector<float>& v, uint16_t i) {
        return Vec{v[3 * size_t{i}], v[3 * size_t{i} + 1],
                   v[3 * size_t{i} + 2]};
      };
      // A corner's key: its position to a micrometre.
      auto key = [&](uint16_t i) {
        const Vec q = at(position, i);
        return std::array<long long, 3>{std::llround(q[0] * 1e6),
                                        std::llround(q[1] * 1e6),
                                        std::llround(q[2] * 1e6)};
      };
      std::map<std::pair<std::array<long long, 3>, std::array<long long, 3>>,
               int>
          directed;
      double volume = 0;
      for (size_t t = 0; t < index.size(); t += 3) {
        const std::array<uint16_t, 3> c{index[t], index[t + 1], index[t + 2]};
        const Vec a = at(position, c[0]);
        const Vec b = at(position, c[1]);
        const Vec d = at(position, c[2]);
        const Vec e1{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const Vec e2{d[0] - a[0], d[1] - a[1], d[2] - a[2]};
        const Vec face{e1[1] * e2[2] - e1[2] * e2[1],
                       e1[2] * e2[0] - e1[0] * e2[2],
                       e1[0] * e2[1] - e1[1] * e2[0]};
        volume += (a[0] * (b[1] * d[2] - b[2] * d[1]) +
                   a[1] * (b[2] * d[0] - b[0] * d[2]) +
                   a[2] * (b[0] * d[1] - b[1] * d[0])) /
                  6;
        for (int k = 0; k < 3; ++k) {
          const Vec n = at(normal, c[k]);
          EXPECT_NEAR(std::hypot(n[0], n[1], n[2]), 1, 1e-6);
          EXPECT_GT(face[0] * n[0] + face[1] * n[1] + face[2] * n[2], 0)
              << "triangle " << t / 3 << " faces away from its normals";
          ++directed[{key(c[k]), key(c[(k + 1) % 3])}];
        }
      }
      EXPECT_GT(volume, 0) << "wound inward";
      // Closed and consistently wound: each directed edge once, and its
      // reverse once.
      for (const auto& [edge, count] : directed) {
        EXPECT_EQ(count, 1) << "an edge used twice the same way";
        const auto reverse = directed.find({edge.second, edge.first});
        EXPECT_TRUE(reverse != directed.end() && reverse->second == 1)
            << "an open edge";
      }
    }
  }
}

}  // namespace
