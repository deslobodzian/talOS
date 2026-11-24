// REQ-DEV-38: the generated MuJoCo model of the 2026 field (REBUILT) keeps
// the names and collision bits other simulators rely on, is symmetric, has
// the manual's dimensions, agrees with the official AprilTag poses, and makes
// fuel behave as on the field: at rest when staged, through the funnel into
// the hub, which keeps it, out of an exit when released there, off the roof
// without scoring, out of the chute when its door opens, and never through a
// wall at 10 m/s (README.md).

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <numbers>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"

namespace {

constexpr double kInch = 0.0254;
// The field (DWG FE-2026 sheet 3): 651.22 x 317.69 in, wall to wall and
// guardrail to guardrail; red is blue rotated about its centre.
constexpr double kFieldLengthIn = 651.22;
constexpr double kFieldWidthIn = 317.69;
constexpr double kCentreX = kFieldLengthIn * kInch / 2;
constexpr double kCentreY = kFieldWidthIn * kInch / 2;
constexpr int kFuelCount = 504;
constexpr double kFuelRadiusM = 2.955 * kInch;  // GM 5.10.1, 5.91 in
constexpr double kFuelMassKg = 0.215;           // middle of 0.203 to 0.227
constexpr double kMillimetre = 0.001;
// The blue hub (DWG s3, GM 5.4, GS p89, p111, p133): centre, box side, rim
// height and flat to flat, throat flat to flat, funnel panel slant.
constexpr double kHubXIn = 182.11;
constexpr double kHubYIn = 158.84;
constexpr double kHubSideIn = 47.00;
constexpr double kRimZIn = 72.00;
constexpr double kRimFlatsIn = 41.73;
constexpr double kThroatFlatsIn = 23.32;
constexpr double kFunnelSlantIn = 17.90;

struct ModelDeleter {
  void operator()(mjModel* m) const { mj_deleteModel(m); }
};
struct DataDeleter {
  void operator()(mjData* d) const { mj_deleteData(d); }
};
struct SpecDeleter {
  void operator()(mjSpec* s) const { mj_deleteSpec(s); }
};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;
using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

// The offset of row `index` in a MuJoCo array of rows `width` long.
constexpr std::ptrdiff_t Idx(int width, int index) {
  return static_cast<std::ptrdiff_t>(width) * index;
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

// A double in [lo, hi) from raw mt19937 output, whose sequence the standard
// fixes (its distributions differ between libstdc++ and libc++), so every
// host draws the same values.
double Uniform(std::mt19937& rng, double lo, double hi) {
  return lo + (hi - lo) * (static_cast<double>(rng()) / 4294967296.0);
}

// A point given in inches on the blue half, in metres on `side`'s half.
std::array<double, 2> OnSide(bool blue, double x_in, double y_in) {
  const double x = x_in * kInch;
  const double y = y_in * kInch;
  return blue ? std::array<double, 2>{x, y}
              : std::array<double, 2>{2 * kCentreX - x, 2 * kCentreY - y};
}

struct Box {
  std::array<double, 3> lo{std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity()};
  std::array<double, 3> hi{-std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()};
  bool Empty() const { return lo[0] > hi[0]; }
  void Add(const std::array<double, 3>& p) {
    for (int i = 0; i < 3; ++i) {
      lo[i] = std::min(lo[i], p[i]);
      hi[i] = std::max(hi[i], p[i]);
    }
  }
  void Add(const Box& b) {
    if (b.Empty()) return;
    Add(b.lo);
    Add(b.hi);
  }
};

// Expects `actual`'s x and y extent to be the blue box [x0, x1] x [y0, y1]
// inches, rotated to red for red, each edge to 1 mm.
void ExpectPlan(const Box& actual, bool blue, double x0, double x1, double y0,
                double y1) {
  const auto a = OnSide(blue, x0, y0);
  const auto b = OnSide(blue, x1, y1);
  EXPECT_NEAR(actual.lo[0], std::min(a[0], b[0]), kMillimetre);
  EXPECT_NEAR(actual.hi[0], std::max(a[0], b[0]), kMillimetre);
  EXPECT_NEAR(actual.lo[1], std::min(a[1], b[1]), kMillimetre);
  EXPECT_NEAR(actual.hi[1], std::max(a[1], b[1]), kMillimetre);
}

class FieldTest : public ::testing::Test {
 protected:
  // One compiled model for every suite; each test makes its own mjData,
  // and a test that changes the model works on its own copy.
  static void SetUpTestSuite() {
    if (Shared()) return;
    std::array<char, 1024> error{};
    Shared().reset(mj_loadXML(Runfile("FIELD_XML").c_str(), nullptr,
                              error.data(), static_cast<int>(error.size())));
    ASSERT_TRUE(Shared()) << error.data();
  }

  static ModelPtr& Shared() {
    static ModelPtr shared;
    return shared;
  }

  static const mjModel* model() { return Shared().get(); }

  // The id of a named element; a missing name fails the test (gtest reports
  // the exception) instead of indexing with -1.
  static int Id(const mjModel* m, mjtObj type, const std::string& name) {
    const int id = mj_name2id(m, type, name.c_str());
    if (id < 0) throw std::invalid_argument("no element named " + name);
    return id;
  }
  static std::string Name(const mjModel* m, mjtObj type, int id) {
    const char* name = mj_id2name(m, type, id);
    return name == nullptr ? "" : name;
  }

  static int Numeric(const mjModel* m, const std::string& name) {
    const int id = Id(m, mjOBJ_NUMERIC, name);
    return static_cast<int>(m->numeric_data[m->numeric_adr[id]]);
  }

  static std::string FuelName(int index) {
    std::array<char, 16> name{};
    std::snprintf(name.data(), name.size(), "fuel_%03d", index);
    return name.data();
  }
  static int FuelQpos(const mjModel* m, int index) {
    return m->jnt_qposadr[m->body_jntadr[Id(m, mjOBJ_BODY, FuelName(index))]];
  }
  static int FuelDof(const mjModel* m, int index) {
    return m->body_dofadr[Id(m, mjOBJ_BODY, FuelName(index))];
  }

  static std::vector<int> Range(const std::string& group) {
    std::vector<int> out;
    const int first = Numeric(model(), "fuel_" + group + "_first");
    const int count = Numeric(model(), "fuel_" + group + "_count");
    out.reserve(std::max(count, 0));
    for (int i = 0; i < count; ++i) out.push_back(first + i);
    return out;
  }

  // The field compiled without the fuel not in `keep`: the same structure,
  // fuel parameters and staging, only fewer bodies, so a test of a few balls
  // does not pay for 504. What it leaves out can only matter where removed
  // fuel would have been touched; each test says why that does not happen.
  // The fuel bodies are cut from the generated text (each is one
  // `<body name="fuel_NNN" ...>...</body>` with no child body), which is far
  // cheaper than deleting 500 bodies from an mjSpec one by one.
  static ModelPtr WithOnlyFuel(const std::vector<int>& keep) {
    static const std::string kXml = [] {
      std::ifstream file(Runfile("FIELD_XML"));
      return std::string(std::istreambuf_iterator<char>(file), {});
    }();
    std::string xml;
    xml.reserve(kXml.size());
    size_t at = 0;
    int removed = 0;
    for (size_t start = kXml.find("<body name=\"fuel_");
         start != std::string::npos;
         start = kXml.find("<body name=\"fuel_", at)) {
      const size_t end = kXml.find("</body>", start);
      if (end == std::string::npos) throw std::runtime_error("unclosed body");
      const int index = std::stoi(kXml.substr(start + 17, 3));
      xml.append(kXml, at, start - at);
      if (std::find(keep.begin(), keep.end(), index) != keep.end()) {
        xml.append(kXml, start, end + 7 - start);
      } else {
        ++removed;
      }
      at = end + 7;
    }
    xml.append(kXml, at, std::string::npos);
    if (removed + static_cast<int>(keep.size()) != kFuelCount) {
      throw std::runtime_error("fuel bodies not found as generated");
    }
    std::array<char, 1024> error{};
    SpecPtr spec(mj_parseXMLString(xml.c_str(), nullptr, error.data(),
                                   static_cast<int>(error.size())));
    if (!spec) throw std::runtime_error(error.data());
    ModelPtr m(mj_compile(spec.get(), nullptr));
    if (!m) throw std::runtime_error(mjs_getError(spec.get()));
    return m;
  }

  // The world-frame bounding box of a geom (box, mesh; a cylinder's box).
  static Box Bounds(const mjModel* m, const mjData* d, int geom) {
    Box box;
    const mjtNum* x = d->geom_xpos + Idx(3, geom);
    const mjtNum* r = d->geom_xmat + Idx(9, geom);
    auto add_local = [&](const double* p) {
      std::array<double, 3> w{};
      for (int i = 0; i < 3; ++i) {
        w[i] = x[i] + r[Idx(3, i)] * p[0] + r[Idx(3, i) + 1] * p[1] +
               r[Idx(3, i) + 2] * p[2];
      }
      box.Add(w);
    };
    if (m->geom_type[geom] == mjGEOM_MESH) {
      const int mesh = m->geom_dataid[geom];
      for (int v = 0; v < m->mesh_vertnum[mesh]; ++v) {
        const float* f = m->mesh_vert + Idx(3, m->mesh_vertadr[mesh] + v);
        const std::array<double, 3> p{f[0], f[1], f[2]};
        add_local(p.data());
      }
      return box;
    }
    const mjtNum* s = m->geom_size + Idx(3, geom);
    std::array<double, 3> half{s[0], s[1], s[2]};
    if (m->geom_type[geom] == mjGEOM_CYLINDER) half = {s[0], s[0], s[1]};
    for (int c = 0; c < 8; ++c) {
      const std::array<double, 3> p{(c & 1 ? 1 : -1) * half[0],
                                    (c & 2 ? 1 : -1) * half[1],
                                    (c & 4 ? 1 : -1) * half[2]};
      add_local(p.data());
    }
    return box;
  }

  // The bounding box of every geom whose name starts with `prefix`; a
  // prefix that names no geom fails the test.
  static Box BoundsOf(const mjModel* m, const mjData* d,
                      const std::string& prefix) {
    Box box;
    for (int g = 0; g < m->ngeom; ++g) {
      if (Name(m, mjOBJ_GEOM, g).rfind(prefix, 0) == 0) {
        box.Add(Bounds(m, d, g));
      }
    }
    if (box.Empty()) throw std::invalid_argument("no geom named " + prefix);
    return box;
  }

  // A site's world box (box or cylinder site, axis-aligned).
  static Box SiteBounds(const mjModel* m, const mjData* d, int site) {
    Box box;
    const mjtNum* x = d->site_xpos + Idx(3, site);
    const mjtNum* s = m->site_size + Idx(3, site);
    std::array<double, 3> half{s[0], s[1], s[2]};
    if (m->site_type[site] == mjGEOM_CYLINDER) half = {s[0], s[0], s[1]};
    box.Add(
        std::array<double, 3>{x[0] - half[0], x[1] - half[1], x[2] - half[2]});
    box.Add(
        std::array<double, 3>{x[0] + half[0], x[1] + half[1], x[2] + half[2]});
    return box;
  }

  static DataPtr Forward(const mjModel* m) {
    DataPtr d(mj_makeData(m));
    mj_forward(m, d.get());
    return d;
  }

  // The distance along a ray to the first structure geom, and that geom.
  static std::pair<double, std::string> Ray(const mjModel* m, const mjData* d,
                                            const std::array<double, 3>& from,
                                            const std::array<double, 3>& dir,
                                            std::array<double, 3>* normal) {
    static constexpr std::array<mjtByte, mjNGROUP> kStructure{1, 0, 0, 0, 0, 0};
    std::array<int, 1> hit{-1};
    std::array<double, 3> n{};
    const double distance =
        mj_ray(m, d, from.data(), dir.data(), kStructure.data(), 1, -1,
               hit.data(), n.data());
    if (normal != nullptr) *normal = n;
    return {distance, hit[0] < 0 ? "" : Name(m, mjOBJ_GEOM, hit[0])};
  }
};

// Counts and constants: 504 free fuel bodies of the manual's size and mass,
// named as simulators address them, and index ranges that tile them.
TEST_F(FieldTest, FuelBodiesAndRangesMatchTheContract) {
  const mjModel* m = model();
  int free_bodies = 0;
  for (int b = 1; b < m->nbody; ++b) {
    free_bodies +=
        m->body_jntnum[b] == 1 && m->jnt_type[m->body_jntadr[b]] == mjJNT_FREE;
  }
  EXPECT_EQ(free_bodies, kFuelCount);
  for (int i = 0; i < kFuelCount; ++i) {
    SCOPED_TRACE(FuelName(i));
    const int body = Id(m, mjOBJ_BODY, FuelName(i));
    const int joint = Id(m, mjOBJ_JOINT, FuelName(i) + "_joint");
    const int geom = Id(m, mjOBJ_GEOM, FuelName(i) + "_geom");
    EXPECT_EQ(m->jnt_bodyid[joint], body);
    EXPECT_EQ(m->jnt_type[joint], mjJNT_FREE);
    EXPECT_EQ(m->geom_bodyid[geom], body);
    EXPECT_EQ(m->body_geomnum[body], 1);
    EXPECT_EQ(m->geom_type[geom], mjGEOM_SPHERE);
    EXPECT_NEAR(m->geom_size[Idx(3, geom)], kFuelRadiusM, 1e-9);
    EXPECT_NEAR(m->body_mass[body], kFuelMassKg, 1e-9);
  }
  int next = 0;
  for (const char* group : {"neutral", "blue_depot", "red_depot", "blue_chute",
                            "red_chute", "reserve"}) {
    SCOPED_TRACE(group);
    EXPECT_EQ(Numeric(m, std::string("fuel_") + group + "_first"), next);
    next += Numeric(m, std::string("fuel_") + group + "_count");
  }
  EXPECT_EQ(next, kFuelCount);
  EXPECT_EQ(Numeric(m, "fuel_neutral_count"), 360);
  for (const char* group :
       {"blue_depot", "red_depot", "blue_chute", "red_chute"}) {
    EXPECT_EQ(Numeric(m, std::string("fuel_") + group + "_count"), 24) << group;
  }
}

// The collision contract a robot model follows (README.md): which geoms
// collide with which.
TEST_F(FieldTest, CollisionBitsFollowTheContract) {
  const mjModel* m = model();
  for (int g = 0; g < m->ngeom; ++g) {
    const std::string name = Name(m, mjOBJ_GEOM, g);
    SCOPED_TRACE(name);
    std::array<int, 2> expected{1, 0};  // field structure
    if (name.rfind("fuel_", 0) == 0) {
      expected = {2, 15};
    } else if (name == "carpet_fuel") {
      expected = {8, 0};
      EXPECT_EQ(m->geom_condim[g], 6);
      EXPECT_GT(m->geom_friction[Idx(3, g) + 2], 0);
    } else if (name == "carpet_robot") {
      expected = {16, 0};
      EXPECT_EQ(m->geom_condim[g], 3);
    } else if (name.rfind("tag_", 0) == 0) {
      expected = {0, 0};
    } else {
      EXPECT_TRUE(name.rfind("blue_", 0) == 0 || name.rfind("red_", 0) == 0 ||
                  name.rfind("guardrail_", 0) == 0);
    }
    EXPECT_EQ(m->geom_contype[g], expected[0]);
    EXPECT_EQ(m->geom_conaffinity[g], expected[1]);
  }
}

// Every blue geom and site has a red twin rotated 180 degrees about the
// field centre (position to 1e-9 m, orientation) with the same shape and
// contact parameters, and red's staged fuel is blue's rotated.
TEST_F(FieldTest, RedIsBlueRotatedAboutTheFieldCentre) {
  const mjModel* m = model();
  DataPtr d = Forward(m);
  const int centre_site = Id(m, mjOBJ_SITE, "field_centre");
  const double cx = d->site_xpos[Idx(3, centre_site)];
  const double cy = d->site_xpos[Idx(3, centre_site) + 1];
  EXPECT_NEAR(cx, kCentreX, 1e-9);
  EXPECT_NEAR(cy, kCentreY, 1e-9);
  auto rotated = [&](const mjtNum* blue, const mjtNum* red) {
    EXPECT_NEAR(red[0], 2 * cx - blue[0], 1e-9);
    EXPECT_NEAR(red[1], 2 * cy - blue[1], 1e-9);
    EXPECT_NEAR(red[2], blue[2], 1e-9);
  };
  auto same = [](const auto* blue, const auto* red, int count) {
    for (int k = 0; k < count; ++k) EXPECT_EQ(red[k], blue[k]) << k;
  };
  int pairs = 0;
  auto check = [&](mjtObj type, int count, const mjtNum* xpos,
                   const mjtNum* xmat) {
    for (int i = 0; i < count; ++i) {
      const std::string name = Name(m, type, i);
      const bool blue = name.rfind("blue_", 0) == 0;
      const std::string twin =
          blue ? "red_" + name.substr(5)
               : (name.rfind("red_", 0) == 0 ? "blue_" + name.substr(4) : "");
      if (twin.empty()) continue;
      SCOPED_TRACE(name);
      const int j = Id(m, type, twin);
      if (!blue) continue;
      ++pairs;
      rotated(xpos + Idx(3, i), xpos + Idx(3, j));
      // Rz(180) R: the first two rows change sign.
      for (int k = 0; k < 9; ++k) {
        const double sign = k < 6 ? -1 : 1;
        EXPECT_NEAR(xmat[Idx(9, j) + k], sign * xmat[Idx(9, i) + k], 1e-9) << k;
      }
      if (type == mjOBJ_GEOM) {
        EXPECT_EQ(m->geom_type[j], m->geom_type[i]);
        EXPECT_EQ(m->geom_dataid[j], m->geom_dataid[i]);
        same(m->geom_size + Idx(3, i), m->geom_size + Idx(3, j), 3);
        same(m->geom_friction + Idx(3, i), m->geom_friction + Idx(3, j), 3);
        same(m->geom_solref + Idx(mjNREF, i), m->geom_solref + Idx(mjNREF, j),
             mjNREF);
        same(m->geom_solimp + Idx(mjNIMP, i), m->geom_solimp + Idx(mjNIMP, j),
             mjNIMP);
        same(m->geom_solmix + i, m->geom_solmix + j, 1);
        same(m->geom_condim + i, m->geom_condim + j, 1);
        same(m->geom_margin + i, m->geom_margin + j, 1);
        same(m->geom_priority + i, m->geom_priority + j, 1);
      } else {
        same(m->site_size + Idx(3, i), m->site_size + Idx(3, j), 3);
      }
    }
  };
  check(mjOBJ_GEOM, static_cast<int>(m->ngeom), d->geom_xpos, d->geom_xmat);
  check(mjOBJ_SITE, static_cast<int>(m->nsite), d->site_xpos, d->site_xmat);
  EXPECT_GT(pairs, 60);
  for (const char* staged : {"depot", "chute"}) {
    const std::vector<int> blue = Range(std::string("blue_") + staged);
    const std::vector<int> red = Range(std::string("red_") + staged);
    ASSERT_EQ(blue.size(), red.size());
    for (size_t k = 0; k < blue.size(); ++k) {
      SCOPED_TRACE(std::string(staged) + " fuel " + std::to_string(k));
      rotated(m->qpos0 + FuelQpos(m, blue[k]), m->qpos0 + FuelQpos(m, red[k]));
    }
  }
}

// The official AprilTag poses, an independent source, sit where the model's
// elements say: hub tags within 15 mm outside a lower-box face and inside
// its extent; tower and outpost tags on their alliance wall, within the
// tower or outpost, the outpost's openings centred on its lower tag; trench
// tags above their arm.
TEST_F(FieldTest, OfficialTagsSitOnTheirElements) {
  std::ifstream file(Runfile("TAGS_JSON"));
  ASSERT_TRUE(file.good());
  const nlohmann::json json = nlohmann::json::parse(file);
  const mjModel* m = model();
  DataPtr d = Forward(m);
  constexpr double kOutsideM = 0.015;
  int checked = 0;
  for (const auto& tag : json.at("field-tags")) {
    const int id = tag.at("ID").get<int>();
    SCOPED_TRACE("tag " + std::to_string(id));
    const auto& t = tag.at("pose").at("translation");
    const auto& q = tag.at("pose").at("rotation").at("quaternion");
    const std::array<double, 3> p{t.at("x").get<double>(),
                                  t.at("y").get<double>(),
                                  t.at("z").get<double>()};
    const double w = q.at("W").get<double>();
    const double z = q.at("Z").get<double>();
    // The tag faces along its x axis; the poses are yaw-only.
    const std::array<double, 2> n{1 - 2 * z * z, 2 * w * z};
    const bool blue = p[0] < kCentreX;
    const std::string side = blue ? "blue_" : "red_";
    const bool hub = (id >= 2 && id <= 5) || (id >= 8 && id <= 11) ||
                     (id >= 18 && id <= 21) || (id >= 24 && id <= 27);
    const bool trench = id == 1 || id == 6 || id == 7 || id == 12 || id == 17 ||
                        id == 22 || id == 23 || id == 28;
    const bool tower = id == 15 || id == 16 || id == 31 || id == 32;
    const bool outpost = id == 13 || id == 14 || id == 29 || id == 30;
    const double wall_face = blue ? BoundsOf(m, d.get(), side + "wall_").hi[0]
                                  : BoundsOf(m, d.get(), side + "wall_").lo[0];
    if (hub) {
      const Box box = BoundsOf(m, d.get(), side + "hub_wall_");
      const int axis = std::abs(n[0]) > 0.5 ? 0 : 1;
      const double sign = n[axis] > 0 ? 1 : -1;
      const double face = sign > 0 ? box.hi[axis] : box.lo[axis];
      const double outside = sign * (p[axis] - face);
      EXPECT_GE(outside, 0);
      EXPECT_LE(outside, kOutsideM);
      const int along = 1 - axis;
      EXPECT_GT(p[along], box.lo[along]);
      EXPECT_LT(p[along], box.hi[along]);
      EXPECT_GT(p[2], box.lo[2]);
      EXPECT_LT(p[2], box.hi[2]);
    } else if (trench) {
      // An alliance's outpost-side trench is the one nearer its outpost.
      const bool outpost_side = (p[1] < kCentreY) == blue;
      const Box arm = BoundsOf(
          m, d.get(),
          side + "trench_" + (outpost_side ? "outpost" : "depot") + "_arm");
      EXPECT_GT(p[0], arm.lo[0]);
      EXPECT_LT(p[0], arm.hi[0]);
      EXPECT_GT(p[1], arm.lo[1]);
      EXPECT_LT(p[1], arm.hi[1]);
      EXPECT_GT(p[2], arm.hi[2]);
    } else if (tower || outpost) {
      EXPECT_LE(std::abs(p[0] - wall_face), kOutsideM);
      const Box element =
          BoundsOf(m, d.get(), side + (tower ? "wall_tower" : "wall_outpost"));
      EXPECT_GT(p[1], element.lo[1]);
      EXPECT_LT(p[1], element.hi[1]);
      if (id == 13 || id == 29) {
        // The chute and corral openings are centred on this tag.
        for (const char* opening : {"chute_door", "outpost_corral_pipe"}) {
          const Box box = BoundsOf(m, d.get(), side + opening);
          EXPECT_NEAR((box.lo[1] + box.hi[1]) / 2, p[1], kMillimetre)
              << opening;
        }
      }
    } else {
      continue;
    }
    ++checked;
  }
  EXPECT_EQ(checked, 32);
}

// Staged fuel starts at rest: nothing interpenetrates at t = 0, nothing is
// unsupported, and nothing slides.
TEST_F(FieldTest, StagedFuelIsAtRest) {
  const mjModel* m = model();
  DataPtr d(mj_makeData(m));
  std::vector<double> start(d->qpos, d->qpos + m->nq);
  const int steps = static_cast<int>(std::lround(2.0 / m->opt.timestep));
  for (int s = 0; s < steps; ++s) mj_step(m, d.get());
  for (int i = 0; i < kFuelCount; ++i) {
    const int q = FuelQpos(m, i);
    const int v = FuelDof(m, i);
    const double moved =
        std::hypot(d->qpos[q] - start[q], d->qpos[q + 1] - start[q + 1],
                   d->qpos[q + 2] - start[q + 2]);
    const double speed = std::hypot(d->qvel[v], d->qvel[v + 1], d->qvel[v + 2]);
    EXPECT_LT(moved, 5 * kMillimetre) << "fuel " << i;
    EXPECT_LT(speed, 0.05) << "fuel " << i;
  }
}

class FieldSideTest : public FieldTest,
                      public ::testing::WithParamInterface<std::string> {
 protected:
  bool blue() const { return GetParam() == "blue"; }
  // +1 when the neutral zone is at +x of this hub.
  double toward() const { return blue() ? 1 : -1; }
  std::array<double, 2> hub_centre() const {
    return OnSide(blue(), kHubXIn, kHubYIn);
  }
  double neutral_face() const {
    return hub_centre()[0] + toward() * kHubSideIn / 2 * kInch;
  }
};

// A hub keeps what it scores: 60 fuel released over 3 s (20 per second)
// from 0.3 m above the rim at seeded points over the opening. At least 57
// pass down through the scoring site; at this rate about 1 in 60 bounces off
// the crowded funnel and out, a miss as on a real field. Every fuel that
// passed stays in the bin from then on, through 3 s after the last release:
// inside the walls' inner faces and the pocket's back wall, above the
// carpet and below the throat plane. Every one that did not ends outside
// the hub.
//
// Fuel removed from this copy: the neutral block (from 2.1 m beyond the
// face), both depots and chutes, and the reserve not dropped; none is within
// a metre of the hub, where a miss lands.
TEST_P(FieldSideTest, HubKeepsWhatItScores) {
  const std::string& side = GetParam();
  constexpr int kBalls = 60;
  constexpr int kMinScored = 57;
  constexpr double kRate = 20.0;
  constexpr unsigned kSeed = 2026;
  std::vector<int> balls = Range("reserve");
  for (int i : Range("neutral")) {
    if (static_cast<int>(balls.size()) == kBalls) break;
    balls.push_back(i);
  }
  ModelPtr m = WithOnlyFuel(balls);
  DataPtr d = Forward(m.get());
  const int site = Id(m.get(), mjOBJ_SITE, side + "_hub_score");
  const mjtNum* score = d->site_xpos + Idx(3, site);
  const double score_radius = m->site_size[Idx(3, site)];
  const double score_half = m->site_size[Idx(3, site) + 1];
  const auto centre = hub_centre();
  const double half = kHubSideIn / 2 * kInch;
  const double drop_z = kRimZIn * kInch + 0.3;
  const double reach = kRimFlatsIn / 2 * kInch - kFuelRadiusM;
  std::mt19937 rng(kSeed);
  std::vector<std::array<double, 2>> drops;
  while (static_cast<int>(drops.size()) < kBalls) {
    const double x = Uniform(rng, -reach, reach);
    const double y = Uniform(rng, -reach, reach);
    if (std::hypot(x, y) <= reach) drops.push_back({x, y});
  }
  std::vector<int> qs;
  std::vector<int> vs;
  for (int b : balls) {
    qs.push_back(FuelQpos(m.get(), b));
    vs.push_back(FuelDof(m.get(), b));
  }
  // The bin's interior: inside the walls' inner faces, above the carpet,
  // below the throat plane, and out of the pockets (behind their back wall,
  // or above their roof).
  const Box alliance_wall =
      BoundsOf(m.get(), d.get(), side + "_hub_wall_alliance");
  const Box neutral_wall =
      BoundsOf(m.get(), d.get(), side + "_hub_wall_neutral");
  const Box pocket_back = BoundsOf(m.get(), d.get(), side + "_hub_pocket_back");
  const Box pocket_roof = BoundsOf(m.get(), d.get(), side + "_hub_pocket_roof");
  const Box outpost_wall =
      BoundsOf(m.get(), d.get(), side + "_hub_wall_outpost");
  const Box depot_wall = BoundsOf(m.get(), d.get(), side + "_hub_wall_depot");
  // Distances toward the neutral zone, from the alliance wall's inner face.
  const double start = blue() ? alliance_wall.hi[0] : alliance_wall.lo[0];
  auto along = [&](double x) { return toward() * (x - start); };
  const double neutral_inner =
      along(blue() ? neutral_wall.lo[0] : neutral_wall.hi[0]);
  const double pocket_start =
      along(blue() ? pocket_back.lo[0] : pocket_back.hi[0]);
  const double y0 = std::min(outpost_wall.hi[1], depot_wall.hi[1]);
  const double y1 = std::max(outpost_wall.lo[1], depot_wall.lo[1]);
  auto in_box = [&](const double* p) {
    const double u = along(p[0]);
    return u > 0 && u < neutral_inner && p[1] > y0 && p[1] < y1 && p[2] > 0 &&
           p[2] <= score[2] + score_half &&
           (u < pocket_start || p[2] > pocket_roof.hi[2]);
  };
  std::vector<bool> scored(kBalls, false);
  std::vector<bool> left(kBalls, false);
  int released = 0;
  double last_release = 0;
  while (released < kBalls || d->time < last_release + 3.0) {
    while (released < kBalls && d->time >= released / kRate - 1e-9) {
      // The same drops for both hubs, in the hub's own (rotated) frame.
      const std::array<double, 7> pose{
          centre[0] + toward() * drops[released][0],
          centre[1] + toward() * drops[released][1],
          drop_z,
          1,
          0,
          0,
          0};
      std::copy(pose.begin(), pose.end(), d->qpos + qs[released]);
      std::fill(d->qvel + vs[released], d->qvel + vs[released] + 6, 0.0);
      last_release = d->time;
      ++released;
    }
    mj_step(m.get(), d.get());
    for (int k = 0; k < released; ++k) {
      const double* p = d->qpos + qs[k];
      if (scored[k]) {
        left[k] = left[k] || !in_box(p);
      } else if (std::hypot(p[0] - score[0], p[1] - score[1]) < score_radius &&
                 std::abs(p[2] - score[2]) <= score_half &&
                 d->qvel[vs[k] + 2] < 0) {
        scored[k] = true;
      }
    }
  }
  int passed = 0;
  for (int k = 0; k < kBalls; ++k) {
    const double* p = d->qpos + qs[k];
    if (scored[k]) {
      ++passed;
      EXPECT_FALSE(left[k]) << side << " ball " << k << " left the bin";
    } else {
      EXPECT_TRUE(std::abs(p[0] - centre[0]) > half + kFuelRadiusM ||
                  std::abs(p[1] - centre[1]) > half + kFuelRadiusM)
          << side << " ball " << k << " missed but is at " << p[0] << ", "
          << p[1] << ", " << p[2];
    }
  }
  RecordProperty("passed_scoring_site", passed);
  EXPECT_GE(passed, kMinScored) << side << " (seed " << kSeed << ")";
}

// An exit releases into the neutral zone: a fuel placed at an exit site
// with 3.0 m/s along the site's +x axis comes to rest on the carpet more
// than 1.0 m beyond the neutral-zone face, and one placed there at rest
// rolls out, beyond the face by a ball diameter within 3 s.
//
// Every other fuel is removed from this copy; none is near a hub.
TEST_P(FieldSideTest, ExitReleasesFuelIntoTheNeutralZone) {
  const std::string& side = GetParam();
  const int ball = Range("reserve").front();
  ModelPtr m = WithOnlyFuel({ball});
  const int q = FuelQpos(m.get(), ball);
  const int v = FuelDof(m.get(), ball);
  const double face = neutral_face();
  for (int i = 0; i < 4; ++i) {
    for (const double speed : {3.0, 0.0}) {
      SCOPED_TRACE(side + " exit " + std::to_string(i) + " at " +
                   std::to_string(speed) + " m/s");
      DataPtr d = Forward(m.get());
      const int site =
          Id(m.get(), mjOBJ_SITE, side + "_hub_exit_" + std::to_string(i));
      const mjtNum* x = d->site_xpos + Idx(3, site);
      const mjtNum* r = d->site_xmat + Idx(9, site);
      const std::array<double, 7> pose{x[0], x[1], x[2], 1, 0, 0, 0};
      std::copy(pose.begin(), pose.end(), d->qpos + q);
      for (int k = 0; k < 3; ++k) d->qvel[v + k] = speed * r[Idx(3, k)];
      double cleared_at = -1;
      const double limit = speed > 0 ? 10.0 : 3.0;
      while (d->time < limit) {
        mj_step(m.get(), d.get());
        if (cleared_at < 0 &&
            toward() * (d->qpos[q] - face) > 2 * kFuelRadiusM) {
          cleared_at = d->time;
          if (speed == 0) break;
        }
      }
      const double beyond = toward() * (d->qpos[q] - face);
      if (speed > 0) {
        EXPECT_GT(beyond, 1.0);
        EXPECT_LT(std::hypot(d->qvel[v], d->qvel[v + 1], d->qvel[v + 2]), 0.01)
            << "still rolling";
        EXPECT_LT(d->qpos[q + 2], kFuelRadiusM + 5 * kMillimetre);
      } else {
        EXPECT_GE(cleared_at, 0) << "at " << beyond << " m beyond the face";
      }
    }
  }
}

// Releases that follow the contract (README.md) clear their exit: 40
// fuel released at one exit, one every 0.2 s whenever no fuel centre is
// within a ball diameter of the site, each at 3 to 5 m/s within 20 degrees
// of the site's +x axis (seeded, varied from release to release), are all
// released within 12 s, and 3 s after the last none is in the pocket or
// the bin.
//
// Fuel removed from this copy: everything but the 40 reserve fuel, which
// wait outside the field until released.
TEST_P(FieldSideTest, ContractReleasesClearTheirExit) {
  const std::string& side = GetParam();
  constexpr int kBalls = 40;
  constexpr double kInterval = 0.2;
  constexpr double kMinSpeed = 3.0;
  constexpr double kMaxSpeed = 5.0;
  constexpr double kMaxYaw = 20.0 * std::numbers::pi / 180;
  constexpr unsigned kSeed = 38;
  const std::vector<int> reserve = Range("reserve");
  const std::vector<int> balls(reserve.begin(), reserve.begin() + kBalls);
  ModelPtr m = WithOnlyFuel(balls);
  DataPtr d = Forward(m.get());
  const int site = Id(m.get(), mjOBJ_SITE, side + "_hub_exit_1");
  const std::array<double, 3> at{d->site_xpos[Idx(3, site)],
                                 d->site_xpos[Idx(3, site) + 1],
                                 d->site_xpos[Idx(3, site) + 2]};
  const mjtNum* axes = d->site_xmat + Idx(9, site);
  std::vector<int> qs;
  std::vector<int> vs;
  for (int b : balls) {
    qs.push_back(FuelQpos(m.get(), b));
    vs.push_back(FuelDof(m.get(), b));
  }
  std::mt19937 rng(kSeed);
  const double face = neutral_face();
  int released = 0;
  double last_release = 0;
  while ((released < kBalls && d->time < 12.0) ||
         (released == kBalls && d->time < last_release + 3.0)) {
    bool empty = true;
    for (int q : qs) {
      empty = empty && std::hypot(d->qpos[q] - at[0], d->qpos[q + 1] - at[1],
                                  d->qpos[q + 2] - at[2]) > 2 * kFuelRadiusM;
    }
    if (released < kBalls && empty && d->time >= released * kInterval - 1e-9) {
      const double speed = Uniform(rng, kMinSpeed, kMaxSpeed);
      const double yaw = Uniform(rng, -kMaxYaw, kMaxYaw);
      const std::array<double, 7> pose{at[0], at[1], at[2], 1, 0, 0, 0};
      std::copy(pose.begin(), pose.end(), d->qpos + qs[released]);
      std::fill(d->qvel + vs[released], d->qvel + vs[released] + 6, 0.0);
      for (int k = 0; k < 3; ++k) {
        // Along the site's x axis, turned by `yaw` about its z axis.
        d->qvel[vs[released] + k] =
            speed * (std::cos(yaw) * axes[Idx(3, k)] +
                     std::sin(yaw) * axes[Idx(3, k) + 1]);
      }
      last_release = d->time;
      ++released;
    }
    mj_step(m.get(), d.get());
  }
  RecordProperty("last_release_s", std::to_string(last_release));
  EXPECT_EQ(released, kBalls) << side << " (seed " << kSeed << ")";
  int inside = 0;
  for (int k = 0; k < released; ++k) {
    inside += toward() * (d->qpos[qs[k]] - face) <= 0;
  }
  EXPECT_EQ(inside, 0) << side << " (seed " << kSeed << ")";
}

// An exit does not lead into the bin: a fuel rolling at 3 m/s from the
// neutral zone straight into each exit pocket comes back out, and never
// gets past the pocket into the box.
//
// Every other fuel is removed from this copy; none is near a hub.
TEST_P(FieldSideTest, FuelRollingIntoAnExitStaysOutOfTheBin) {
  const std::string& side = GetParam();
  const int ball = Range("reserve").front();
  ModelPtr m = WithOnlyFuel({ball});
  const int q = FuelQpos(m.get(), ball);
  const int v = FuelDof(m.get(), ball);
  const double face = neutral_face();
  const double depth = 7.0 * kInch;  // the pocket, ESTIMATED in the README
  for (int i = 0; i < 4; ++i) {
    SCOPED_TRACE(side + " exit " + std::to_string(i));
    DataPtr d = Forward(m.get());
    const int site =
        Id(m.get(), mjOBJ_SITE, side + "_hub_exit_" + std::to_string(i));
    const double y = d->site_xpos[Idx(3, site) + 1];
    const std::array<double, 7> pose{
        face + toward() * 1.0, y, kFuelRadiusM, 1, 0, 0, 0};
    std::copy(pose.begin(), pose.end(), d->qpos + q);
    d->qvel[v] = -toward() * 3.0;
    d->qvel[v + 4] = -toward() * 3.0 / kFuelRadiusM;  // rolling, not sliding
    double deepest = 0;
    while (d->time < 3.0) {
      mj_step(m.get(), d.get());
      deepest = std::max(deepest, toward() * (face - d->qpos[q]));
    }
    EXPECT_GT(deepest, 0) << "never entered the pocket";
    EXPECT_LT(deepest, depth) << "reached past the pocket";
    EXPECT_GT(toward() * (d->qpos[q] - face), 0) << "ended inside the face";
  }
}

// The scoring site counts only fuel that went through the funnel: a ball
// dropped onto the roof outside the rim (over the box's corner) never
// enters the site's cylinder and ends on the carpet outside the hub.
//
// Every other fuel is removed from this copy; none is near a hub at start.
TEST_P(FieldSideTest, RoofFuelDoesNotScore) {
  const std::string& side = GetParam();
  const int ball = Range("reserve").front();
  ModelPtr m = WithOnlyFuel({ball});
  DataPtr d = Forward(m.get());
  const int site = Id(m.get(), mjOBJ_SITE, side + "_hub_score");
  const mjtNum* score = d->site_xpos + Idx(3, site);
  // 3.5 in in from the box's corner, outside the rim's hexagon.
  const double corner = (kHubSideIn / 2 - 3.5) * kInch;
  const auto centre = hub_centre();
  const std::array<double, 2> drop{centre[0] + toward() * corner,
                                   centre[1] + toward() * corner};
  ASSERT_GT(corner * (std::cos(std::numbers::pi / 6) +
                      std::sin(std::numbers::pi / 6)),
            kRimFlatsIn / 2 * kInch + kFuelRadiusM);
  const int q = FuelQpos(m.get(), ball);
  const std::array<double, 7> pose{
      drop[0], drop[1], kRimZIn * kInch + 0.3, 1, 0, 0, 0};
  std::copy(pose.begin(), pose.end(), d->qpos + q);
  bool entered = false;
  while (d->time < 3.0) {
    mj_step(m.get(), d.get());
    const double* p = d->qpos + q;
    entered = entered ||
              (std::hypot(p[0] - score[0], p[1] - score[1]) <
                   m->site_size[Idx(3, site)] &&
               std::abs(p[2] - score[2]) <= m->site_size[Idx(3, site) + 1]);
  }
  const double* p = d->qpos + q;
  const double half = kHubSideIn / 2 * kInch;
  EXPECT_FALSE(entered) << side;
  EXPECT_TRUE(std::abs(p[0] - centre[0]) > half + kFuelRadiusM ||
              std::abs(p[1] - centre[1]) > half + kFuelRadiusM)
      << side << " ball at " << p[0] << ", " << p[1];
  EXPECT_LT(p[2], kFuelRadiusM + 5 * kMillimetre) << side;
}

// The chute feeds: with its door's contype cleared, all 24 of its fuel
// reach the field side of the alliance wall within 1.5 s; down the 15 degree
// floor they take about 0.9 s, and the bound leaves margin over that.
//
// The other fuel is removed from this copy; none is near the outpost.
TEST_P(FieldSideTest, ChuteFeedsTheFieldWhenItsDoorOpens) {
  const std::string& side = GetParam();
  const std::vector<int> chute = Range(side + "_chute");
  ModelPtr m = WithOnlyFuel(chute);
  m->geom_contype[Id(m.get(), mjOBJ_GEOM, side + "_chute_door")] = 0;
  DataPtr d(mj_makeData(m.get()));
  const double face = blue() ? 0.0 : kFieldLengthIn * kInch;
  std::vector<int> qs;
  qs.reserve(chute.size());
  for (int i : chute) qs.push_back(FuelQpos(m.get(), i));
  int out = 0;
  while (out < 24 && d->time < 1.5) {
    mj_step(m.get(), d.get());
    out = 0;
    for (int q : qs) out += toward() * (d->qpos[q] - face) > 0;
  }
  EXPECT_EQ(out, 24) << side << " after " << d->time << " s";
}

INSTANTIATE_TEST_SUITE_P(Alliances, FieldSideTest,
                         ::testing::Values("blue", "red"));

// Nothing tunnels: a ball at 10 m/s, square to a guardrail, an alliance
// wall, the net, a hub wall, and straight down onto a funnel and a roof
// panel, never has its centre cross the plane of the surface it hits. Each
// launch is repeated at eight phases of the 5 ms step, so the ball's first
// contact ranges from touching to nearly a full step (5 cm) deep.
//
// Every other fuel is removed from this copy; none is on these paths.
TEST_F(FieldTest, FastFuelDoesNotTunnel) {
  struct Case {
    const char* geom;
    std::array<double, 3> aim;  // a point on the path, before the surface
    std::array<double, 3> direction;
  };
  const double hx = kHubXIn * kInch;
  const double hy = kHubYIn * kInch;
  const std::array<Case, 6> cases = {{
      {"guardrail_table", {7.0, 1.0, 0.25}, {0, -1, 0}},
      {"blue_wall_stations_depot", {1.0, 7.3, 1.0}, {-1, 0, 0}},
      {"blue_hub_net", {hx, hy, 2.4}, {1, 0, 0}},
      {"blue_hub_wall_alliance", {hx - 1.0, hy + 0.2, 0.6}, {1, 0, 0}},
      // Over the middle of the funnel panel on the +y flat.
      {"blue_hub_funnel_1", {hx, hy + 16.3 * kInch, 2.5}, {0, 0, -1}},
      // Over the box's +x +y corner, outside the funnel rim.
      {"blue_hub_roof_0", {hx + 20 * kInch, hy + 20 * kInch, 2.5}, {0, 0, -1}},
  }};
  constexpr double kSpeed = 10.0;
  constexpr int kPhases = 8;
  // Judged near the impact point only: a ball that rolls off a funnel
  // panel through the throat crosses that panel's plane, inside the hub.
  constexpr double kNearM = 0.15;
  const int ball = Range("reserve").front();
  ModelPtr m = WithOnlyFuel({ball});
  const double step = m->opt.timestep;
  const int q = FuelQpos(m.get(), ball);
  const int v = FuelDof(m.get(), ball);
  for (const Case& c : cases) {
    SCOPED_TRACE(c.geom);
    DataPtr d = Forward(m.get());
    std::array<double, 3> normal{};
    const auto [distance, hit] =
        Ray(m.get(), d.get(), c.aim, c.direction, &normal);
    ASSERT_GE(distance, 0);
    ASSERT_EQ(hit, c.geom);
    std::array<double, 3> surface{};
    for (int i = 0; i < 3; ++i) {
      surface[i] = c.aim[i] + distance * c.direction[i];
    }
    double closest = std::numeric_limits<double>::infinity();
    for (int phase = 0; phase < kPhases; ++phase) {
      mj_resetData(m.get(), d.get());
      // 0.3 m of flight, then a fraction of one step's travel.
      const double standoff =
          kFuelRadiusM + 0.3 + kSpeed * step * phase / kPhases;
      for (int i = 0; i < 3; ++i) {
        d->qpos[q + i] = surface[i] - standoff * c.direction[i];
        d->qvel[v + i] = kSpeed * c.direction[i];
      }
      while (d->time < 0.12) {
        mj_step(m.get(), d.get());
        double side = 0;
        double from_hit = 0;
        for (int i = 0; i < 3; ++i) {
          const double offset = d->qpos[q + i] - surface[i];
          side += offset * normal[i];
          from_hit += offset * offset;
        }
        if (from_hit - side * side < kNearM * kNearM) {
          closest = std::min(closest, side);
        }
      }
    }
    EXPECT_GT(closest, 0) << "centre crossed the surface by " << -closest
                          << " m";
    EXPECT_LT(closest, kFuelRadiusM) << "the ball never reached the surface";
  }
}

// The dimensions the manual and drawings give, read from the compiled model,
// each to 1 mm: field, guardrails, hub box, rim, funnel, scoring site and
// exits, net, bumps, trenches, depots and tower, both alliances.
TEST_F(FieldTest, GeometryMatchesTheManual) {
  const mjModel* m = model();
  DataPtr d = Forward(m);
  const Box table_rail = BoundsOf(m, d.get(), "guardrail_table");
  const Box far_rail = BoundsOf(m, d.get(), "guardrail_far");
  EXPECT_NEAR(table_rail.hi[2], 20.0 * kInch, kMillimetre);
  EXPECT_NEAR(far_rail.hi[2], 20.0 * kInch, kMillimetre);
  EXPECT_NEAR(table_rail.hi[1], 0, kMillimetre);
  EXPECT_NEAR(far_rail.lo[1], kFieldWidthIn * kInch, kMillimetre);
  EXPECT_NEAR(BoundsOf(m, d.get(), "blue_wall_").hi[0], 0, kMillimetre);
  EXPECT_NEAR(BoundsOf(m, d.get(), "red_wall_").lo[0], kFieldLengthIn * kInch,
              kMillimetre);
  // The throat plane, from the quoted rim, flats and slant.
  const double apothem_drop = (kRimFlatsIn - kThroatFlatsIn) / 2;
  const double throat_z = (kRimZIn - std::sqrt(kFunnelSlantIn * kFunnelSlantIn -
                                               apothem_drop * apothem_drop)) *
                          kInch;
  for (const bool blue : {true, false}) {
    const std::string side = blue ? "blue_" : "red_";
    SCOPED_TRACE(side);
    const double toward = blue ? 1 : -1;
    const auto hub = OnSide(blue, kHubXIn, kHubYIn);
    const double half = kHubSideIn / 2;
    // Hub box.
    const Box box = BoundsOf(m, d.get(), side + "hub_wall_");
    ExpectPlan(box, blue, kHubXIn - half, kHubXIn + half, kHubYIn - half,
               kHubYIn + half);
    // Rim: rays along +-y from the axis just below the rim meet the funnel
    // at the rim's flats; just above it they meet nothing of the funnel.
    double flats = 0;
    for (const double dir : {1.0, -1.0}) {
      const auto below =
          Ray(m, d.get(), {hub[0], hub[1], kRimZIn * kInch - 2e-4}, {0, dir, 0},
              nullptr);
      EXPECT_EQ(below.second.rfind(side + "hub_funnel_", 0), 0U)
          << below.second;
      flats += below.first;
      const auto above =
          Ray(m, d.get(), {hub[0], hub[1], kRimZIn * kInch + 1e-3}, {0, dir, 0},
              nullptr);
      EXPECT_EQ(above.second.find("hub_funnel_"), std::string::npos)
          << above.second;
    }
    EXPECT_NEAR(flats, kRimFlatsIn * kInch, kMillimetre);
    // Six funnel panels: a drop onto each panel's middle meets that panel,
    // which turns it toward the throat.
    for (int k = 0; k < 6; ++k) {
      const double a =
          (60 * k + 30 + (blue ? 0 : 180)) * std::numbers::pi / 180;
      const double r = (kRimFlatsIn + kThroatFlatsIn) / 4 * kInch;
      std::array<double, 3> normal{};
      const auto hit = Ray(
          m, d.get(), {hub[0] + r * std::cos(a), hub[1] + r * std::sin(a), 3.0},
          {0, 0, -1}, &normal);
      EXPECT_EQ(hit.second, side + "hub_funnel_" + std::to_string(k));
      EXPECT_LT(normal[0] * std::cos(a) + normal[1] * std::sin(a), 0) << k;
    }
    // Scoring site: on the axis, inside the throat, at or just below it.
    const int score = Id(m, mjOBJ_SITE, side + "hub_score");
    const mjtNum* s = d->site_xpos + Idx(3, score);
    EXPECT_NEAR(s[0], hub[0], kMillimetre);
    EXPECT_NEAR(s[1], hub[1], kMillimetre);
    EXPECT_LE(m->site_size[Idx(3, score)], kThroatFlatsIn / 2 * kInch + 1e-9);
    EXPECT_LE(s[2], throat_z + 1e-9);
    EXPECT_GE(s[2], throat_z - kFuelRadiusM - kMillimetre);
    EXPECT_LT(s[2] + m->site_size[Idx(3, score) + 1], kRimZIn * kInch);
    // Exits: each release site lies inside its pocket (between the pocket's
    // back wall and the face, between the lane's dividers or side walls),
    // with its +x axis pointing horizontally out of the hub.
    const double face = toward > 0 ? box.hi[0] : box.lo[0];
    const Box back = BoundsOf(m, d.get(), side + "hub_pocket_back");
    const double pocket_back = toward > 0 ? back.hi[0] : back.lo[0];
    std::vector<std::pair<double, double>> walls;
    for (const std::string& part :
         {std::string("hub_wall_outpost"), std::string("hub_wall_depot"),
          std::string("hub_pocket_divider_0"),
          std::string("hub_pocket_divider_1"),
          std::string("hub_pocket_divider_2")}) {
      const Box b = BoundsOf(m, d.get(), side + part);
      walls.emplace_back(b.lo[1], b.hi[1]);
    }
    std::sort(walls.begin(), walls.end());
    for (int i = 0; i < 4; ++i) {
      SCOPED_TRACE("exit " + std::to_string(i));
      const int site =
          Id(m, mjOBJ_SITE, side + "hub_exit_" + std::to_string(i));
      const mjtNum* x = d->site_xpos + Idx(3, site);
      const mjtNum* r = d->site_xmat + Idx(9, site);
      EXPECT_GT(toward * (x[0] - pocket_back), kFuelRadiusM);
      EXPECT_LT(toward * (x[0] - face), 0);
      // Exit i of blue is the i-th lane from -y; red's are rotated.
      const int lane = blue ? i : 3 - i;
      EXPECT_GT(x[1] - kFuelRadiusM, walls[lane].second);
      EXPECT_LT(x[1] + kFuelRadiusM, walls[lane + 1].first);
      EXPECT_LT(x[2] + kFuelRadiusM,
                BoundsOf(m, d.get(), side + "hub_pocket_roof").lo[2]);
      EXPECT_NEAR(r[0], toward, 1e-9);
      EXPECT_NEAR(r[3], 0, 1e-9);
      EXPECT_NEAR(r[6], 0, 1e-9);
    }
    // Net: 58.41 in wide, top 120.36 in, its hub-side face 10.26 in beyond
    // the neutral-zone face.
    const Box net = BoundsOf(m, d.get(), side + "hub_net");
    EXPECT_NEAR(net.hi[1] - net.lo[1], 58.41 * kInch, kMillimetre);
    EXPECT_NEAR(net.hi[2], 120.36 * kInch, kMillimetre);
    EXPECT_NEAR(toward > 0 ? net.lo[0] : net.hi[0],
                face + toward * 10.26 * kInch, kMillimetre);
    // Bumps: 44.40 in deep on the hub's x, 73.0 in long from 23.5 in off
    // the hub's centre line; the apex 6.513 in.
    const double bd = 44.40 / 2;
    ExpectPlan(BoundsOf(m, d.get(), side + "bump_outpost"), blue, kHubXIn - bd,
               kHubXIn + bd, kHubYIn - 96.5, kHubYIn - 23.5);
    ExpectPlan(BoundsOf(m, d.get(), side + "bump_depot"), blue, kHubXIn - bd,
               kHubXIn + bd, kHubYIn + 23.5, kHubYIn + 96.5);
    for (const char* bump : {"bump_outpost", "bump_depot"}) {
      EXPECT_NEAR(BoundsOf(m, d.get(), side + bump).hi[2], 6.513 * kInch,
                  kMillimetre)
          << bump;
    }
    // Trenches: the opening under each arm is 22.25 in high and 50.35 in
    // from the block to the nearer guardrail.
    for (const char* trench : {"outpost", "depot"}) {
      const std::string prefix = side + "trench_" + trench;
      SCOPED_TRACE(prefix);
      const Box arm = BoundsOf(m, d.get(), prefix + "_arm");
      const Box block = BoundsOf(m, d.get(), prefix + "_block");
      EXPECT_NEAR(arm.lo[2], 22.25 * kInch, kMillimetre);
      const double width = block.lo[1] < kCentreY
                               ? block.lo[1] - table_rail.hi[1]
                               : far_rail.lo[1] - block.hi[1];
      EXPECT_NEAR(width, 50.35 * kInch, kMillimetre);
    }
    // Depot: 42 x 27 in, open to the wall, centred 234.85 in from the
    // table guardrail for blue (82.84 in for red, by rotation).
    ExpectPlan(BoundsOf(m, d.get(), side + "depot_bar_"), blue, 0, 27.0,
               234.85 - 21.0, 234.85 + 21.0);
    // Tower: base plate 45.18 x 39.0 in, uprights 40.00 to 43.51 in from
    // the wall, 32.25 in apart, centred on 147.47 in; rungs at 27, 45, 63.
    constexpr double kTowerY = 147.47;
    ExpectPlan(BoundsOf(m, d.get(), side + "tower_base"), blue, 0, 45.18,
               kTowerY - 19.5, kTowerY + 19.5);
    ExpectPlan(BoundsOf(m, d.get(), side + "tower_upright_outpost"), blue,
               40.00, 43.51, kTowerY - 16.125 - 1.5, kTowerY - 16.125);
    ExpectPlan(BoundsOf(m, d.get(), side + "tower_upright_depot"), blue, 40.00,
               43.51, kTowerY + 16.125, kTowerY + 16.125 + 1.5);
    const std::array<std::pair<const char*, double>, 3> rungs = {
        {{"low", 27.0}, {"mid", 45.0}, {"high", 63.0}}};
    for (const auto& [rung, height] : rungs) {
      const int g = Id(m, mjOBJ_GEOM, side + "tower_rung_" + rung);
      EXPECT_NEAR(d->geom_xpos[Idx(3, g) + 2], height * kInch, kMillimetre)
          << rung;
    }
  }
}

}  // namespace
