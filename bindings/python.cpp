#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>

#include "justblend/justblend.hpp"

namespace py = pybind11;
using namespace justblend;

PYBIND11_MODULE(_core, m)
{
    m.doc() = "justblend: closed-form N-dimensional trajectory generator (C++ core)";

    py::register_exception<ValidationError>(m, "ValidationError", PyExc_ValueError);

    py::enum_<BlendShape>(m, "BlendShape")
        .value("PARABOLIC", BlendShape::Parabolic)
        .value("HERMITE", BlendShape::Hermite);

    py::enum_<CornerHandling>(m, "CornerHandling")
        .value("STRICT_CORNERS", CornerHandling::StrictCorners)
        .value("USE_BLENDING", CornerHandling::UseBlending)
        .value("HYBRID", CornerHandling::Hybrid);

    py::enum_<CornerType>(m, "CornerType")
        .value("ENDPOINT", CornerType::Endpoint)
        .value("PASS", CornerType::Pass)
        .value("STOP", CornerType::Stop)
        .value("BLEND", CornerType::Blend);

    py::enum_<SegmentType>(m, "SegmentType").value("LINEAR", SegmentType::Linear).value("BLEND", SegmentType::Blend);

    py::class_<Limits>(m, "Limits")
        .def(
            py::init(
                [](const Eigen::VectorXd& v_max, const Eigen::VectorXd& a_max, std::optional<Eigen::VectorXd> j_max)
                {
                    Limits l;
                    l.v_max = v_max;
                    l.a_max = a_max;
                    l.j_max = std::move(j_max);
                    return l;
                }
            ),
            py::arg("v_max"), py::arg("a_max"), py::arg("j_max") = std::nullopt
        )
        .def_readwrite("v_max", &Limits::v_max)
        .def_readwrite("a_max", &Limits::a_max)
        .def_readwrite("j_max", &Limits::j_max)
        .def(
            "__repr__",
            [](const Limits& l)
            {
                std::string s = "Limits(v_max=..., a_max=...";
                if (l.j_max.has_value())
                    s += ", j_max=...";
                s += ")";
                return s;
            }
        );

    py::class_<GenerationOptions>(m, "GenerationOptions")
        .def(
            py::init(
                [](double blend_radius, std::optional<Eigen::VectorXd> blend_radii, CornerHandling corner_handling,
                   std::optional<BlendShape> blend_shape)
                {
                    GenerationOptions o;
                    o.blend_radius = blend_radius;
                    o.blend_radii = std::move(blend_radii);
                    o.corner_handling = corner_handling;
                    o.blend_shape = blend_shape;
                    return o;
                }
            ),
            py::arg("blend_radius") = 0.1, py::arg("blend_radii") = std::nullopt,
            py::arg("corner_handling") = CornerHandling::StrictCorners, py::arg("blend_shape") = std::nullopt
        )
        .def_readwrite("blend_radius", &GenerationOptions::blend_radius)
        .def_readwrite("blend_radii", &GenerationOptions::blend_radii)
        .def_readwrite("corner_handling", &GenerationOptions::corner_handling)
        .def_readwrite("blend_shape", &GenerationOptions::blend_shape);

    py::class_<Sample>(m, "Sample")
        .def_readonly("t", &Sample::t)
        .def_readonly("q", &Sample::q)
        .def_readonly("qd", &Sample::qd)
        .def_readonly("qdd", &Sample::qdd)
        .def(
            "__repr__", [](const Sample& s)
            { return "Sample(t=" + std::to_string(s.t) + ", dim=" + std::to_string(s.q.size()) + ")"; }
        );

    py::class_<SegmentInfo>(m, "SegmentInfo")
        .def_readonly("type", &SegmentInfo::type)
        .def_readonly("duration", &SegmentInfo::duration)
        .def_readonly("v0", &SegmentInfo::v0)
        .def_readonly("length", &SegmentInfo::length)
        .def_readonly("u_dir", &SegmentInfo::u_dir)
        .def_readonly("q0", &SegmentInfo::q0)
        .def_readonly("blend_shape", &SegmentInfo::blend_shape)
        .def_readonly("V", &SegmentInfo::V)
        .def_readonly("r", &SegmentInfo::r)
        .def_readonly("d_in", &SegmentInfo::d_in)
        .def_readonly("d_out", &SegmentInfo::d_out)
        .def_readonly("q_start", &SegmentInfo::q_start);

    py::class_<Trajectory::SamplesResult>(m, "SamplesResult")
        .def_readonly("t", &Trajectory::SamplesResult::t)
        .def_readonly("q", &Trajectory::SamplesResult::q)
        .def_readonly("qd", &Trajectory::SamplesResult::qd)
        .def_readonly("qdd", &Trajectory::SamplesResult::qdd);

    py::class_<Trajectory>(m, "Trajectory")
        .def(py::init<>())
        .def("sample", &Trajectory::sample, py::arg("t"))
        .def("samples", py::overload_cast<double>(&Trajectory::samples, py::const_), py::arg("dt"))
        .def(
            "samples", [](const Trajectory& self, const Eigen::VectorXd& times) { return self.samples(times); },
            py::arg("times")
        )
        .def("duration", &Trajectory::duration)
        .def("dim", &Trajectory::dim)
        .def("junction_speeds", &Trajectory::junctionSpeeds, py::return_value_policy::reference_internal)
        .def("blend_radii", &Trajectory::blendRadii, py::return_value_policy::reference_internal)
        .def("corner_types", &Trajectory::cornerTypes, py::return_value_policy::reference_internal)
        .def("blend_shape", &Trajectory::blendShape)
        .def("corner_handling", &Trajectory::cornerHandling)
        .def("num_segments", &Trajectory::numSegments)
        .def("segments", &Trajectory::segments, py::return_value_policy::reference_internal)
        .def("empty", &Trajectory::empty);

    py::class_<TrajectoryGenerator>(m, "TrajectoryGenerator")
        .def("set_limits", &TrajectoryGenerator::setLimits, py::arg("limits"))
        .def("set_options", &TrajectoryGenerator::setOptions, py::arg("options"))
        .def("generate", &TrajectoryGenerator::generate, py::arg("waypoints"))
        .def("dim", &TrajectoryGenerator::dim)
        .def("limits_set", &TrajectoryGenerator::limitsSet)
        .def("limits", &TrajectoryGenerator::limits, py::return_value_policy::reference_internal)
        .def("options", &TrajectoryGenerator::options, py::return_value_policy::reference_internal);

    py::class_<TrapezoidalTrajectoryGenerator, TrajectoryGenerator>(m, "TrapezoidalTrajectoryGenerator")
        .def(py::init<std::size_t>(), py::arg("dim"));

    py::class_<SCurveTrajectoryGenerator, TrajectoryGenerator>(m, "SCurveTrajectoryGenerator")
        .def(py::init<std::size_t>(), py::arg("dim"));
}
