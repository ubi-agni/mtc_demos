#include <moveit/python/task_constructor/properties.h>
#include <stages/grasp_provider.h>
#include <stages/generate_touch_pose.h>
#include <stages/grasping_tasks.h>
#include <stages/automatica.h>

namespace py = pybind11;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::stages::GenerateTouchPose)

namespace moveit {
namespace python {

void export_mtc_stages(pybind11::module& m)
{
	properties::class_<GenerateTouchPose, GenerateGraspPose>(m, "GenerateTouchPose")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("touch_pose"));

	m.def("PickTask", &pick, py::arg("name") = std::string("pick"), py::arg("side") = std::string("right"));
	m.def("PickPlaceTask", &pickPlace, py::arg("object_target_pose"),
	      py::arg("name") = std::string("pick + place"), py::arg("side") = std::string("right"));

	m.def("BimodalPickTask", &bimodalPick, py::arg("name") = std::string("bimodal pick"));
	m.def("BimodalPickPlaceTask", &bimodalPickPlace, py::arg("object_target_pose"),
	      py::arg("name") = std::string("bimodal pick + place"));

	m.def("PickShakeTask", &pickShake, py::arg("name"), py::arg("side") = std::string("left"), py::arg("num") = 3u);

	m.def("approachAndPush", &approachAndPush, py::arg("name"), py::arg("side"));
	m.def("grasp", &grasp, py::arg("name"), py::arg("side"));
	m.def("graspAndDrop", &graspAndDrop, py::arg("name"), py::arg("side"));

	m.def("juggleStart", &juggleStart);
	m.def("juggle", &juggle);
}

} }
