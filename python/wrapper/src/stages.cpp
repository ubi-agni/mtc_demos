#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>
#include <stages/grasp_provider.h>
#include <stages/generate_touch_pose.h>
#include <stages/grasping_tasks.h>
#include <stages/automatica.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

namespace moveit {
namespace python {

namespace {
BOOST_PYTHON_FUNCTION_OVERLOADS(pickOL, pick, 0, 2);
BOOST_PYTHON_FUNCTION_OVERLOADS(pickPlaceOL, pickPlace, 1, 3);
BOOST_PYTHON_FUNCTION_OVERLOADS(bimodalPickOL, bimodalPick, 0, 1);
BOOST_PYTHON_FUNCTION_OVERLOADS(bimodalPickPlaceOL, bimodalPickPlace, 1, 2);
}

void export_mtc_stages()
{

	properties::class_<GenerateTouchPose, std::auto_ptr<GenerateTouchPose>, bp::bases<GenerateGraspPose>, boost::noncopyable>
	      ("GenerateTouchPose", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<GenerateTouchPose>, std::auto_ptr<Stage>>();

	bp::def("PickTask", &pick, pickOL()[bp::return_value_policy<bp::manage_new_object>()]);
	bp::def("PickPlaceTask", &pickPlace, pickPlaceOL()[bp::return_value_policy<bp::manage_new_object>()]);

	bp::def("BimodalPickTask", &bimodalPick, bimodalPickOL()[bp::return_value_policy<bp::manage_new_object>()]);
	bp::def("BimodalPickPlaceTask", &bimodalPickPlace, bimodalPickPlaceOL()[bp::return_value_policy<bp::manage_new_object>()]);

	bp::def("PickShakeTask", &pickShake, bp::return_value_policy<bp::manage_new_object>());

	bp::def("approachAndPush", &approachAndPush, bp::return_value_policy<bp::manage_new_object>());
	bp::def("grasp", &grasp, bp::return_value_policy<bp::manage_new_object>());
	bp::def("graspAndDrop", &graspAndDrop, bp::return_value_policy<bp::manage_new_object>());

	bp::def("juggleStart", &juggleStart, bp::return_value_policy<bp::manage_new_object>());
	bp::def("juggle", &juggle, bp::return_value_policy<bp::manage_new_object>());
}

} }
