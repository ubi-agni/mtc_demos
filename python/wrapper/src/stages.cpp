#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>
#include <stages/grasp_provider.h>
#include <stages/generate_touch_pose.h>
#include <stages/automatica.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;
using namespace moveit::task_constructor::stages;

namespace moveit {
namespace python {

void export_mtc_stages()
{

	properties::class_<GenerateTouchPose, std::auto_ptr<GenerateTouchPose>, bp::bases<GenerateGraspPose>, boost::noncopyable>
	      ("GenerateTouchPose", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<GenerateTouchPose>, std::auto_ptr<Stage>>();

	bp::def("approachAndPush", &approachAndPush, bp::return_value_policy<bp::manage_new_object>());
	bp::def("grasp", &grasp, bp::return_value_policy<bp::manage_new_object>());
	bp::def("graspAndDrop", &graspAndDrop, bp::return_value_policy<bp::manage_new_object>());

	bp::def("juggleStart", &juggleStart, bp::return_value_policy<bp::manage_new_object>());
	bp::def("juggle", &juggle, bp::return_value_policy<bp::manage_new_object>());
}

} }
