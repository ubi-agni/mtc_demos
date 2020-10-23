#include <unistd.h>

inline bool doWait(int argc, char** argv, double timeout = 5.0) {
	// return false if any argument is "nowait"
	for (int i=1; i < argc; ++i)
		if (argv[i] == std::string("nowait")) {
			// wait some time for move_group to come up
			ros::WallDuration(timeout).sleep();
			return false;
		}

	return true;
}
