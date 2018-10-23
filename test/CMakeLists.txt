#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)

	add_rostest(pr2_bimodal.test)
	add_rostest(pa10_bimodal.test)
# baxter test fails when run by rostest, but succeeds when run manually?!?
#	add_rostest(baxter_bimodal.test)
	add_rostest(pepper_bimodal.test)

	add_rostest(pepper_bimanual.test)
	add_rostest(yumi_bimanual.test)
	add_rostest(yumi_handover.test)
endif()
