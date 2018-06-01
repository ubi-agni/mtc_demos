#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	add_rostest(pr2_bimodal.test)
	add_rostest(pa10_bimodal.test)
# baxter test fails when run by rostest, but succeeds when run manually?!?
#	add_rostest(baxter_bimodal.test)
	add_rostest(pepper_bimodal.test)
endif()
