cmake_minimum_required(VERSION 3.0.2)
project(hnbv_eval)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/aggregate_trials.py
  scripts/compute_metrics.py
  scripts/extract_rosbag.py
  scripts/export_tables.py
  scripts/make_figures.py
  scripts/make_report.py
  scripts/plot_trajectories.py
  scripts/run_experiment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
