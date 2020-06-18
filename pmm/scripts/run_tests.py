#!/usr/bin/env python
'''import rosunit
rosunit.unitrun('patrol_management_module','FromUIInteractions','test.unittest_patrol_management.IncomingDataTestSuite')'''

import rostest
rostest.rosrun('patrol_management_module','test_robot_init_msg','InputInitializeTeam')