^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_uav_trajectory_generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2023-01-20)
------------------

1.0.3 (2023-01-20)
------------------
* updated readme, updated ci
* fixed launch, env->optenv
* Update README.md
* Contributors: Tomas Baca, Tomáš Báča

1.0.2 (2022-05-09)
------------------
* refactored agains publisher handler
* updated transformer interface
* refactored agains the new transformer
* + install in cmakelists, refactoring
* fixed arbitrary frame_id bug
* add ScopeTimer and its logger; disabled by default
* Contributors: Pavel Petracek, Tomas Baca

1.0.1 (2021-10-05)
------------------
* added path input id
* Add absolute limit on computation time
* Add check of feasibilty for constraints overriding
* Add override jerk constraints
* reworked constrains: separate horizontal, vertical and heading
* added async execution
* added checks for nlopt return code
* updated constraints callback
* relaxed default path straightenner distance to 5 cm
* added loop option
* added republisher for the original path
* added path-from-future compensation
* Contributors: Tomas Baca, Vit Kratky, Yurii Stasinchuk

* updated ros::shutdown
* fixed includes
* fixed trajectory from future time stamp
* added looping mechanism
* added republisher for the original path
* updated defaut configs to be more safe
* fixed frame id for rviz markers
* removed warn flags
* updated CI
* removed unused argument
* added debug prints showing computation time
* fixed initial conditions bug
* added additional nan checks
* added fallback method for trajectory generation
* updated the bad-sampling check
* added trajectory sampling check with factor =2.0
* ceil -> floor in the first trajectory idx
* tweaked futurization to 1/2 ;-)
* added path-from-future compensation
* Contributors: Tomas Baca

0.9.9
-----------
* Hello there!
