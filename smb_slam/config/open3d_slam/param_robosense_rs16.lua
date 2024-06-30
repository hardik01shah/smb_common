include "default/default_parameters.lua"


params = deepcopy(DEFAULT_PARAMETERS)

--ScanToScan ODOMETRY
params.odometry.scan_processing.voxel_size = 0.2
params.odometry.scan_processing.downsampling_ratio = 1.0
params.odometry.scan_processing.scan_cropping.cropping_radius_max = 40.0

--Advanced Options.
params.odometry.use_odometry_topic_instead_of_scan_to_scan = true --Uses External Odometry topic instead of Scan2Scan registration.
params.odometry.use_IMU_for_attitude_initialization = false --Uses IMU msgs to initialize gravity aligned attitude.

--MAPPER_LOCALIZER
params.mapper_localizer.is_use_map_initialization = false --If enabled, a map should be given by the params below. 
params.mapper_localizer.republish_the_preloaded_map = false --if enabled, the pre-loaded map is re-published for viz. Costly.
params.mapper_localizer.is_merge_scans_into_map = false --if enabled, the provided map is extended with the scans we see.
params.mapper_localizer.is_build_dense_map = false --if enabled a dense map is built alongside the regular map. If you want this to be saved, enable submap saving.
params.mapper_localizer.is_attempt_loop_closures = false
params.mapper_localizer.is_print_timing_information = false --if enabled additional logs are printed
params.mapper_localizer.map_merge_delay_in_seconds = 10.0 --the delay in seconds before merging to the existing map.
params.mapper_localizer.is_refine_odometry_constraints_between_submaps = false

params.mapper_localizer.is_carving_enabled = false --Space carving, allows user to clear the dynamic obstables from the map if points are re-observed.
params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.25 
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 1.0 -- 1.0: no down sampling, 0.1: 1 in every 10 points left.
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 15.0 --meters
params.mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 2.0 --NOT USED RIGHT NOW
params.mapper_localizer.scan_to_map_registration.icp.knn = 5 --Currently only used for surface normal estimation.
params.mapper_localizer.scan_to_map_registration.icp.max_distance_knn = 1.0 --Currently only used for surface normal estimation.
params.mapper_localizer.scan_to_map_registration.icp.reference_cloud_seting_period = 2.0 --Seconds

--MAP_INITIALIZER
params.map_initializer.pcd_file_package = "open3d_slam_ros"
params.map_initializer.pcd_file_path = "" --you need to give your own map here. .pcd or .ply well supported.
params.map_initializer.is_initialize_interactively = false
params.map_initializer.init_pose.x = 0.0
params.map_initializer.init_pose.y = 2.0
params.map_initializer.init_pose.z = 0.0
params.map_initializer.init_pose.roll = 0.0
params.map_initializer.init_pose.pitch = 0.0
params.map_initializer.init_pose.yaw = 120.0

--SUBMAP
params.submap.submap_size = 15.0 --meters
params.submap.adjacency_based_revisiting_min_fitness = 0.5 --How easy we want the system to switch submaps
params.submap.min_seconds_between_feature_computation = 10.0 --seconds, this is a feature of loop-closure
params.submap.submaps_num_scan_overlap = 5 -- 10 -15
params.submap.max_num_points = 250000 --We dont want our submaps to get too large in dense scenes where distance is a not a indicator of fullness. Hence, we create a submap also based on this upper point limit.

--MAP_BUILDER
params.map_builder.map_voxel_size = 0.25 --the map voxel size 25cm
params.map_builder.scan_cropping.cropping_radius_max = 40.0
params.map_builder.scan_cropping.cropping_radius_min = 2.0
params.map_builder.space_carving.carve_space_every_n_scans = 10 --feature of space carving
params.map_builder.space_carving.max_raytracing_length = 20 --feature of space carving
params.map_builder.space_carving.truncation_distance = 0.3 --feature of space carving
params.map_builder.space_carving.voxel_size = 0.2 --feature of space carving

--DENSE_MAP_BUILDER
params.dense_map_builder.map_voxel_size = 0.05
params.dense_map_builder.scan_cropping.cropping_radius_max = 105.0
params.dense_map_builder.space_carving.carve_space_every_n_scans = 10
params.dense_map_builder.space_carving.truncation_distance = 0.1

--PLACE_RECOGNITION
params.place_recognition.ransac_min_corresondence_set_size = 40
params.place_recognition.max_icp_correspondence_distance = 1.0
params.place_recognition.min_icp_refinement_fitness = 0.7
params.place_recognition.dump_aligned_place_recognitions_to_file = false 
params.place_recognition.min_submaps_between_loop_closures = 2 --If you make this 0, you start doing submap-to-submap registration as well. Loop-closure feature has to be enabled for this to take action.
params.place_recognition.loop_closure_search_radius = 20.0 -- The range of search for loop-closures, technically it has to be bigger than the submap size.
params.place_recognition.consistency_check.max_drift_roll = 30.0 --deg
params.place_recognition.consistency_check.max_drift_pitch = 30.0 --deg
params.place_recognition.consistency_check.max_drift_yaw = 30.0 --deg
params.place_recognition.consistency_check.max_drift_x = 80.0 --m
params.place_recognition.consistency_check.max_drift_y = 80.0 --m
params.place_recognition.consistency_check.max_drift_z = 40.0 --m

--MOTION_COMPENSATION
params.motion_compensation.is_undistort_scan = false --Needs point timestamp field to exist. Not well-tested for summer school.
params.motion_compensation.num_poses_vel_estimation = 3

--SAVING
params.saving.save_map = true
params.saving.save_submaps = false --If enabled, the submaps are separately saved.


return params