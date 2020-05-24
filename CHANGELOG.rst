^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package datmo
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (24/5/2020)
-----------
* fixes message generation bug
* removes unused rviz configurations
* removes ukf filter tracking
* organizes topic names under datmo namespace
* adds tf2-geometry-msgs build dependency
* changes default world frame from world to laser
* adds experiment videos
* merges presentation
* adds corner switching
* changes breakpoint image
* adds clustering
* removes data association video
* adds association gif
* adds flowcharts
* deletes info stream
* removes unnecessary file
* changes covariance of ukf
* removes old l_shape_tracker
* moves two_robots bag file
* removes new tag from recorded bag file
* implements same orientation for kf and ukf
* populates more ukf messages
* publishes ukf tracking msgs
* publishes length-width
* tunes Q and R matrices
* implements Mahalanobis distance for cornerswitches
* deletes some clutter
* starts Lshape tracker merge
* implements ACT model
* implements constant turn rate model
* constant velocity model
* removes poseCovariance visualization
* refactors launch files
* Merge branch 'corner_switching'
* remoces unnecessary filters
* simplifies findOrientation function
* changes line length
* separates dynamic and shape updates
* same parameters kf ukf
* improves length width estimation
* fixes ukf Q matrix
* fixes clustering bug
* removes execution timing
* Merge branch 'master' of github.com:kostaskonkk/datmo
* adds overtake experiment
* adds overtake bagfile
* tries to improve rectanglefitting
* moves ukf prediction in detectCornerPointSwitch
* improves orientation estimation by KF
* fixes velocity estimation bug
* add experiment launch
* adds expemeriment launch
* adds launch for new experiment
* finishes ukf l_shape_tracker
* fixes orientatin bug
* implements constant acceleration model
* moves detectCornerPoint switch in lshapetracker
* adds separate LShapeTracker object for UKF
* adds launch files
* better distance for data association
* saves amount of tracked objects
* add constant turn rate ukf
* corrects orientations estimation
* moves all rviz configs in datmo
* adds box_kf
* ukf works with newtonian model
* refactoring of ukf filter class
* adds example bag file
* make filters remain between measurements
* white background rviz
* Adds some progress
* Merges with refactored master
* Fixes closest corner point bug
* Moves tf to datmo class/ major refactoring
* merge with master
* Code refactoring and cleaning
* Adds documentation
* Adds arrow for theta box
* Adds prototype covariance
* Addition of UKF
* Publishing of covariances
* Adds buggy nonlinear observer
* Refactoring and map_frame bug fix
* Fixes transformation bug
* Corrects the box_track_msg
* Publishing of length and width
* Timing of execution
* changed coordinate frame of box_tracks
* moved datmo constructor to source file
* finished corner point switching
* worked on shape switching
* initial corner point switching
* closeness criterion
* fixed bounding box in rviz
* fixed track.stamp bug
* absolute velocities, arrows and new track msg
* changed to PoseStamped in Track msg
* publishing of /tracks topic
* refactoring and initial bounding box
* Trajectories and primitive classification
* added configuration parameters
  xMerge branch 'master' of github.com:kostaskonkk/datmo
* Addition of Kalman Filter and estimation of velocity
* Update README.md
* v1
* Initial commit
* Contributors: Kostantinos Konstantinidis, Kostas, Kostas Flou, kostas
