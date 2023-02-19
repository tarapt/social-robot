ConfigVersion 2.0
Section Files
;SectionFlags for Files: 
Map /home/tara/Robita_lab.map                   ; Map of the environment that the robot uses for
                         			; navigation


Section Localization Manager settings
;SectionFlags for Localization Manager settings: 
AllowResetFromLost false ; When there are more than one localizations, it can
                         ; so happen that some localizations may get lost while
                         ; the others keep localized. This flag decides whether
                         ; to allow the localization manager to feed the robot
                         ; pose from the localizations that are not lost to
                         ; reset the pose of the localizations that are lost.

DistanceLimit 1000.00000 ; If the standard deviation of the pose XY is less
                         ; than this value the robot will be moved to this mean
                         ; pose using moveTo. If the value is negative the
                         ; limit is considered infinite. (used along with the
                         ; AngleLimit parameter)

AngleLimit 10.00000      ; If the standard deviation of the pose Theta is less
                         ; than this value the robot will be moved to this mean
                         ; pose using moveTo. If the value is negative the
                         ; limit is considered infinite. (used along with the
                         ; DistanceLimit parameter)

LogFlag false            ; Write debug data to a file named locaManagerLog.txt

ErrorLogFlag false       ; Write the localization error computed at each
                         ; successful localization onto disk.


Section Localization settings
;SectionFlags for Localization settings: 
NumSamples 2000          ; minimum 1,  No of pose samples for MCL. The larger
                         ; this number, the more computation will localization
                         ; take. Too low a number will cause the robot to lose
                         ; localization. This is also the maximum no of samples
                         ; which will be used for localization if no of samples
                         ; are varied along with the localization score.

NumSamplesAtInit 2000    ; minimum 0,  No of pose samples for MCL when
                         ; initializing the robot in the map. Since this is
                         ; presumably done when the robot is at rest, it can be
                         ; larger than NumSamples and use more computation for
                         ; more accuracy. (A value of 0 will make it equal to
                         ; NumSamples).

GridRes 100.00000        ; minimum 10,  The resolution of the occupancy grid
                         ; representing the map in mm. Smaller resolution
                         ; results in more accuracy but more computation.

PassThreshold 0.20000    ; range [0, 1],  After MCL sensor correction, the
                         ; sample with the maximum probablity will have a score
                         ; based on the match between sensor and the map
                         ; points. This is the minimum score out of 1.0 to be
                         ; considered localized. Note that this threshold now
                         ; is not the only measure of the robot being lost.
                         ; Once the score drops below PassThreshold, the robot
                         ; has to move a significant distance to raise its
                         ; uncertainity above the parameter called
                         ; LostThresholdDistance to be considered lost.

LostThresholdDistance 100.00000 ; minimum 0,  This threshold will only come
                         ; into play if none of the samples during sensor
                         ; correction step, score higher than the pass
                         ; threshold. As soon as this happens, the localization
                         ; uncertainity distance will grow based on the
                         ; movement from the last localized pose. If the robot
                         ; pose computed by the kalman filter has its position
                         ; uncertainity greater than this distance, the robot
                         ; will be considered lost as far as localization is
                         ; concerned. mm

KMmPerMm 0.05000         ; minimum 0,  When the robot moves linearly, the error
                         ; in distance is proportional to the distance moved.
                         ; This error is is given as a fraction in mm per mm

KDegPerDeg 0.05000       ; minimum 0,  When the robot rotates, the error in the
                         ; angle is proportional to the angle turned. This is
                         ; expressed as a fraction in degs per deg.

KDegPerMm 0.00250        ; minimum 0,  When the robot moves linearly it can
                         ; also affect its orientation. This drift can be
                         ; expressed as a fraction in degs per mm.

TriggerDistance 200.00000 ; minimum 0,  Since MCL localization is
                         ; computationally expensive, it is triggered only when
                         ; the robot has moved this far in mm.

TriggerAngle 5.00000     ; minimum 0,  Since MCL localization is
                         ; computationally expensive, it is triggered only when
                         ; the robot has rotated this far in degs.

TriggerTimeEnabled false ; This flag will decide if the localization will be
                         ; called every 'TriggerTime' msecs. Once this flag is
                         ; true the IdleTimeTrigger* parameters will take
                         ; effect. This feature is meant to take care of cases
                         ; when the robot has not moved much for a time and the
                         ; position should be refined .

TriggerTime 10000.00000  ; minimum 1500,  Once the TriggerTimeFlag is set to
                         ; true this parameter will decide how long the robot
                         ; has been idle in milli seconds before it starts a
                         ; localization near the last known robot pose.

IdleTimeTriggerX 200.00000 ; minimum 0,  When localization is triggered by idle
                         ; time this parameter decides the range of the samples
                         ; in X coords in mm.

IdleTimeTriggerY 200.00000 ; minimum 0,  When localization is triggered by idle
                         ; time this parameter decides the range of the samples
                         ; in Y coords in mm.

IdleTimeTriggerTh 15.00000 ; minimum 0,  When localization is triggered by idle
                         ; time this parameter decides the range of the samples
                         ; in Theta coords in degs.

RecoverOnFail false      ; If localization fails, this flag will decide if a
                         ; static localization is attempted around last known
                         ; robot pose. Such a reinitialization can cause the
                         ; robot to be hopelessly lost if the actual robot is
                         ; very different from its known pose

FailedX 300.00000        ; minimum 0,  Range of the box in the X axis in mm to
                         ; distribute samples after localization fails.

FailedY 300.00000        ; minimum 0,  Range of the box in the Y axis in mm to
                         ; distribute samples after localization fails.

FailedTh 45.00000        ; minimum 0,  Range of the angle in degs to distribute
                         ; samples after localization fails.

PeturbX 10.00000         ; minimum 0,  After sensor correction and resampling
                         ; the chosen pose is perturbed to generate a new
                         ; sample. This parameter decides the range to peturb
                         ; the X axis in mm.

PeturbY 10.00000         ; minimum 0,  After sensor correction and resampling
                         ; the chosen pose is perturbed to generate a new
                         ; sample. This parameter decides the range to peturb
                         ; the Y axis in mm.

PeturbTh 1.00000         ; minimum 0,  After sensor correction and resampling
                         ; the chosen pose is perturbed to generate a new
                         ; sample. This parameter decides the range to peturb
                         ; the angle in degs.

PeakStdX 10.00000        ; minimum 0,  Extent of the ellipse in the X axis in
                         ; mm beyond which the sample poses will be considered
                         ; multiple localizations after resampling.

PeakStdY 10.00000        ; minimum 0,  Extent of the ellipse in the X axis in
                         ; mm beyond which the sample poses will be considered
                         ; multiple localizations after resampling.

PeakStdTh 1.00000        ; minimum 0,  Extent of the angle in degs beyond which
                         ; the sample poses will be considered multiple
                         ; localizations after resampling.

PeakFactor 0.00000       ; range [0, 1],  When a no of samples have non zero
                         ; probabilities such as when there is ambiguities in a
                         ; corridor. This is the threshold below the maximum
                         ; probablity to be considered a valid hypothesis.

StdX 400.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; ellipse in X axis in mm at start of localization.

StdY 400.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; ellipse in Y axis in mm at start of localization.

StdTh 30.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; angle in degs at start of localization.

SensorBelief 0.90000     ; range [0, 1],  Probablility that a range reading
                         ; from the laser is valid. This is used in the
                         ; correction of the probablities of the samples using
                         ; the sensor.

OccThreshold 0.10000     ; range [0, 1],  The threshold value of the occupancy
                         ; grid to consider as occupied.

AngleIncrement 0.00000   ; range [0, 180],  Only the laser readings which are
                         ; this many degrees apart are used for the
                         ; localization. The lower limit is decided by the
                         ; LaserIncrement setting

DiscardThreshold 0.33000 ; range [0.33, 1],  A robot sample pose lying inside
                         ; an occupancy grid cell with a value above this will
                         ; be usually discarded Useful in cases where robot may
                         ; intersect map points such as during patrolbot
                         ; docking

ScoreToVarFactor 0.00000 ; minimum 0,  When MCL based localization is combined
                         ; with other localizations using the variance and
                         ; mean. The variance of the pose in the MCL is
                         ; increased by the exp(1/pow(score, ScoreToVarFactor)

TransitionFromIdleNumSamples 100 ; minimum 1,  When localization is renabled
                         ; after it was idle, the laser localization has to
                         ; initialize at the new robot pose. This is the number
                         ; of samples it will use in that case.

TransitionFromIdleXStd 100.00000 ; minimum 0,  When localization is renabled
                         ; after it was idle, the laser localization has to
                         ; initialize at the new robot pose. This is the spread
                         ; of the samples in the X coordinates.

TransitionFromIdleYStd 100.00000 ; minimum 0,  When localization is renabled
                         ; after it was idle, the laser localization has to
                         ; initialize at the new robot pose. This is the spread
                         ; of the samples in the Y coordinates.

TransitionFromIdleTStd 1.00000 ; minimum 0,  When localization is renabled
                         ; after it was idle, the laser localization has to
                         ; initialize at the new robot pose. This is the spread
                         ; of the samples in the T coordinates.

EnableReflectorLocalization false ; ARNL uses an extended Kalman filter which
                         ; allows you to combine the data from the MCL
                         ; localization, and range measurements from reflectors
                         ; if mapped and seen by the laser. This advanced
                         ; feature can be disabled to revert to the basic MCL
                         ; localization, using this flag

DistanceThreshold -1.00000 ; The robot pose computed by the kalman filter will
                         ; be used to reset the robot pose using moveTo() when
                         ; the two diverge by more than this distance. If the
                         ; threshold is negative the robot pose will not be
                         ; corrected by the kalman pose.

ReflectorVariance 100000.00000 ; minimum 0,  This number will be used as the
                         ; variance of the (x, y) coords of the center of the
                         ; reflectors in the R matrix of the Kalman filter.

ReflectorMatchDist 2000.00000 ; minimum 0,  When finding the closest reflector
                         ; in the map to an observed reflection, this is the
                         ; maximum distance the system will search to find the
                         ; closest reflector.

ReflectorMatchAngle 5.00000 ; minimum 0,  When finding the closest reflector in
                         ; the map to an observed reflection, this is the
                         ; maximum angle about which the system will search to
                         ; find the closest reflector.

ReflectorMaxRange 32000.00000 ; minimum 0,  This is the maximum distance that
                         ; the SICK lrf is capable of seeing a reflector. (This
                         ; is smaller than the max range of the regular SICK
                         ; readings)

ReflectorMaxAngle 45.00000 ; minimum 0,  This is the maximum angle of incidence
                         ; that the SICK lrf is capable of seeing a reflector
                         ; at. (This is much smaller than the angle that the
                         ; regular SICK readings are capable of returning)

ReflectorSize 300.00000  ; minimum 0,  When clustering the reflector points in
                         ; the laser return, all reflector points inside this
                         ; range will be classed as belonging to one reflector.

ReflectorAngleVarLimit 0.00000 ; minimum 0,  The (2, 2) element in the kalman
                         ; filter P for the reflector localization seems to be
                         ; too small. This may swamp out the angle information
                         ; from the other localizations. Till a solution is
                         ; found this is artificial lower limit imposed on the
                         ; (2, 2) element.

BadReflectorFactor 1000.00000 ; minimum 1,  When reflectors are far away, they
                         ; are sensed as lone points. Such points sometimes
                         ; flicker between edges if they are just around a
                         ; corner. This can mess up localization. So the
                         ; variance on x and y for this measurement will be
                         ; increased by this factor in such cases.

ReflectorRangeFactor -1.00000 ; When reflectors are far away, their
                         ; measurements are not as accurate as when they are
                         ; closer. To account for this, the variance of such
                         ; reflectors are weighted down by
                         ; range(mm)^2/reflectorRangeFactor. A negative value
                         ; will make it not use this factor at all.

EnableTriangulation true ; When possible the kalman cycle will resort to
                         ; triangulation when multiple reflectors are seen.
                         ; This flag can be used to enable or disable it.

ReflectorTriDistLimit 500.00000 ; minimum 0,  When possible the kalman cycle
                         ; will resort to triangulation when multiple
                         ; reflectors are seen and triangulation is enabled.
                         ; The resulting pose will be allowed if the standard
                         ; deviation in the XY coords is less this limit

ReflectorTriAngLimit 5.00000 ; minimum 0,  When possible the kalman cycle will
                         ; resort to triangulation when multiple reflectors are
                         ; seen and triangulation is enabled. The resulting
                         ; pose will be allowed if the standard deviation in
                         ; the angle coord is less this limit

ReflectanceThreshold 31  ; range [0, 255],  The SICK laser is capable of
                         ; sensing the reflectance of the points it ranges.
                         ; When reflectors are used for localization by
                         ; triangulation or otherwise, it is imperative that
                         ; the reflections the SICK sees is the one coming from
                         ; a known reflector and not from other shiny objects
                         ; in the map. To exclude these extraneous reflections
                         ; all reflectances below this threshold are discarded.
                         ; The reflectors made by SICK that MobileRobots use
                         ; for localization usually return a value greater than
                         ; 31 when 3 reflector bits are used. Change this value
                         ; accordingly when smaller no of rbits are used.

QTra 1000.00000          ; minimum 0,  The variance of X and Y coords in the
                         ; kalman filter's plant model. (mm^2)

QRot 1.00000             ; minimum 0,  The variance of Theta coords in the
                         ; kalman filter's plant model. (deg^2)

QTraVel 1.00000          ; minimum 0,  The variance of X and Y velocity in the
                         ; kalman filter's plant model. (mm/sec)^2

QRotVel 1.00000          ; minimum 0,  The variance of rot velocity in the
                         ; kalman filter's plant model. (deg/sec)^2

QTraAcc 1.00000          ; minimum 0,  The variance of X and Y acceleration in
                         ; the kalman filter's plant model. (mm/sec/sec)^2

QRotAcc 1.00000          ; minimum 0,  The variance of rot acceleration in the
                         ; kalman filter's plant model. (deg/sec/sec)^2

QZeroSpeedFactor 100.00000 ; minimum 1,  The factor by which the Qs will be
                         ; divided by when the robot is still

XYLimit 400.00000        ; minimum 0,  Three times this limit will be the
                         ; maximum innovation allowed in the XY coords during
                         ; sensor update. If the innovation exceeds this it
                         ; will be considered an outlier and the sensor reading
                         ; will be ignored.(mm)

ThLimit 4.00000          ; minimum 0,  Three times this limit will be the
                         ; maximum innovation allowed in the theta coords
                         ; during sensor update. If the innovation exceeds this
                         ; it will be considered an outlier and the sensor
                         ; reading will be ignored.(degs)

UseAllK true             ; The gain matrix K of the Kalman is sensitive to the
                         ; values of Q and R especially about the elements
                         ; related to the angle. In some cases, it might be
                         ; easier to blank out all innovations related to the
                         ; angle from all measurements. This is the flag to use
                         ; in that case.

LogFlag false            ; Write internal debugging data onto disk

LogLevel 1               ; minimum 1,  The amount of logging. 1 is minimal and
                         ; 3 is maximum

ErrorLogFlag false       ; Write the localization error computed at each
                         ; successful localization onto disk.

AdjustNumSamplesFlag false ; The number of samples is by default kept high to
                         ; keep the robot from losing localization even after
                         ; initialization. This number can be lowered during
                         ; motion in places of the map where the localization
                         ; score is high to reduce the computation load. Set
                         ; this flag to true if you want to vary the number of
                         ; samples with the localization score. (As the score
                         ; drops, the no of samples will rise)

MinNumSamples 200        ; minimum 0,  When the AdjustSamplesFlag is set to
                         ; true the number of samples is reduced as the
                         ; localization score rises. But, this will be the
                         ; lowest number it will be reduced to.

NumSamplesAngleFactor 1.00000 ; minimum 0,  When the AdjustSamplesFlag is set
                         ; to true the number of samples is reduced as the
                         ; localization score rises. But, when the robot has
                         ; rotated significantly, it needs more samples than if
                         ; it had only moved in translation. A bigger angle
                         ; factor will cause the no of samples to not drop as
                         ; fast when the localization is triggered due to
                         ; rotation.


Section Sonar Localization settings
;SectionFlags for Sonar Localization settings: 
;  MCL Sonar Localization parameters

NumSamples 2000          ; minimum 1,  No of pose samples for MCL. The larger
                         ; this number, the more computation will localization
                         ; take. Too low a number will cause the robot to lose
                         ; localization. This is also the maximum no of samples
                         ; which will be used for localization if no of samples
                         ; are varied along with the localization score.

NumSamplesAtInit 10000   ; minimum 0,  No of pose samples for MCL when
                         ; initializing the robot in the map. Since this is
                         ; presumably done when the robot is at rest, it can be
                         ; larger than NumSamples and use more computation for
                         ; more accuracy. (A value of 0 will make it equal to
                         ; NumSamples).

AdjustNumSamplesFlag false ; The number of samples is by default kept high to
                         ; keep the robot from losing localization even after
                         ; initialization. This number can be lowered during
                         ; motion in places of the map where the localization
                         ; score is high to reduce the computation load. Set
                         ; this flag to true if you want to vary the number of
                         ; samples with the localization score. (As the score
                         ; drops the no of samples will rise)

PassThreshold 0.10000    ; range [0, 1],  After MCL sensor correction, the
                         ; sample with the maximum probablity will have a score
                         ; based on the closeness between sensor and the map
                         ; lines. This is the minimum score out of 1.0 to be
                         ; considered localized. This number is related to the
                         ; sum of probabilities of the sensor readings matching
                         ; the distance to the closest line in the map.

PeturbX 10.00000         ; minimum 0,  After sensor correction and resampling
                         ; the chosen pose is perturbed to generate a new
                         ; sample. This parameter decides the range to peturb
                         ; the X axis in mm.

PeturbY 10.00000         ; minimum 0,  After sensor correction and resampling
                         ; the chosen pose is perturbed to generate a new
                         ; sample. This parameter decides the range to peturb
                         ; the Y axis in mm.

PeturbTh 1.00000         ; minimum 0,  After sensor correction and resampling
                         ; the chosen pose is perturbed to generate a new
                         ; sample. This parameter decides the range to peturb
                         ; the angle in degs.

FailedX 300.00000        ; minimum 0,  Range of the box in the X axis in mm to
                         ; distribute samples after localization fails.

FailedY 300.00000        ; minimum 0,  Range of the box in the Y axis in mm to
                         ; distribute samples after localization fails.

FailedTh 45.00000        ; minimum 0,  Range of the angle in degs to distribute
                         ; samples after localization fails.

RecoverOnFail false      ; If localization fails, this flag will decide if a
                         ; static localization is attempted around last known
                         ; robot pose. Such a reinitialization can cause the
                         ; robot to be hopelessly lost if the actual robot is
                         ; very different from its known pose

PeakStdX 100.00000       ; minimum 0,  Extent of the ellipse in the X axis in
                         ; mm beyond which the sample poses will be considered
                         ; multiple localizations after resampling.

PeakStdY 100.00000       ; minimum 0,  Extent of the ellipse in the X axis in
                         ; mm beyond which the sample poses will be considered
                         ; multiple localizations after resampling.

PeakStdTh 2.00000        ; minimum 0,  Extent of the angle in degs beyond which
                         ; the sample poses will be considered multiple
                         ; localizations after resampling.

PeakFactor 0.90000       ; range [0, 1],  When a no of samples have non zero
                         ; probabilities such as when there is ambiguities in a
                         ; corridor. This is the threshold below the maximum
                         ; probablity to be considered a valid hypothesis.

; 

;  Stationary error settings

StdX 400.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; ellipse in X axis in mm at start of localization.

StdY 400.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; ellipse in Y axis in mm at start of localization.

StdTh 30.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; angle in degs at start of localization.

; 

;  Motion error coefficients settings

KMmPerMm 0.05000         ; minimum 0,  When the robot moves linearly, the error
                         ; in distance is proportional to the distance moved.
                         ; This error is is given as a fraction in mm per mm

KDegPerDeg 0.05000       ; minimum 0,  When the robot rotates, the error in the
                         ; angle is proportional to the angle turned. This is
                         ; expressed as a fraction in degs per deg.

KDegPerMm 0.00250        ; minimum 0,  When the robot moves linearly it can
                         ; also affect its orientation. This drift can be
                         ; expressed as a fraction in degs per mm.

TriggerDistance 200.00000 ; minimum 0,  Since MCL localization is
                         ; computationally expensive, it is triggered only when
                         ; the robot has moved this far in mm.

TriggerAngle 5.00000     ; minimum 0,  Since MCL localization is
                         ; computationally expensive, it is triggered only when
                         ; the robot has rotated this far in degs.

TriggerTimeEnabled false ; This flag will decide if the localization will be
                         ; called every 'TriggerTime' msecs. Once this flag is
                         ; true the IdleTimeTrigger* parameters will take
                         ; effect. This feature is meant to take care of cases
                         ; when the robot has not moved much for a time and the
                         ; position should be refined .

TriggerTime 10000.00000  ; minimum 1500,  Once the TriggerTimeFlag is set to
                         ; true this parameter will decide how long the robot
                         ; has been idle in milli seconds before it starts a
                         ; localization near the last known robot pose.

IdleTimeTriggerX 200.00000 ; minimum 0,  When localization is triggered by idle
                         ; time this parameter decides the range of the samples
                         ; in X coords in mm.

IdleTimeTriggerY 200.00000 ; minimum 0,  When localization is triggered by idle
                         ; time this parameter decides the range of the samples
                         ; in Y coords in mm.

IdleTimeTriggerTh 15.00000 ; minimum 0,  When localization is triggered by idle
                         ; time this parameter decides the range of the samples
                         ; in Theta coords in degs.

; 

;  Range sensor (laser) settings

SonarMaxRange 2000.00000 ; minimum 1000,  Maximum range of the sonar sensors in
                         ; mm.

SonarAperture 30.00000   ; range [0, 180],  Aperture of the sonar sensors in
                         ; degs.

SonarRangeRes 100.00000  ; minimum 10,  Range resolution of the probability
                         ; table in mm.

SonarAngleRes 1.00000    ; minimum 1,  Angle resolution of the probability
                         ; table in degs.

SonarIncidenceLimit 15.00000 ; minimum 0,  Angle of incidence with an obstacle
                         ; surface at which sonar reflection becomes specular
                         ; in degs.

SonarLambdaF 0.00002     ; minimum 0,  Probablity of false readings in
                         ; times/mm.

SonarLambdaR 0.00005     ; minimum 0,  Probablity of reflection of readings in
                         ; times/mm.

SonarSigma 30.00000      ; minimum 0,  Standard deviation of sonar range in mm.

SonarBetaMin 0.80000     ; minimum 0,  Attenuation factor at min reading.

SonarBetaMax 0.60000     ; minimum 0,  Attenuation factor at max reading.

SonarMinLineSize 100.00000 ; minimum 0,  Min size of lines in the map which
                         ; will beused for localization.

MinNumSamples 200        ; minimum 0,  When the AdjustSamplesFlag is set to
                         ; true the number of samples is reduced as the
                         ; localization score rises. But, this will be the
                         ; lowest number it will be reduced to.

NumSamplesAngleFactor 1.00000 ; minimum 0,  When the AdjustSamplesFlag is set
                         ; to true the number of samples is reduced as the
                         ; localization score rises. But, when the robot has
                         ; rotated significantly, it needs more samples than if
                         ; it had only moved in translation. A bigger angle
                         ; factor will cause the no of samples to not drop as
                         ; fast when the localization is triggered due to
                         ; rotation.


Section GPS Localization settings
;SectionFlags for GPS Localization settings: 
InitFactor 1             ; minimum 1,  Factor by which the 5 secs of GPS is
                         ; increased for averaging before setting it as
                         ; reference.

RTKError 200.00000       ; minimum 0,  The value for the user equivalent range
                         ; errors for the Omnistar after getting a 3d fix. This
                         ; value is used to compute the std dev of the GPS
                         ; readings. (mm)

FRTKError 400.00000      ; minimum 0,  The value for the user equivalent range
                         ; errors for the Omnistar while floating before
                         ; getting a 3d fix. This value is used to compute the
                         ; std dev of the GPS readings. (mm)

DGPSError 800.00000      ; minimum 0,  The value for the user equivalent range
                         ; errors when omnistar is lost but DGPS is available.
                         ; This value is used to compute the std dev of the GPS
                         ; readings. (mm)

GPSError 1600.00000      ; minimum 0,  The value for the user equivalent range
                         ; errors when omnistar is lost but basic GPS is
                         ; available. This value is used to compute the std dev
                         ; of the GPS readings. (mm)

BadGPSFactor 1.00000     ; minimum 1,  Aria maps can have areas in which GPS is
                         ; known to be unreliable be marked using sectors. If
                         ; the robot enters such a sector the std dev of the
                         ; GPS pose will be increased by this factor.

UseFirstAngleFix false   ; Flag which when set lets the robot to set the
                         ; heading to AngleFixOffset in East-North coordinates.
                         ; If set to false the heading set by the user will be
                         ; substituted.

AngleFixOffset 0.00000   ; When aligning the robot with a known map the first
                         ; fix can be visually aligned with known mapped
                         ; objects such as walls. This fine adjustment of the
                         ; orientation is better handled through such an offset
                         ; instead of from the drag clicking of the mouse.
                         ; (degs)

StdX 400.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; ellipse in X axis in mm at start of localization.

StdY 400.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; ellipse in Y axis in mm at start of localization.

StdTh 30.00000           ; minimum 0,  The standard deviation of the gaussian
                         ; angle in degs at start of localization.

DriveForHeadingIncrement 1000.00000 ; Minimum distance between the points
                         ; stored during the drive for heading mode for the
                         ; point to be stored.

DriveForHeadingMaxPoints 100 ; minimum 2,  Maximum no of points stored during
                         ; the drive for heading mode. The limit is in case the
                         ; user begins the mode and forgets to end it.

KMMPerMMLin 0.01000      ; minimum 0,  When the robot moves forward, the error
                         ; in distance is proportional to the distance moved.
                         ; This error is specified as a fraction in mm per mm

KMMPerMMLat 0.01000      ; minimum 0,  When the robot moves sideways, the error
                         ; in distance is proportional to the distance moved.
                         ; This error is specified as a fraction in mm per mm

KDegPerDeg 0.00050       ; minimum 0,  When the robot rotates, its orientation
                         ; error is in proportion to the angle moved. This
                         ; error is is specified as a fraction in degs per deg

KDegPerMMLin 0.00100     ; minimum 0,  When the robot moves forward, its
                         ; orientation error is in proportion to the distance
                         ; moved. This error is specified as a fraction in degs
                         ; per mm

KDegPerMMLat 0.00100     ; minimum 0,  When the robot moves sideways, its
                         ; orientation error is in proportion to the distance
                         ; moved. This error is specified as a fraction in degs
                         ; per mm

LostThreshold 1000.00000 ; minimum 0,  If the robot pose computed by the kalman
                         ; filter has its position uncertainity greater than
                         ; this distance, the robot will be considered lost as
                         ; far as GPS localization is concerned. mm

DistanceThreshold 0.00000 ; The robot pose computed by the kalman filter will
                         ; be used to reset the robot pose using moveTo() when
                         ; the two diverge by more than this distance. If the
                         ; threshold is negative the robot pose will not be
                         ; corrected by the kalman pose.

RLinVel 1000.00000       ; minimum 0,  The variance of the robot's linear
                         ; velocity as reported by the odometry, which is
                         ; needed for the kalman filter. (mm/sec)^2

RLatVel 1000.00000       ; minimum 0,  The variance of the robots lateral
                         ; velocity as reported by the odometry, which is
                         ; needed for the kalman filter. (mm/sec)^2

RRotVel 40.00000         ; minimum 0,  The variance of the robots rotational
                         ; velocity as reported by the odometry, which is
                         ; needed for the kalman filter. (degs/sec)^2

QTra 100000.00000        ; minimum 0,  The variance of X and Y coords in the
                         ; kalman filter's plant model. (mm^2)

QRot 1.00000             ; minimum 0,  The variance of Theta coords in the
                         ; kalman filter's plant model. (deg^2)

QTraVel 1.00000          ; minimum 0,  The variance of X and Y velocity in the
                         ; kalman filter's plant model. (mm/sec)^2

QRotVel 1.00000          ; minimum 0,  The variance of rot velocity in the
                         ; kalman filter's plant model. (deg/sec)^2

QTraAcc 1.00000          ; minimum 0,  The variance of X and Y acceleration in
                         ; the kalman filter's plant model. (mm/sec/sec)^2

QRotAcc 1.00000          ; minimum 0,  The variance of rot acceleration in the
                         ; kalman filter's plant model. (deg/sec/sec)^2

QZeroSpeedFactor 10000000000.00000 ; minimum 1,  The factor by which the Qs
                         ; will be divided by when the robot is still. Odometry
                         ; to be trusted more than GPS when still.

XYLimit 1000.00000       ; minimum 0,  Three times this limit will be the
                         ; maximum innovation allowed in the XY coords during
                         ; sensor update. If the innovation exceeds this it
                         ; will be considered an outlier and the sensor reading
                         ; will be ignored.(mm)

ThLimit 40.00000         ; minimum 0,  Three times this limit will be the
                         ; maximum innovation allowed in the theta coords
                         ; during sensor update. If the innovation exceeds this
                         ; it will be considered an outlier and the sensor
                         ; reading will be ignored.(degs)

UseGPSHeading true       ; Since the magnetic compass is rendered useless by
                         ; SEEKUR's extremly powerful motors, the alternative
                         ; is to compute the heading from GPS lines. This is
                         ; the flag that decides whether to use this technique
                         ; or not.

GPSHeadingDist 100.00000 ; minimum 0,  GPS heading is to computed as the
                         ; heading between two GPS reads. This is the min
                         ; distance the GPS points must be separated between
                         ; GPS reads to make such a line. The robot odometry
                         ; must also have moved at least this distance for the
                         ; line to be valid (mm) Typically this should be set
                         ; to just above the max vel times the time between two
                         ; GPS readings.

GPSHeadingLinVel 200.00000 ; minimum 0,  This is the min linear speed the
                         ; vehicle must be going between GPS reads to be useful
                         ; for GPS heading computation. (mm/sec)

GPSHeadingLatVel 10.00000 ; minimum 0,  This is the max lateral speed the
                         ; vehicle can be going between GPS reads to be useful
                         ; for GPS heading computation. (mm/sec)

GPSHeadingRotVel 5.00000 ; minimum 0,  This is the max rotational speed the
                         ; vehicle must be going between GPS reads to be useful
                         ; for GPS heading computation. (deg/sec)

GPSHeadingDOP 4.00000    ; minimum 0,  This is the max Horizontal dilution of
                         ; precision of the GPS reads to be useful for GPS
                         ; heading computation.

GPSHeadingTurn 5.00000   ; minimum 0,  This is the max angle the robot can turn
                         ; between the GPS reads to be useful for GPS heading
                         ; computation.

GPSHeadingError 10.00000 ; minimum 0,  The std dev of the GPS heading error.
                         ; The compass is currently not being used for
                         ; navigation. Instead the GPS points are used to
                         ; compute the heading when the vehicle is going
                         ; straight ahead. (degs)

UseAllK false            ; The gain matrix K of the Kalman is sensitive to the
                         ; values of Q and R especially about the elements
                         ; related to the angle. In some cases, it might be
                         ; easier to blank out all innovations related to the
                         ; angle from all measurements except the compass, such
                         ; as the GPS measurements. This is the flag to use in
                         ; that case.

StateSize 6              ; range [6, 9],  The Kalman filter state can be
                         ; configured to be position, velocity (PV) or position
                         ; velocity and accceleration (PVA) The former is 6
                         ; elements and the latter 9. The user can choose
                         ; depending on the anticipated dynamics and accuracy
                         ; of the sensor data

DisplaySize 250          ; minimum 0,  The number of points shown in the trail
                         ; of odometry, GPS and Kalman poses for debugging
                         ; purposes. Setting this to 0 will remove the
                         ; computation associated with this and probably useful
                         ; for saving computer cycles.

LogFlag false            ; Flag to write data into file


Section Path Planning Settings
;  The parameters in this section will decide how the robot will plan a path
;  and then follow it.
;SectionFlags for Path Planning Settings: 
; 

;  The parameters in this section will decide how the robot will plan a path
;  and then follow it.

MaxSpeed 750.00000       ; minimum 0,  Maximum speed during path following in
                         ; mm/sec.

MaxRotSpeed 100.00000    ; minimum 0,  Maximum rotational speed in degs/sec

PlanRes 106.25000        ; minimum 0,  The resolution of the grid used for path
                         ; planning in mm. It is best to use an integral
                         ; fraction of the robot half width to allow for more
                         ; free space in tight spaces.

UseRadiusForPlan false   ; ARNL uses the half width of the robot to expand all
                         ; obstacles to compute the 2D configuration space for
                         ; planning. This allows the robot to squeak a plan
                         ; through narrow doors but could lead to it getting
                         ; stuck when following that path if it marginally gets
                         ; off that path. To avoid this, you can make ARNL use
                         ; the bigger radius of the robot during the planning
                         ; and avoid getting stuck in narrow passages.

UseRearFraction 1.00000  ; range [0, 1],  ARNL uses the full length to rear
                         ; measure to make a rectanble to check for collisions
                         ; during path following. This may cause the robot to
                         ; get stuck during path following when the rear
                         ; corners clip obstacles. To reduce this, you can set
                         ; this fraction to less than 1.0. This allows you to
                         ; factor in the rounded corners which are typical of
                         ; real robots.

PlanFreeSpace 637.50000  ; minimum 0,  Preferred distance from side of robot to
                         ; obstacles. The path planner marks each grid cell
                         ; depending on its distance to the nearest obstacle.
                         ; Any cell within half the robot width of an occupied
                         ; cell will still be non traversable by the robot.The
                         ; cell at half robot width will be traversable but
                         ; will have the maximum cost. The cost of the cells
                         ; beyond half width will progressively decrease. The
                         ; PlanFreeSpace is the distance from the side of the
                         ; robot beyond which the cell cost stops decreasing
                         ; and remains constant. Larger values will cause
                         ; larger excursions around obstacles, (This variable
                         ; is related to the FreeSpacing variable in previous
                         ; versions of ARNL which was measured from the center
                         ; of the robot.)

CollisionRange 2000.00000 ; minimum 0,  The distance from the robot within
                         ; which the obstacles seen by the sensor and those on
                         ; the map are used to compute the local path. (User
                         ; should keep this above (maxlvel*maxlvel)/2/goalDecel
                         ; to allow for smoother motion when encountering
                         ; unmapped obstacles)

UseCollisionRangeForPlanning false ; The robot plans ahead locally for a
                         ; distance based on its speed. This may not be
                         ; sufficient in some cases where the user may want it
                         ; to look ahead till the CollisionRange of the
                         ; sensors. This flag will enable the user to force the
                         ; robot to look at least as far as the distance the
                         ; sensors data is incorporated.

ClearOnFail true         ; If this flag is true, any failure in the local path
                         ; planning will force the cumulative range buffers to
                         ; be cleared

NumTimesToReplan 100     ; minimum 0,  The maximum number of times to replan to
                         ; a goal if the path is blocked, 0 for never

FrontClearance 100.00000 ; minimum 0,  Front clearance in mm of the robot while
                         ; avoiding obstacles.

SlowSpeed 100.00000      ; minimum 0,  Speed below and at which is considered
                         ; slow.

SideClearanceAtSlowSpeed 100.00000 ; minimum 0,  Side clearance in mm of the
                         ; robot while avoiding obstacles when it is moving at
                         ; or below the slow speed.

FrontPaddingAtSlowSpeed 100.00000 ; minimum 0,  Distance in addition to the
                         ; front clearance of the robot while avoiding
                         ; obstacles when it is moving at or below the slow
                         ; speed. In this padding the super max translation
                         ; deceleration will be allowed to engage to keep the
                         ; obstacle away before slamming on eStop.

FastSpeed 2000.00000     ; minimum 0,  Speed at or above which is considered
                         ; fast.

SideClearanceAtFastSpeed 1000.00000 ; minimum 0,  Side clearance in mm of the
                         ; robot while avoiding obstacles when it is moving at
                         ; or above the fast speed.

FrontPaddingAtFastSpeed 1000.00000 ; minimum 0,  Distance in addition to the
                         ; front clearance of the robot while avoiding
                         ; obstacles when it is moving at or above the fast
                         ; speed. In this padding the super max translation
                         ; deceleration will be allowed to engage to keep the
                         ; obstacle away before slamming on eStop.

SuperMaxTransDecel 1000.00000 ; minimum 0,  The maximum translational
                         ; deceleration allowed for avoiding obstacles. This is
                         ; used in conjunction with the padding parameters to
                         ; limit the wear and tear if emergency deceleration is
                         ; unlimited.

EmergencyMaxTransDecel 0.00000 ; minimum 0,  The emergency translational
                         ; deceleration allowed for avoiding obstacles. This is
                         ; used in conjunction with the padding parameters to
                         ; limit the maximum deceleration allowed even if there
                         ; is a chance for collision. (A zero will make it use
                         ; the computed deceleration or SuperMaxTransDecel
                         ; however large it happens to be)

UseEStop false           ; If this flag is true, any obstacle in the path which
                         ; cannot be avoided by regular robot deceleration will
                         ; cause the software emergency stop to be engaged
                         ; which will result in an almost instantaneous stop.
                         ; (May not be suitable for platforms like the
                         ; wheelchair)

UseLaser true            ; Use the laser for collision avoidance?

UseSonar true            ; Use the sonar for collision avoidance? Due to the
                         ; sonar inaccuracies the obstacles may appear larger.
                         ; This flag can be used to set the sonar off when the
                         ; robot has to navigate through narrow openings such
                         ; as doors.

SecsToFail 8             ; minimum 0,  The time in seconds for which local
                         ; search will continue trying to plan when it fails to
                         ; find a safe local path to move.

EndMoveSecsToFail 60     ; The timeout for the end move to goal in seconds
                         ; after which reaching the goal is cancelled. This is
                         ; a safety feature in case other systems fail. A
                         ; negative value will allow an infinite timeout.

LocalPathFailDistance 1000.00000 ; minimum 0,  When the sensors see that the
                         ; local path is blocked, the robot will still be
                         ; allowed to drive this close to the block. This is
                         ; meant to help in cases where the robot senses false
                         ; obstacles from afar and stops too soon such as on
                         ; ramps. (The distance at which the robot actually
                         ; stops will depend on the velocity of the robot)

MarkOldPathFactor 0.75000 ; range [0.1, 1],  When the robot operates in an
                         ; environment where there are a number of unmapped
                         ; obstacles, the local path it plans may flip from one
                         ; side of the obstacle to the other between cycles. To
                         ; avoid this, ARNL marks the old path by reducing the
                         ; costs of its cells by this factor. This factor
                         ; should be a non zero value between 0.1 and 1

StartMarkPoint 0.20000   ; range [0, 1],  When the robot operates in an
                         ; environment where there are a number of unmapped
                         ; obstacles, the local path it plans may flip from one
                         ; side of the obstacle to the other between cycles. To
                         ; avoid this, ARNL marks the old path by reducing the
                         ; costs of those cells. The marking will only start
                         ; from this fraction of the local path to its end to
                         ; avoid persistent kinks in the path

GoalDistanceTol 200.00000 ; minimum 0,  When the robot is at this distance in
                         ; mm to the goal the end move will start and the robot
                         ; will slow down and try to reach the goal.

GoalAngleTol 10.00000    ; minimum 0,  If the robot angle to the goal
                         ; orientation is within this value, the goal
                         ; orientation will considered as reached.

GoalOccupiedFailDistance 1000.00000 ; minimum 100,  When the sensors see that
                         ; the goal is occupied the robot will still be allowed
                         ; to drive this close to the goal anyway.

GoalSpeed 250.00000      ; minimum 0,  Maximum speed at which end move to goal
                         ; is executed. This value and the robot's inertia will
                         ; decide the goal positioning accuracy. (A value of 0
                         ; will keep the normal driving value)

GoalRotSpeed 33.00000    ; minimum 0,  Maximum rotational velocity at which end
                         ; move to goal is executed. This value and the robot's
                         ; inertia will decide the goal positioning accuracy.
                         ; (A value of 0 will keep the normal driving value)

GoalRotVelKp 10.00000    ; minimum 0,  When the robot is closing in on the goal
                         ; ARNL will use the error in goal direction angle to
                         ; send rotational velocity commands to correct the
                         ; error. This value will be the mulitplier used in
                         ; that computation

GoalTransAccel 200.00000 ; minimum 0,  Maximum linear acceleration at which end
                         ; move to goal is executed. (A value of 0 will keep
                         ; the normal driving value)

GoalTransDecel 200.00000 ; minimum 0,  Maximum linear deceleration at which end
                         ; move to goal is executed. (A value of 0 will keep
                         ; the normal driving value)

GoalRotAccel 33.00000    ; minimum 0,  Maximum rotational acceleration at which
                         ; end move to goal is executed. (A value of 0 will
                         ; keep the normal driving value)

GoalRotDecel 33.00000    ; minimum 0,  Maximum rotational deceleration at which
                         ; end move to goal is executed. (A value of 0 will
                         ; keep the normal driving value)

GoalSwitchTime 0.40000   ; minimum 0,  Time in secs to switch into end move
                         ; mode in addition to the coasting time. (To allow for
                         ; slack in the ramping down of the velocity to zero)

GoalUseEncoder true      ; For fine positioning at goal, the robot can switch
                         ; to moving based on its encoder pose only. This flag
                         ; decides this

AlignAngle 30.00000      ; minimum 0,  When the robot is stopped this is the
                         ; minimum angle it will rotate in place to align to
                         ; the planned path before it will move linearly.

AlignSpeed 10.00000      ; minimum 0,  This is the velocity below which the
                         ; robot will do the aligning to path direction before
                         ; linear motion.

HeadingRotSpeed 50.00000 ; minimum 0,  There will be some points on the path
                         ; where the robot will be exclusively rotating to
                         ; reach a heading. The user may wish to set the
                         ; rotational speed in this situation to lower than
                         ; normal especially when dealing with heavier robots
                         ; like the Powerbot. (A value of 0 will keep the
                         ; normal driving value) (This will not override the
                         ; GoalRotSpeed)

HeadingRotAccel 50.00000 ; minimum 0,  There will be some points on the path
                         ; where the robot will be exclusively rotating to
                         ; reach a heading. The user may wish to set the
                         ; rotational accel in this situation to lower than
                         ; normal especially when dealing with heavier robots
                         ; like the Powerbot. (A value of 0 will keep the
                         ; normal driving value) (This will not override the
                         ; GoalRotAccel)

HeadingRotDecel 50.00000 ; minimum 0,  There will be some points on the path
                         ; where the robot will be exclusively rotating to
                         ; reach a heading. The user may wish to set the
                         ; rotational decel in this situation to lower than
                         ; normal especially when dealing with heavier robots
                         ; like the Powerbot. (A value of 0 will keep the
                         ; normal driving value) (This will not override the
                         ; GoalRotDecel)

HeadingWt 0.80000        ; range [0, 1],  Heading weight for DWA. Unlike the
                         ; heading objective computed from a destination pose
                         ; on the path, as in the conventional DWA, we use a
                         ; path matching function to match the arc made from
                         ; the velocities with the desired computed path.

DistanceWt 0.10000       ; range [0, 1],  Distance weight for DWA. Distance
                         ; refers to the distance to collision if any if the
                         ; robot continues on the path computed from the
                         ; velocities.

VelocityWt 0.10000       ; range [0, 1],  Velocity weight for DWA. Velocity
                         ; refers to the linear velocity only.

SmoothingWt 0.00000      ; range [0, 1],  Smoothing weight for DWA. This is the
                         ; weight given to smoother paths. The smoothing
                         ; objective minimizes the changes in the linear and
                         ; rotational velocities from this cycle from the
                         ; velocities used in the previous cycle. This is
                         ; probably useful only when smoothness not speed is
                         ; more important. Set this to 0.1 or so for smoothly
                         ; traversing narrow passages such as doors.

NforLinVelIncrements 1   ; minimum 0,  If N is the value of this parameter, the
                         ; no of linear velocity increments of the search table
                         ; is 2*N+1 for the DWA.

NforRotVelIncrements 8   ; minimum 0,  If N is the value of this parameter, the
                         ; no of rotational velocity increments of the search
                         ; table is 2*N+1 for the DWA.

SmoothWindow 2           ; minimum 0,  Smoothing window size for DWA

ObsThreshold 0.20000     ; range [0, 1],  The threshold value of the occupancy
                         ; grid to consider as occupied for path planning.

MatchLengths 2.00000     ; minimum 0,  The DWA path matching distance is
                         ; computed as a multiple of the robot lengths. This
                         ; parameter decides the multiplier for this purpose.
                         ; Shorter path lengths may be appropriate for driving
                         ; through narrow openings such as doors at low speeds.

CheckInsideRadius true   ; If this flag is false, then the robot path tracking
                         ; module will ignore any objects it sees inside its
                         ; radius and still keep moving to its local goal
                         ; point. This may be useful in case the user wants to
                         ; reproduce such behavior in older versions of ARNL.

CheckInsideBounds false  ; If this flag is set to false, the robot will behave
                         ; as in the old days when obstacle points inside the
                         ; robot bounds did not prevent it from turning at the
                         ; start and end of a path plan. If true the robot will
                         ; not move if there is any obstacle is sensed inside
                         ; its bounds.

SplineDegree 3           ; minimum 1,  Degree of the B-Splines used to smooth
                         ; local path.

NumSplinePoints 5        ; minimum 1,  The number of points that will be used
                         ; to subdivide the look ahead in the local path which
                         ; will then serve as the knots for the spline to form
                         ; over.

CenterAwayCost 1.00000   ; Added cost of being away from the central spine of
                         ; the one way areas. A zero would cause it to not
                         ; consider centering in one way areas at all.

OneWayToOldCostFactor 0.90000 ; range [0, 0.99],  The factor by which a cell
                         ; cost is weighted in one way sectors, against the old
                         ; value of the cell computed based on free space from
                         ; obstacles. A low value will cause the robot to plan
                         ; paths based more on the free spacing. A high value
                         ; will cause the robot to plan paths which are closer
                         ; to the center line in the one way sector.

Resistance 2             ; range [1, 32767],  The Resistance for Restrictive
                         ; sectors and lines. The cost of traversing a cell is
                         ; multiplied by this resistance value. The normal cost
                         ; for regular non-restrictive cell is 1, so set this
                         ; value to 1 to turn off restrictive behavior (on
                         ; default Resisteds and all Restrictives sectors).

Preference 2             ; range [1, 32767],  The preference for Preferred
                         ; lines. The cost of traversing a cell is divided by
                         ; this preference value. The normal cost or regular
                         ; non-prefered cell is 1, so set this value to 1 to
                         ; turn off preferred behavior for PreferredLines.

PreferredLineWidth 300.00000 ; minimum 0,  If there are preferred lines in the
                         ; map the robot will compute paths preferring those
                         ; lines. The preferred line width decides how wide
                         ; these preferred lines will extend to the sides. The
                         ; default value of 0 will keep the lines a single cell
                         ; wide.

PreferredDirectionSideOffset 250.00000 ; The side offset decides how far away
                         ; from the appropriate side of the preferred direction
                         ; sector the robot will keep itself during path
                         ; planning. This is to allow the user to customize a
                         ; prefered spine on the sector instead of relying on
                         ; the center line.

PreferredDirectionAwayCost 1.00000 ; minimum 0,  The cost of being away from
                         ; the side offset line in preferred direction sectors.
                         ; Change this only if the default value does not seem
                         ; to do the right thing.

PreferredDirectionWrongWayCost 4.00000 ; minimum 1,  This is the cost of going
                         ; wrong way in the preferred direction sectors. Note
                         ; that the actual value used is the sum of this value
                         ; and the PreferedDirectionSideAway cost. Change this
                         ; only if the default value does not seem to do the
                         ; right thing.

PreferredDirectionToOldCostFactor 0.25000 ; range [0, 0.99],  The value for the
                         ; preferred to old cost factor. The factor by which a
                         ; cell cost is weighted in two way preferred direction
                         ; sectors and single preferred direction sectors,
                         ; against the old value of the cell computed based on
                         ; free space from obstacles. A low value will cause
                         ; the robot to plan paths based more on the free
                         ; spacing. A high value will cause the robot to plan
                         ; paths which are closer to the line decided by the
                         ; side offset in the sector. This is used only for
                         ; local path planning.

PreferredDirectionToOldCostFactorMain 0.90000 ; range [0, 0.99],  The value for
                         ; the preferred to old cost factor for computing the
                         ; main path to goal. This factor is the conterpart of
                         ; the PreferredDirectionToOldCostFactor, which is used
                         ; only for during the main path plan through preferred
                         ; direction sectors.

NoLocalPlanLookAhead 200.00000 ; minimum 0,  If the no local plan is in effect
                         ; due the robot being in such a sector or by setting
                         ; the flag. This parameter will decide the extremely
                         ; short lookahead in such cases.

LogFlag false            ; Flag to write data into file

LogLevel 0               ; minimum 0,  Level of detail in the log data

StallRecoverEnabled false ; Whether a stall recover should be done when the
                         ; robot stalls.

StallRecoverSpeed 150.00000 ; minimum 0,  Speed at which to back away when
                         ; stalled.

StallRecoverDuration 50  ; minimum 0,  Cycles of operation to move when
                         ; recovering from stall.

StallRecoverRotation 45.00000 ; minimum 0,  Amount of rotation when recovering
                         ; (degrees).

DrivingTransVelMax 0.00000 ; minimum 0,  Maximum forward translational velocity
                         ; (0 means use default)

DrivingTransNegVelMax 0.00000 ; minimum 0,  Maximum backwards translational
                         ; velocity (0 means use default)

DrivingTransAccel 0.00000 ; minimum 0,  Translational acceleration (0 means use
                         ; default)

DrivingTransDecel 0.00000 ; minimum 0,  Translational deceleration (0 means use
                         ; default)

DrivingRotVelMax 0.00000 ; minimum 0,  Maximum rotational velocity (0 means use
                         ; default)

DrivingRotAccel 0.00000  ; minimum 0,  Rotational acceleration (0 means use
                         ; default)

DrivingRotDecel 0.00000  ; minimum 0,  Rotational deceleration (0 means use
                         ; default)


Section Teleop settings
;SectionFlags for Teleop settings: 
FullThrottleForwards 0.00000 ; minimum 0,  The maximum forwards speed (0 means
                         ; robot's TransVelMax) (mm/sec)

FullThrottleBackwards 290.00000 ; minimum 0,  The maximum backwards speed (0
                         ; means 1/4 robot's TransVelMax) (mm/sec)

RotAtFullForwards 35.00000 ; minimum 0,  The maximum speed we turn at when
                         ; going full forwards (0 means 1/2 robots RotVelMax)
                         ; (deg/sec)

RotAtFullBackwards 35.00000 ; minimum 0,  The maximum speed we turn at when
                         ; going full backwards (0 means 1/2 robots RotVelMax)
                         ; (deg/sec)

RotAtStopped 60.00000    ; minimum 0,  The maximum speed we turn at when
                         ; stopped (0 means robot's RotVelMax) (deg/sec)

TransDeadZone 10.00000   ; minimum 0,  The percentage in the middle of the
                         ; translation direction to not drive (percent)

RotDeadZone 5.00000      ; minimum 0,  The percentage in the middle of the
                         ; rotation direction to not drive (percent)

