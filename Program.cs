using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System;
using VRage.Collections;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;
using VRage;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        // =====================================================================
        // CONFIGURATION
        // Change these values if you want to tweak the behavior of this script!
        // =====================================================================

        private const float X = 1.0f;

        // If this is true, the script will use the programmable block itself as an LCD screen and update the text on the
        // programmable block (as well as any other LCDs the script has already detected).
        // Default value: false
        private const bool DISPLAY_ON_PB = false;

        // Similar to DISPLAY_ON_PB
        // Default value: false
        private const bool DISPLAY_ON_COCKPIT = false;

        // The minimum altitude, in meters, this script will try to maintain. If the ship is below this altitude, it will fly up.
        // Default value: 3.5
        private const double NORMAL_MIN_ALTITUDE = 3.5;

        // The maximum altitude, in meters, this script will try to maintain. If the ship is above this altitude, it will fly down.
        // Default value: 7.0
        private const double NORMAL_MAX_ALTITUDE = 7.0;

        // The vertical speed that the script will try to maintain when landing.
        // Default value: 3.5
        private const double LANDING_VELOCITY = 3.5f;

        // If the ship is landing, the script will automatically turn off all the thrusters when the ship's altitude is at or
        // below this number.
        // Default value: 2.0
        private const double LANDING_ALTITUDE_THRESHOLD = 2.0;

        // Normally, the script will automatically set the parameters of the LCD panel to make sure the text is easily visible.
        // If this is false it will not do so.
        // Default value: true
        private const bool AUTOMATICALLY_SET_LCD_PARAMS = true;

        // When the ship is slowing down and its deceleration at least this value, then the ship's stopping distance will be
        // displayed on the third line of the LCD panel.
        // Default value: 2.0
        private const double DISPLAY_STOPDIST_THRESHOLD_ACCEL = 2.0;

        // When the ship is slowing down and its speed is greater than this value, then the ship's stopping distance will be
        // displayed on the third line of the LCD panel.
        // Default value: 5.0
        private const double DISPLAY_STOPDIST_THRESHOLD_SPEED = 5.0;

        // (Advanced) To calculate the slope of the ground, the script will automatically check the height of the ground every
        // few meters. This value means how many meters the script should wait before checking the height of the ground again.
        // Default value: 5.0
        private const double GROUND_HEIGHT_SAMPLE_RATE = 5.0;

        // If the ship has bottom-facing cameras, the script will perform use these cameras to perform a raycast (scan) several
        // times every second to check the height of the ground. This is the interval, in seconds, between each raycast. A
        // smaller interval will result in more raycasts, which means more accurate height measurements, but makes the script
        // more processing-heavy and could cause lag. (Note: the interval at which cameras can raycast may also be limited by
        // the server's settings)
        // Default value: 0.125
        private const double GROUND_RAYCAST_INTERVAL = 0.001f;

        private const double GROUND_RAYCAST_EXPIRY = 0.25;

        // The maximum distance, in meters, that downwards-facing cameras will raycast when trying to determine the height of the
        // ground. See GROUND_RAYCAST_INTERVAL for more info.
        // Default value: 150.0
        private const double MAX_DOWNWARDS_RAYCAST = 150.0;

        private const double HANGAR_MODE_MIN_ALTITUDE = 2.0;
        private const double HANGAR_MODE_MAX_ALTITUDE = 3.0;

        // ==============================================================================
        // DEBUG CONFIGURATION
        // These are likely not useful unless you're editing the script or told to change
        // them by a developer
        // ==============================================================================

        // Show difference between raycasted altitude and cockpit altitude on third line of LCD
        private const bool DEBUG_RAYCAST_DIFF = true;

        // ==========================================================================
        // Don't change anything below this line (unless you want to edit the script)
        // ==========================================================================

        private const string VERSION = "1.1_dev";
        private const string DEFAULT_BLOCK_GROUP = "Hoverbike";

        private string _blockGroupName;

        // Blocks and stuff needed for the script, set up in the Init() function
        private bool _initialized;
        private IMyCockpit _cockpit;
        private List<IMyThrust> _thrusters;
        private List<ScanningCamera> _forwardCameras;
        private List<ScanningCamera> _bottomCameras;
        private float _bottomCamerasMaxAngle;
        private List<IMyTextSurface> _displays;

        // Control variables (public to save state between script runs)
        private bool _controlling;
        private bool _landing;
        private bool _isHangarMode;

        // Variables related to obtaining ground height through checking altitude & sea level
        private Vector3 _lastSeaLevelPos;
        private double _lastGroundHeight;

        // Computed slope of the ground
        private double _slope;

        // Computed center mass
        private Vector3 _localCenterOfMass;
        private float _comPhysicalMass;

        private double _totalTimeRan;

        public Program()
        {
            LoadCustomData();
            Init();
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
        }

        public void Save()
        {
            SaveCustomData();
        }

        public void Main(string argument, UpdateType updateSource)
        {
            // Handle continuous update
            if ((updateSource & UpdateType.Update1) != 0)
            {
                if (_initialized && _controlling)
                {
                    DoUpdate();
                    _totalTimeRan += Runtime.TimeSinceLastRun.TotalSeconds;
                }
                return;
            }

            // Make arguments case insensitive
            string capitalsArgument = argument;
            argument = argument.ToLower();

            // Handle actions that don't require script initialization
            if (argument == "init")
            {
                SaveCustomData();
                Init();
                return;
            }

            if (argument.StartsWith("blockgroup"))
            {
                if (argument.Length <= "blockgroup ".Length)
                {
                    Echo($"The the block group currently being used by this script is \"{_blockGroupName}\"." +
                        $"You can change it by running the script with the argument \"BlockGroup <New group>\"");
                }
                else
                {
                    _blockGroupName = capitalsArgument.Substring("blockgroup ".Length);
                    SaveCustomData();

                    Echo($"The block group used by this script has been changed to \"{_blockGroupName}\".");
                }
                return;
            }

            // Handle arguments that do require that the script is initialized
            if (!_initialized)
            {
                Echo("The script isn't initialized yet. Run it with the argument \"Init\" (without the quotes) to set it up.");
                return;
            }

            if (argument == "auto")
            {
                _controlling = true;
                _landing = false;
                _isHangarMode = false;
            }
            if (argument == "stop")
            {
                _controlling = false;
                if (_initialized)
                {
                    ResetThrusters();
                    ResetDisplays();
                }
            }
            if (argument == "land")
            {
                _landing = !_controlling ? true : !_landing;
                _controlling = true;
            }
            if (argument == "hangar")
            {
                _controlling = true;
                _landing = false;
                _isHangarMode = true;
            }
        }

        void DoUpdate()
        {
            // Determine ship characteristics
            MyShipMass myMass = _cockpit.CalculateShipMass();
            float mass = myMass.PhysicalMass;

            MyShipVelocities myVelocity = _cockpit.GetShipVelocities();
            Vector3 velocity = myVelocity.LinearVelocity;

            // Recalculate center of mass if necessary
            if (mass != _comPhysicalMass)
                ComputeLocalCenterOfMass();

            // Determine gravity vector
            float gravity = (float)_cockpit.GetNaturalGravity().Length();
            Vector3 gravityDown = _cockpit.GetNaturalGravity() / gravity;

            // Determine axes relative to the ship
            IMyCubeGrid grid = Me.CubeGrid;
            Vector3 shipDown = grid.GridIntegerToWorld(-Base6Directions.GetIntVector(_cockpit.Orientation.Up)) -
                grid.GridIntegerToWorld(Vector3I.Zero);
            Vector3 shipRight = grid.GridIntegerToWorld(-Base6Directions.GetIntVector(_cockpit.Orientation.Left)) -
                grid.GridIntegerToWorld(Vector3I.Zero);
            Vector3 shipForward = grid.GridIntegerToWorld(Base6Directions.GetIntVector(_cockpit.Orientation.Forward)) -
                grid.GridIntegerToWorld(Vector3I.Zero);
            shipDown.Normalize();
            shipRight.Normalize();
            shipForward.Normalize();

            // Determine axes relative to the ship, but also parallel to the ground
            Vector3 groundForward = Vector3.Cross(shipRight, gravityDown);
            Vector3 groundRight = Vector3.Cross(gravityDown, groundForward);
            groundForward.Normalize();
            groundRight.Normalize();

            // Calculate ship roll and pitch
            // Normally, we would be using acos(dot(x, y)), but since Vector3.Dot returns a float and Math.Acos takes a double, the dot value is
            // automatically casted from a float to a double, which can sometimes result in 1f getting cast to 1.00001 due to precision errors,
            // which causes acos to return NaN. Therefore we need to clamp the dot value after we cast it to avoid such errors from causing NaN.
            double shipRollDot = Clamp(0.0, 1.0, (double) Vector3.Dot(groundRight, shipRight));
            double shipPitchDot = Clamp(0.0, 1.0, (double) Vector3.Dot(groundForward, shipForward));

            float shipRoll = ToDeg((float) Math.Acos(shipRollDot)) * Math.Sign(Vector3.Dot(gravityDown, shipRight));
            float shipPitch = ToDeg((float) Math.Acos(shipPitchDot)) * -Math.Sign(Vector3.Dot(gravityDown, shipForward));

            // Determine maximum acceleration that can be provided by the thrusters
            float thrustAlignment = Vector3.Dot(shipDown, gravityDown);
            float minAccel = -gravity;
            float maxAccel = -gravity;
            foreach (IMyThrust thrust in _thrusters)
            {
                // Acceleration is theoretically "Force / Mass", but we have to take into account the fact that the thrust vector might not
                // be aligned with gravity
                float perfectAccel = thrust.MaxEffectiveThrust / mass;
                maxAccel += perfectAccel * thrustAlignment;
            }

            // Determine current altitude
            var altitudeTuple = CheckAltitude(gravityDown, shipDown);
            double? raycastedAltitude = altitudeTuple.a;
            double? elevationAltitude = altitudeTuple.b;
            double? maybeAltitude = raycastedAltitude ?? elevationAltitude; // prefer raycasted altitude, fall back to elevation altitude
            if (maybeAltitude == null)
            {
                Echo("Couldn't compute altitude " + _totalTimeRan.ToString("F1"));
                ResetThrusters();
                return;
            }
            double altitude = maybeAltitude.Value;

            // Check the ground height by comparing our current altitude to sea level
            double distanceAboveSl;
            _cockpit.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out distanceAboveSl);
            double groundHeight = distanceAboveSl - altitude;

            // Determine components of velocity relative to various axes
            float verticalVelocity = Vector3.Dot(velocity, -gravityDown);
            Vector3 horizontalVelocity = velocity + verticalVelocity * gravityDown;
            float forwardVelocity = Vector3.Dot(velocity, groundForward);
            float sidewaysVelocity = Vector3.Dot(velocity, groundRight);

            // Measure the slope of the ground by seeing how fast ground height is changing
            Vector3D seaLevelPosition = _cockpit.GetPosition() - gravityDown * (float) distanceAboveSl;
            if (Vector3.DistanceSquared(seaLevelPosition, _lastSeaLevelPos) >= (GROUND_HEIGHT_SAMPLE_RATE * GROUND_HEIGHT_SAMPLE_RATE))
            {
                _slope = (groundHeight - _lastGroundHeight) / Vector3.Distance(seaLevelPosition, _lastSeaLevelPos);

                _lastSeaLevelPos = seaLevelPosition;
                _lastGroundHeight = groundHeight;
            }

            // Calculate the vertical velocity, but take into account the slope of the ground.
            double slopeVelocity = verticalVelocity - Math.Max(_slope, 0.0) * horizontalVelocity.Length();

            // Determine the magnitude of the current thrust vector
            float currentThrustAmount = 0f;
            foreach (IMyThrust thrust in _thrusters)
            {
                currentThrustAmount += thrust.CurrentThrust;
            }

            // If we are slowing down, estimate the stopping distance
            float gravityAccel = gravity / thrustAlignment; // the amount of acceleration supplied by thrusters needed to counteract gravity
            float forwardAcceleration = gravityAccel * Vector3.Dot(shipForward, gravityDown);
            float stoppingDistance = -1f;
            if (Math.Sign(forwardAcceleration) != Math.Sign(forwardVelocity)) { // check if we are slowing down
                float t = -forwardVelocity / forwardAcceleration;
                stoppingDistance = Math.Abs(forwardVelocity * t + 0.5f * forwardAcceleration * t * t);
            }

            // Determine the desired minumum/maximum altitude based on the current control settings
            double minAltitude = _isHangarMode ? HANGAR_MODE_MIN_ALTITUDE : NORMAL_MIN_ALTITUDE;
            double maxAltitude = _isHangarMode ? HANGAR_MODE_MAX_ALTITUDE : NORMAL_MAX_ALTITUDE;

            // OK, we know everything we need to know to start doing the autopilot thingy
            // Determine desired net vertical acceleration (we'll take gravity into account later)
            float accel;
            bool isManualInput = false;
            if (Math.Abs(_cockpit.MoveIndicator.Y) > 0.1f)
            {
                // Handle when the user presses Space or C
                float clampedInput = 0.5f + 0.5f * _cockpit.MoveIndicator.Y;
                if (clampedInput < -1) clampedInput = -1f;
                if (clampedInput > 1) clampedInput = 1f;

                accel = minAccel + (maxAccel - minAccel) * clampedInput;
                isManualInput = true;
            }
            else if (_landing)
            {
                // Handle when we want to land
                // We want to accelerate such that the current vertical velocity is equal to the constant -LANDING_VELOCITY
                // Note: thrusters will automatically cut off when altitude is less than LANDING_ALTITUDE_THRESHOLD
                accel = (float) (-LANDING_VELOCITY - slopeVelocity);
                if (accel < minAccel) accel = minAccel;
                if (accel > maxAccel) accel = maxAccel;
            }
            else if (altitude < minAltitude)
            {
                // Handle when we are too low (below the minimum altitude)
                if (slopeVelocity > 0)
                {
                    // We are already moving up
                    // Predict at what altitude we will stop if we start stopping now
                    // If we will stop over the minimum altitude, then it's time to stop
                    double t = -slopeVelocity / minAccel;
                    double y = altitude + slopeVelocity * t + 0.5 * minAccel * t * t;

                    accel = (y > minAltitude) ? minAccel : maxAccel;
                }
                else
                {
                    // We are moving down right now -- this is BAD!!!! We might crash!
                    accel = maxAccel;
                }
            }
            else if (altitude > maxAltitude)
            {
                // Handle when we are too high (above the maximum altitude)
                if (slopeVelocity < 0)
                {
                    // We are moving down
                    // Predict at what altitude we will stop if we start braking now
                    // If we will stop under the maximum altitude, then it's time to brake
                    double t = -slopeVelocity / maxAccel;
                    double y = altitude + slopeVelocity * t + 0.5 * maxAccel * t * t;

                    accel = (y < maxAltitude) ? maxAccel : minAccel;
                }
                else
                {
                    // We are moving up
                    // We don't want to move up, we're too high
                    accel = minAccel;
                }
            }
            else
            {
                // We are just between the minimum and maximum altitude
                // Try to accelerate such that y-velocity will be zero
                accel = -verticalVelocity;
                if (accel > maxAccel) accel = maxAccel;
                if (accel < minAccel) accel = minAccel;
            }

            // Count non-damaged thrusters
            int numFunctionalThrusters = 0;
            foreach (IMyThrust thrust in _thrusters)
            {
                if (thrust.IsFunctional)
                    numFunctionalThrusters++;
            }

            // Set the thrust override to achieve the desired acceleration
            float totalThrust = (accel + gravity) * mass;
            foreach (IMyThrust thrust in _thrusters)
            {
                if (!thrust.IsFunctional)
                    continue;

                float efficiency = thrust.MaxEffectiveThrust / thrust.MaxThrust;
                float thrustOverride = totalThrust / efficiency / thrustAlignment / numFunctionalThrusters;
                thrust.ThrustOverride = Math.Max(thrustOverride, 0.0000000001f);
            }

            // Turn off thrusters if we have landed
            foreach (IMyThrust thrust in _thrusters)
            {
                thrust.Enabled = !(_landing && altitude < LANDING_ALTITUDE_THRESHOLD);
            }

            // Update LCD (if any)
            foreach (IMyTextSurface display in _displays)
            {
                // Ignore keyboard on the programmable block
                if (Me.SurfaceCount >= 2 && display == Me.GetSurface(1))
                    continue;

                // Determine first two lines to write
                string line1 = (Math.Abs(forwardVelocity) > 5) ? forwardVelocity.ToString("F0") : forwardVelocity.ToString("F1");
                string line2 = (Math.Abs(sidewaysVelocity) > 5) ? sidewaysVelocity.ToString("F0") : sidewaysVelocity.ToString("F1");

                // Determine third line to write
                string line3 = "";
                if (_landing)
                {
                    line3 = "LAND";
                }
                else if (_isHangarMode && raycastedAltitude != null)
                {
                    line3 = raycastedAltitude.Value.ToString("F1");
                }
                else if (stoppingDistance >= 0 &&
                    Math.Abs(forwardAcceleration) > DISPLAY_STOPDIST_THRESHOLD_ACCEL &&
                    Math.Abs(forwardVelocity) > DISPLAY_STOPDIST_THRESHOLD_SPEED)
                {
                    line3 = stoppingDistance.ToString("F0");
                }

                // Debug difference between raycasted and elevation altitude
                if (DEBUG_RAYCAST_DIFF)
                    line3 = (raycastedAltitude != null && elevationAltitude != null) ?
                        (raycastedAltitude.Value - elevationAltitude.Value).ToString("F2") :
                        "none";

                //line2 = gravityDown.LengthSquared().ToString("F2"); // gravityDown magnitude is correct
                float diff = Vector3.Dot(-gravityDown, _bottomCameras[0]._cameraBlock.GetPosition() - _cockpit.GetPosition());
                line2 = diff.ToString("f2");

                _cockpit.CustomData = FormatGPS(_cockpit.GetPosition(), "scr Cockpit pos") + "\n" +
                    FormatGPS(_bottomCameras[0]._cameraBlock.GetPosition(), "scr Camera pos") + "\n" +
                    FormatGPS(_bottomCameras[0]._cameraBlock.GetPosition() + gravityDown * diff, "scr Camera pos cockpit level") + "\n" +
                    FormatGPS(Me.CubeGrid.GetPosition(), "scr CubeGrid GetPosition hopefully COM");

                // Write text onto display
                display.WriteText(line1 + '\n' + line2 + '\n' + line3);

                // If we would have normally used raycasted altitude, but the ship is tilted so far that we need to use elevation
                // altitude instead, this can result in inaccuracies (e.g. didn't detect the grid we are hovering over)
                // Warn the user if we would normally use raycasted altitude but can't/almost can't because we are tilted too far.
                bool isOverMaxAngle = false;
                bool isNearMaxAngle = false;
                if (_bottomCameras.Count > 0)
                {
                    float maxAngle = _bottomCamerasMaxAngle;
                    isOverMaxAngle = Math.Abs(shipPitch) >= maxAngle || Math.Abs(shipRoll) >= maxAngle;
                    isNearMaxAngle = Math.Abs(shipPitch) >= (maxAngle - 5) || Math.Abs(shipRoll) >= (maxAngle - 5);
                }

                // Determine text color of LCD panel
                // If we are in hangar mode, set the color to blue, otherwise set it to white
                if (_isHangarMode) display.FontColor = new Color(50, 150, 185);
                else display.FontColor = new Color(255, 255, 255);

                // Determine background color of LCD panel
                // If we are user controlled, flash purple
                if (isManualInput) display.BackgroundColor = (_totalTimeRan % 0.15 < 0.075) ? new Color(25, 0, 60) : new Color(30, 0, 50);
                // If maximum acceleration is too low, warn the user and set the background to red
                else if (maxAccel < 2f || isOverMaxAngle) display.BackgroundColor = new Color(75, 0, 0);
                // If maximum acceleration is almost too low, warn the user and set the background to yellow
                else if (maxAccel < 4f || isNearMaxAngle) display.BackgroundColor = new Color(40, 25, 0);
                // If we are going right too fast, set background to blue
                else if (sidewaysVelocity > 2f && !_isHangarMode) display.BackgroundColor = new Color(0, 0, (int) Math.Min(100, sidewaysVelocity * 10f));
                // If we are going left too fast, set background to green
                else if (sidewaysVelocity < -2f && !_isHangarMode) display.BackgroundColor = new Color(0, (int) Math.Min(100, -sidewaysVelocity * 10f), 0);
                // Otherwise, set background to black (transparent)
                else display.BackgroundColor = Color.Black;
            }
        }

        // Checks the current altitude of the ship using both raycasts (if available) and planet elevation (if available).
        // Returns a tuple where the first member is the raycasted altitude, and the second member is the elevation-determined altitude.
        ValueTuple<double?, double?> CheckAltitude(Vector3 gravDown, Vector3 shipDown)
        {
            // Ideally, check altitude using camera raycast
            // Go through each bottom camera and check if we haven't raycasted recently; if so, do another raycast
            foreach (ScanningCamera sc in _bottomCameras)
            {
                // Do a raycast if we haven't raycasted recently and the camera's distance limit is high enough
                double timeSinceLastRaycast = _totalTimeRan - (sc._recentRaycast?._time ?? -100);
                double rayDistLimit = sc._cameraBlock.RaycastDistanceLimit;
                if (timeSinceLastRaycast >= GROUND_RAYCAST_INTERVAL && (rayDistLimit >= MAX_DOWNWARDS_RAYCAST || rayDistLimit < 0))
                {
                    // Do the raycast!
                    MyDetectedEntityInfo raycast = sc._cameraBlock.Raycast(sc._cameraBlock.GetPosition() + gravDown * (float) MAX_DOWNWARDS_RAYCAST);
                    if (!raycast.IsEmpty() /*&& raycast.HitPosition != null*/)
                    {
                        // Got the result, store it into the ScanningCamera
                        float verticalDistFromCockpit = Vector3.Dot(-gravDown, sc._cameraBlock.GetPosition() - GetCenterOfMass()) * X;
                        RaycastResult result = new RaycastResult();
                        result._distance = Vector3.Distance(sc._cameraBlock.WorldMatrix.Translation, raycast.HitPosition.Value) - verticalDistFromCockpit;
                        result._time = _totalTimeRan;

                        sc._previousRaycast = sc._recentRaycast;
                        sc._recentRaycast = result;
                    }
                }
            }

            // Calculate the altitude using recently raycasted cameras.
            double? raycastedAltitude = null;
            foreach (ScanningCamera sc in _bottomCameras)
            {
                double? scResult = sc.ExtrapolateRaycast(_totalTimeRan);
                if (scResult != null)
                    raycastedAltitude = Math.Min(raycastedAltitude ?? double.MaxValue, scResult.Value);
            }

            // Calculate the altitude using cockpit altitude
            // (This will detect terrain but not blocks or other ships under the hoverbike)
            double? elevationAltitude = null;
            double elevationAltitude2;
            if (_cockpit.TryGetPlanetElevation(MyPlanetElevation.Surface, out elevationAltitude2))
                elevationAltitude = elevationAltitude2;

            // Return the results from both altitude checks
            return new ValueTuple<double?, double?>(raycastedAltitude, elevationAltitude);
        }

        // Resets the thrusters so that they are ready for normal controls.
        void ResetThrusters()
        {
            foreach (IMyThrust thrust in _thrusters)
            {
                thrust.ThrustOverride = 0f;
                thrust.Enabled = true;
            }
        }

        void Init()
        {
            _initialized = SubInit();

            if (_initialized) Echo("Script initialized successfully.");
            else Echo("Couldn't initialize script.");
        }

        bool SubInit()
        {
            // Find block group
            IMyBlockGroup group = GridTerminalSystem.GetBlockGroupWithName(_blockGroupName);
            if (group == null)
            {
                Echo($"Block group \"{_blockGroupName}\" not found.");
                return false;
            }

            // Initialize thrusters and check that there is at least 1
            _thrusters = new List<IMyThrust>();
            group.GetBlocksOfType(_thrusters, x => x.IsSameConstructAs(Me));
            if (_thrusters.Count == 0)
            {
                Echo("No thrusters were found in the group.");
                return false;
            }

            // Initialize cockpit and check that it is found
            List<IMyCockpit> cockpitList = new List<IMyCockpit>();
            group.GetBlocksOfType(cockpitList);
            if (cockpitList.Count != 1)
            {
                Echo("There must be exactly one cockpit in the group.");
                return false;
            }
            _cockpit = cockpitList[0];

            // Initialize displays
            _displays = new List<IMyTextSurface>();

            if (DISPLAY_ON_COCKPIT)
                _displays.Add(FixDisplayParams(_cockpit.GetSurface(0), 3f));
            if (DISPLAY_ON_PB)
                for (int i = 0; i < Me.SurfaceCount; i++)
                    _displays.Add(FixDisplayParams(Me.GetSurface(i)));

            List<IMyTextPanel> lcdList = new List<IMyTextPanel>();
            group.GetBlocksOfType(lcdList);
            foreach (IMyTextPanel lcd in lcdList)
            {
                _displays.Add(FixDisplayParams(lcd));
            }

            ResetDisplays();

            // Initialize cameras
            List<IMyCameraBlock> cameraList = new List<IMyCameraBlock>();
            group.GetBlocksOfType(cameraList);
            _forwardCameras = new List<ScanningCamera>(cameraList.Count);
            _bottomCameras = new List<ScanningCamera>(cameraList.Count);
            _bottomCamerasMaxAngle = float.MaxValue;

            foreach (IMyCameraBlock cam in cameraList)
            {
                // Detect forward-facing cameras
                if (cam.Orientation.Forward == _cockpit.Orientation.Forward)
                {
                    ScanningCamera sc = new ScanningCamera();
                    cam.Enabled = true;
                    cam.EnableRaycast = true;
                    sc._cameraBlock = cam;

                    Base6Directions.Direction camUp = cam.Orientation.Up;
                    CameraRotation rotation;
                    if (camUp == _cockpit.Orientation.Up) rotation = CameraRotation.NORMAL;
                    else if (camUp == Base6Directions.GetOppositeDirection(_cockpit.Orientation.Left)) rotation = CameraRotation.CLOCKWISE;
                    else if (camUp == _cockpit.Orientation.Left) rotation = CameraRotation.COUNTERCLOCKWISE;
                    else rotation = CameraRotation.UPSIDEDOWN;
                    sc._rotation = rotation;

                    _forwardCameras.Add(sc);
                }
                // Detect downwards-facing cameras
                if (cam.Orientation.Forward == Base6Directions.GetOppositeDirection(_cockpit.Orientation.Up))
                {
                    ScanningCamera sc = new ScanningCamera();
                    cam.Enabled = true;
                    cam.EnableRaycast = true;
                    sc._cameraBlock = cam;

                    Base6Directions.Direction camUp = cam.Orientation.Up;
                    CameraRotation rotation;
                    if (camUp == _cockpit.Orientation.Forward) rotation = CameraRotation.NORMAL;
                    else if (camUp == Base6Directions.GetOppositeDirection(_cockpit.Orientation.Left)) rotation = CameraRotation.CLOCKWISE;
                    else if (camUp == _cockpit.Orientation.Left) rotation = CameraRotation.COUNTERCLOCKWISE;
                    else rotation = CameraRotation.UPSIDEDOWN;
                    sc._rotation = rotation;

                    _bottomCameras.Add(sc);
                    _bottomCamerasMaxAngle = Math.Min(_bottomCamerasMaxAngle, cam.RaycastConeLimit);
                }
            }

            if (_forwardCameras.Count > 0)
            {
                ScanningCamera sc = _forwardCameras[0];
                float raycastPitch = 0f;
                float raycastYaw = 30f;
                sc.TransformRotation(ref raycastPitch, ref raycastYaw);

                MyDetectedEntityInfo raycast = sc._cameraBlock.Raycast(30.0, raycastPitch, raycastYaw);
                Echo("Raycast result: " + raycast.Name);
            }

            ComputeLocalCenterOfMass();
            _cockpit.CustomData = FormatGPS(GetCenterOfMass(), "Center of mass");

            return true;
        }

        void LoadCustomData()
        {
            string data = Me.CustomData;

            string[] lines = data.Split('\n');
            if (lines.Length >= 2)
            {
                // CustomData is valid, load the data
                _blockGroupName = lines[1];

                // Load controlling/landing/hangarmode state
                if (lines.Length >= 3 && lines[2].Length >= 3)
                {
                    _controlling = (lines[2][0] == 'y');
                    _landing = (lines[2][1] == 'y');
                    _isHangarMode = (lines[2][2] == 'y');
                }
            }
            else
            {
                // CustomData is invalid, load default data
                _blockGroupName = DEFAULT_BLOCK_GROUP;

                _controlling = false;
                _landing = false;
                _isHangarMode = false;
            }

            SaveCustomData();
        }

        // Computes the grid-local center of mass of this ship and stores it in _localCenterOfMass.
        void ComputeLocalCenterOfMass()
        {
            // TODO: For some reason this algorithm is slightly inaccurate, the mass is different from the PhysicalMass and the actual
            // location returned is also slightly off.

            IMyCubeGrid grid = Me.CubeGrid;

            Vector3I min = grid.Min;
            Vector3I max = grid.Max;

            // Determine the total mass of the grid
            float totalMass = 0f;
            for (int z = min.Z; z <= max.Z; z++)
            {
                for (int y = min.Y; y <= max.Y; y++)
                {
                    for (int x = min.X; x <= max.X; x++)
                    {
                        IMySlimBlock block = grid.GetCubeBlock(new Vector3I(x, y, z));
                        if (block != null)
                            totalMass += GetBlockMass(block);
                    }
                }
            }

            // Determine center of mass
            Vector3 centerOfMass = new Vector3();
            for (int z = min.Z; z <= max.Z; z++)
            {
                for (int y = min.Y; y <= max.Y; y++)
                {
                    for (int x = min.X; x <= max.X; x++)
                    {
                        IMySlimBlock block = grid.GetCubeBlock(new Vector3I(x, y, z));
                        if (block != null)
                        {
                            centerOfMass += new Vector3(x, y, z) * GetBlockMass(block) / totalMass;
                        }
                    }
                }
            }

            // Store center of mass
            _localCenterOfMass = centerOfMass;
            _comPhysicalMass = _cockpit.CalculateShipMass().PhysicalMass;
        }

        // Calculates the center of mass in world coordinates.
        Vector3 GetCenterOfMass() =>
            Vector3.Transform(Me.CubeGrid.GridSize * _localCenterOfMass, Me.CubeGrid.WorldMatrix);

        float GetBlockMass(IMySlimBlock block)
        {
            float mass = block.FatBlock?.Mass ?? block.Mass;
            if (block is IMyEntity)
            {
                IMyEntity entity = (IMyEntity) block;
                for (int i = 0; i < entity.InventoryCount; i++)
                    mass += (float) entity.GetInventory(i).CurrentMass;
                if (entity.HasInventory)
                    mass += (float) entity.GetInventory().CurrentMass;
            }
            return mass / GetBlockVolume(block);
        }

        int GetBlockVolume(IMySlimBlock block)
        {
            if (block.FatBlock != null)
            {
                Vector3I dimensions = block.FatBlock.Max - block.FatBlock.Min + Vector3I.One;
                return dimensions.X * dimensions.Y * dimensions.Z;
            }
            return 1;
        }

        void SaveCustomData()
        {
            string state = "";
            state += (_controlling ? 'y' : 'n');
            state += (_landing ? 'y' : 'n');
            state += (_isHangarMode ? 'y' : 'n');

            Me.CustomData = VERSION + '\n' + _blockGroupName + '\n' + state;
        }

        void ResetDisplays()
        {
            foreach (IMyTextSurface display in _displays)
            {
                display.BackgroundColor = Color.Black;
                display.WriteText("");
            }
        }

        IMyTextSurface FixDisplayParams(IMyTextSurface display, float fontSize = 5.0f)
        {
            if (AUTOMATICALLY_SET_LCD_PARAMS)
            {
                display.ContentType = ContentType.TEXT_AND_IMAGE;
                display.Alignment = TextAlignment.CENTER;
                display.FontColor = new Color(150, 150, 150);
                display.Font = "Monospace";
                display.FontSize = fontSize;
            }
            return display;
        }

        float ToDeg(float rad) => rad * 57.2957795131f;

        float ToRad(float deg) => deg / 57.2957795131f;

        double Clamp(double a, double b, double x) => Math.Max(a, Math.Min(b, x));

        new void Echo(object obj)
        {
            base.Echo(obj != null ? obj.ToString() : "null");
        }

        string FormatGPS(Vector3 position, string gpsName) =>
            $"GPS:{gpsName}:{position.X.ToString("F5")}:{position.Y.ToString("F5")}:{position.Z.ToString("F5")}:";

        string FormatGPS(Vector3D position, string gpsName) =>
            $"GPS:{gpsName}:{position.X.ToString("F5")}:{position.Y.ToString("F5")}:{position.Z.ToString("F5")}:";

        // Describes the successful (hit) result of a raycast.
        struct RaycastResult
        {
            // The distance from the hit raycast
            public double _distance;
            // The time that this raycast was performed (see _totalTimeRan)
            public double _time;
        }

        class ScanningCamera
        {
            // The camera block this scanning camera uses
            public IMyCameraBlock _cameraBlock;

            // The rotation of this camera relative to the cockpit, from the perspective of looking forward in the cockpit
            public CameraRotation _rotation;

            // The most recent raycast performed
            public RaycastResult? _recentRaycast;

            // The second-most recent raycast performed
            public RaycastResult? _previousRaycast;

            // Based on the results from recent raycasts, estimates the distance that we'd get if we did a raycast right now.
            public double? ExtrapolateRaycast(double currentTime)
            {
                // Ensure that we have at least one recent raycast
                if (_recentRaycast != null && (currentTime - _recentRaycast.Value._time) < GROUND_RAYCAST_EXPIRY)
                {
                    if (_previousRaycast != null && (currentTime - _previousRaycast.Value._time) < 2 * GROUND_RAYCAST_EXPIRY)
                    {
                        // We have two recent raycasts. Perform a linear extrapolation of data to estimate current raycast value
                        RaycastResult r1 = _previousRaycast.Value;
                        RaycastResult r2 = _recentRaycast.Value;
                        double m = (r2._distance - r1._distance) / (r2._time - r1._time);
                        return r2._distance + m * (currentTime - r2._time);
                    }
                    else
                    {
                        // There is only one recent raycast, so just return the most recent one (no extrapolation possible)
                        return _recentRaycast.Value._distance;
                    }
                }

                // No recent raycasts, there is nothing to go off of
                return null;
            }

            public void TransformRotation(ref float pitch, ref float yaw)
            {
                float oldPitch = pitch;
                switch (_rotation)
                {
                    case CameraRotation.NORMAL:
                        break;
                    case CameraRotation.CLOCKWISE:
                        pitch = yaw;
                        yaw = -oldPitch;
                        break;
                    case CameraRotation.UPSIDEDOWN:
                        pitch = -pitch;
                        yaw = -yaw;
                        break;
                    case CameraRotation.COUNTERCLOCKWISE:
                        pitch = -yaw;
                        yaw = oldPitch;
                        break;
                }
            }
        }

        enum CameraRotation
        {
            NORMAL,
            CLOCKWISE,
            UPSIDEDOWN,
            COUNTERCLOCKWISE
        }

        struct ValueTuple<A, B>
        {
            public A a;
            public B b;

            public ValueTuple(A a, B b)
            {
                this.a = a;
                this.b = b;
            }
        }
    }
}
