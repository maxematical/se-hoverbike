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

        // The minimum altitude this script will normally try to maintain, in meters. If the ship is below this altitude, it will
        // fly up.
        // Default value: 3.5
        private const double NORMAL_MIN_ALTITUDE = 3.5;

        // The maximum altitude this script will normally try to maintain, in meters. If the ship is above this altitude, it will
        // fly down.
        // Default value: 7.0
        private const double NORMAL_MAX_ALTITUDE = 7.0;

        // The minimum altitude the script will try to maintain while in hangar mode, in meters. Hangar mode can be activated by
        // running the argument "Hangar" and is turned off by running "Auto".
        // Default value: 1.0
        private const double HANGAR_MODE_MIN_ALTITUDE = 1.0;

        // The maximum altitude the script will try to maintain while in hangar mode, in meters. Hangar mode can be activated by
        // running the argument "Hangar" and is turned off by running "Auto".
        // Default value: 1.5
        private const double HANGAR_MODE_MAX_ALTITUDE = 1.5;

        // If this is true, the script display on the programmable block's LCD screen, as well as any other LCDs in use. The
        // programmable block does not need to be added to the Hoverbike group.
        // Default value: false
        private const bool DISPLAY_ON_PB = false;

        // If this is true, the script will display its output on one of the cockpit's LCD screen as well. Similar to DISPLAY_ON_PB.
        // Default value: false
        private const bool DISPLAY_ON_COCKPIT = false;

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
        private const bool AUTOMATICALLY_SET_LCD_PARAMS = false;//TODO

        // When the ship is slowing down and its deceleration at least this value, then the ship's stopping distance will be
        // displayed on the third line of the LCD panel.
        // Default value: 2.0
        private const double DISPLAY_STOPDIST_THRESHOLD_ACCEL = 2.0;

        // When the ship is slowing down and its speed is greater than this value, then the ship's stopping distance will be
        // displayed on the third line of the LCD panel.
        // Default value: 5.0
        private const double DISPLAY_STOPDIST_THRESHOLD_SPEED = 5.0;

        // How much the target altitude of the ship should change when running the script with "Higher" or "Lower", in meters.
        // Default value: 2.0
        private const double ALTITUDE_OFFSET_INCREMENT = 2.0;

        // The maximum offset to target altitude that can be obtained by running "Higher" or "Lower", in meters.
        // Default value: 8.0
        private const double MAX_ALTITUDE_OFFSET = 8.0;

        // (Advanced) To calculate the slope of the ground, the script will automatically check the height of the ground every
        // few meters. This value means how many meters the script should wait before checking the height of the ground again.
        // Default value: 5.0
        private const double SLOPE_SAMPLE_RATE = 5.0;

        // (Advanced) Similar to SLOPE_SAMPLE_RATE, this is the interval in seconds between calculating the slope.
        // Default value: 0.5
        private const double SLOPE_SAMPLE_INTERVAL = 0.5;

        // If the ship has bottom-facing cameras, the script will perform use these cameras to perform a raycast (scan) several
        // times every second to check the height of the ground. This is the interval, in seconds, between each raycast. A
        // smaller interval will result in more raycasts, which means more accurate height measurements, but makes the script
        // more processing-heavy and could cause lag. (Note: the interval at which cameras can raycast may also be limited by
        // the server's settings)
        // Default value: 0.125
        private const double GROUND_RAYCAST_INTERVAL = 0.25;

        // (Advanced) How long, in seconds, the result of a raycast will be used before it is considered outdated and not used
        // anymore. In this case the script will fall back to the default altitude algorithms. See GROUND_RAYCAST_INTERVAL for more
        // info.
        // Default value: (2 * GROUND_RAYCAST_INTERVAL)
        private const double GROUND_RAYCAST_EXPIRY = 2 * GROUND_RAYCAST_INTERVAL;

        // The maximum distance, in meters, that downwards-facing cameras will raycast when trying to determine the height of the
        // ground. See GROUND_RAYCAST_INTERVAL for more info.
        // Default value: 150.0
        private const double MAX_DOWNWARDS_RAYCAST = 150.0;

        // The more this value is, the earlier the hoverbike will brake when it's high in the air. When braking early, it will keep
        // using maximal thrust if necessary, but otherwise will use less thrust to avoid stopping too early. Setting this value to 0
        // will disable the safe falling feature.
        // Default value: 3.0
        private const double SAFE_FALLING = 3.0;

        // ==============================================================================
        // DEBUG CONFIGURATION
        // These are likely not useful unless you're editing the script or told to change
        // them by a developer
        // ==============================================================================

        // If this is true, slope and altitude data will be shown on the LCD screen. Reduce text size and disable
        // AUTOMATICALLY_SET_LCD_PARAMS to see it
        private const bool DEBUG_SLOPE_AND_ALTITUDE = false;

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

        private double _altitudeOffset;

        // Display variables
        private LcdMessage? _temporaryMessage;

        // Flight subsystems
        private Info _info;
        private SlopeMeasurer _slopeMeasurer;
        private AltitudeProvider _raycastAltitudeProvider;
        private AltitudeProvider _elevationAltitudeProvider;

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
                }
                _totalTimeRan += Runtime.TimeSinceLastRun.TotalSeconds;
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
            if (argument == "higher" || argument == "lower" || argument == "checkoffset")
            {
                double offsetMul;
                if (argument == "higher") offsetMul = 1.0;
                else if (argument == "lower") offsetMul = -1.0;
                else offsetMul = 0.0;

                _altitudeOffset += ALTITUDE_OFFSET_INCREMENT * offsetMul;
                _altitudeOffset = Clamp(0f, MAX_ALTITUDE_OFFSET, _altitudeOffset);

                _temporaryMessage = new LcdMessage($"+{_altitudeOffset.ToString("F1")}m", (float) _totalTimeRan, 2.0f);
            }
        }

        void DoUpdate()
        {
            // Determine ship characteristics
            MyShipMass myMass = _cockpit.CalculateShipMass();
            float mass = myMass.PhysicalMass;

            MyShipVelocities myVelocity = _cockpit.GetShipVelocities();
            Vector3 velocity = myVelocity.LinearVelocity;

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

            // Determine components of velocity relative to various axes
            float verticalVelocity = Vector3.Dot(velocity, -gravityDown);
            Vector3 horizontalVelocity = velocity + verticalVelocity * gravityDown;
            float forwardVelocity = Vector3.Dot(velocity, groundForward);
            float sidewaysVelocity = Vector3.Dot(velocity, groundRight);

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
            float safeAccel = Math.Max(maxAccel - (float) SAFE_FALLING, 4f);

            // Determine planet center
            Vector3D planetCenter;
            if (!_cockpit.TryGetPlanetPosition(out planetCenter))
            {
                Echo("Not in gravity well");
                return;
            }

            // Determine radius of sea level
            float sealevelRadius = 60000f; // TODO Configurable

            // Update info
            _info.cockpit = _cockpit;
            _info.gravityDown = gravityDown;
            _info.gravityUp = -gravityDown;
            _info.planetCenter = planetCenter;
            _info.sealevelRadius = sealevelRadius;
            _info.slopeRate = _slopeMeasurer._Slope * horizontalVelocity.Length();
            _info.time = (float) _totalTimeRan;
            _info.verticalVelocity = verticalVelocity;

            // Determine current altitude
            AltitudeData? altitudeData = _raycastAltitudeProvider.GetAltitude(_slopeMeasurer, _info) ??
                _elevationAltitudeProvider.GetAltitude(_slopeMeasurer, _info);
            if (!altitudeData.HasValue)
            {
                Echo("Couldn't determine the ship's altitude");
                return;
            }
            float altitude = altitudeData.Value.altitude;
            float groundHeight = altitudeData.Value.groundHeight;

            // TODO: Enable forward scanning
            //DoScanAhead(velocity, gravityDown, altitude);

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
            minAltitude += _altitudeOffset;
            maxAltitude += _altitudeOffset;

            // OK, we know everything we need to know to start doing the autopilot thingy
            // Determine desired net vertical acceleration (we'll take gravity into account later)
            float accel;
            bool isManualInput;
            {
                var result = CalculateDesiredAccel(altitude, minAltitude, maxAltitude,
                    verticalVelocity - _info.slopeRate,
                    minAccel, safeAccel, maxAccel);
                accel = result.a;
                isManualInput = result.b;
            }

            // Count non-damaged thrusters
            int numFunctionalThrusters = _thrusters.Count(x => x.IsFunctional);

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
                if (_temporaryMessage.HasValue && (_totalTimeRan - _temporaryMessage.Value.initTime) < _temporaryMessage.Value.duration)
                {
                    line3 = _temporaryMessage.Value.message;
                }
                else if (_landing)
                {
                    line3 = "LAND";
                }
                else if (_isHangarMode)
                {
                    line3 = altitude.ToString("F1");
                }
                else if (stoppingDistance >= 0 &&
                    Math.Abs(forwardAcceleration) > DISPLAY_STOPDIST_THRESHOLD_ACCEL &&
                    Math.Abs(forwardVelocity) > DISPLAY_STOPDIST_THRESHOLD_SPEED)
                {
                    line3 = stoppingDistance.ToString("F0");
                }

                // Debug slope and altitude
                if (DEBUG_SLOPE_AND_ALTITUDE)
                    line3 = "s" + _slopeMeasurer._Slope.ToString("F3") +
                        "\na" + altitude.ToString("F1");

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

        ValueTuple<float, bool> CalculateDesiredAccel(double altitude, double minAltitude, double maxAltitude,
            double slopeVelocity,
            float minAccel, float safeAccel, float maxAccel)
        {
            if (Math.Abs(_cockpit.MoveIndicator.Y) > 0.1f)
            {
                // Handle when the user presses Space or C
                float clampedInput = 0.5f + 0.5f * _cockpit.MoveIndicator.Y;
                if (clampedInput < -1) clampedInput = -1f;
                if (clampedInput > 1) clampedInput = 1f;

                float accel = minAccel + (maxAccel - minAccel) * clampedInput;
                return new ValueTuple<float, bool>(accel, true);
            }
            else if (_landing)
            {
                // Handle when we want to land
                // We want to accelerate such that the current vertical velocity is equal to the constant -LANDING_VELOCITY
                // Note: thrusters will automatically cut off when altitude is less than LANDING_ALTITUDE_THRESHOLD
                float accel = Clamp(minAccel, maxAccel, (float) (-LANDING_VELOCITY - slopeVelocity));
                return new ValueTuple<float, bool>(accel, false);
            }
            else if (altitude < minAltitude)
            {
                // Handle when we are too low (below the minimum altitude)
                float accel;
                if (slopeVelocity > 0)
                {
                    // We are already moving up
                    // Predict at what altitude we will stop if we start thrusting down now
                    // If we will stop over the minimum altitude, then it's time to stop
                    double y = EstimateStoppingDistance(altitude, slopeVelocity, minAccel);
                    accel = (y > minAltitude) ? minAccel : maxAccel;
                }
                else
                {
                    // We are moving down right now -- this is BAD!!!! We might crash!
                    accel = maxAccel;
                }
                return new ValueTuple<float, bool>(accel, false);
            }
            else if (altitude > maxAltitude)
            {
                // Handle when we are too high (above the maximum altitude)
                float accel;
                if (slopeVelocity < 0)
                {
                    // We are moving down
                    // Predict at what altitude we will stop if:
                    //   a) we start braking with a smaller acceleration (safeAccel)
                    //   b) we start braking with a larger acceleration (maxAccel)
                    double y1 = EstimateStoppingDistance(altitude, slopeVelocity, safeAccel);
                    double y2 = EstimateStoppingDistance(altitude, slopeVelocity, maxAccel);

                    // If we won't stop at the desired altitude even when using maximum thrust, this could be bad. Put thrusters to the max!
                    if (y2 < maxAltitude) accel = maxAccel;
                    // If we won't stop at the desired altitude if using safe thrust, this is probably still ok but use safe thrust
                    else if (y1 < maxAltitude) accel = safeAccel;
                    // Otherwise we can keep falling
                    else accel = minAccel;
                }
                else
                {
                    // We are moving up
                    // We don't want to move up, we're too high
                    accel = minAccel;
                }
                return new ValueTuple<float, bool>(accel, false);
            }
            else
            {
                // We are just between the minimum and maximum altitude
                // Try to accelerate such that y-velocity will be zero
                float accel = Clamp(minAccel, maxAccel, -5f * (float) slopeVelocity);
                return new ValueTuple<float, bool>(accel, false);
            }
        }

        // TODO: Implement forward scanning
        //void DoScanAhead(Vector3 velocity, Vector3 gravityDown, double altitude)
        //{
        //    // Scan 1.5s ahead
        //    float secondsAhead = 1.5f;
        //    float distanceAhead = 20f;
        //    Vector3 horizontalDirection = velocity - gravityDown * Vector3.Dot(velocity, gravityDown);
        //    horizontalDirection.Normalize();

        //    //Vector3 expectedDelta = (velocity + -gravityDown * (float) (_measuredSlope) * horizontalVelocity.Length()) * secondsAhead + gravityDown * (float) altitude;
        //    Vector3 raycastDelta = horizontalDirection * distanceAhead +
        //        (-gravityDown) * (float) _slope * distanceAhead +
        //        gravityDown * (float) altitude;
        //    raycastDelta *= 1.1f;
        //    float raycastDistance = raycastDelta.Length();

        //    // Find camera to do the raycast with
        //    ScanningCamera useCamera = null;
        //    foreach (ScanningCamera sc in _forwardCameras)
        //    {
        //        if (sc._cameraBlock.RaycastDistanceLimit < 0 || sc._cameraBlock.RaycastDistanceLimit >= raycastDistance)
        //        {
        //            useCamera = sc;
        //            break;
        //        }
        //    }
        //    if (useCamera == null)
        //        return; // no camera found

        //    // Perform the raycast
        //    MyDetectedEntityInfo result = useCamera._cameraBlock.Raycast(useCamera._cameraBlock.GetPosition() + raycastDelta);
        //    if (!result.IsEmpty() && result.HitPosition != null)
        //    {
        //        float hitDistanceUpwards = Vector3.Dot(result.HitPosition.Value - useCamera._cameraBlock.GetPosition(), -gravityDown) + (float) altitude;
        //        Vector3 hitHorizontalDelta = distanceAhead * horizontalDirection;// result.HitPosition.Value + gravityDown * hitDistanceUpwards - useCamera._cameraBlock.GetPosition();
        //        _raycastedSlope = hitDistanceUpwards / hitHorizontalDelta.Length();
        //        _lastSlopeScanTime = _totalTimeRan;
        //    }
        //    else
        //    {
        //        _raycastedSlope -= 0.001f;
        //    }

        //    //_cockpit.CustomData = $"Raycasted ahead {raycastDistance.ToString("F1")},\n" +
        //    //    $"  Horizontal velocity: {horizontalVelocity.Length().ToString("F1")},\n" +
        //    //    $"  Found? {!result.IsEmpty()}\n\n";
        //    _cockpit.CustomData = "";
        //    _cockpit.CustomData += FormatGPS(useCamera._cameraBlock.GetPosition(), "scr Camera Pos") + "\n";
        //    _cockpit.CustomData += FormatGPS(useCamera._cameraBlock.GetPosition() + raycastDelta, "scr Raycast To") + "\n";
        //    if (result.HitPosition != null) _cockpit.CustomData += FormatGPS(result.HitPosition.Value, "scr Raycast Hit") + "\n";
        //    _cockpit.CustomData += FormatGPS(useCamera._cameraBlock.GetPosition() + -gravityDown * (float) altitude, "scr Ground Pos");

        //    _cockpit.CustomData += $"\n(Altitude {altitude.ToString("F1")}\n";
        //    //_cockpit.CustomData += FormatGPS(useCamera._cameraBlock.GetPosition() + horizontalVelocity * secondsAhead, "scr Horiz Prediction") + "\n";
        //}

        // Resets the thrusters so that they are ready for normal controls.
        void ResetThrusters()
        {
            foreach (IMyThrust thrust in _thrusters)
            {
                thrust.ThrustOverride = 0f;
                thrust.Enabled = true;
            }
        }

        double EstimateStoppingDistance(double pos, double vel, float acc)
        {
            double t = -vel / acc;
            return pos + vel * t + 0.5 * acc * t * t;
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

                    _forwardCameras.Add(sc);
                }
                // Detect downwards-facing cameras
                if (cam.Orientation.Forward == Base6Directions.GetOppositeDirection(_cockpit.Orientation.Up))
                {
                    ScanningCamera sc = new ScanningCamera();
                    cam.Enabled = true;
                    cam.EnableRaycast = true;
                    sc._cameraBlock = cam;

                    _bottomCameras.Add(sc);
                    _bottomCamerasMaxAngle = Math.Min(_bottomCamerasMaxAngle, cam.RaycastConeLimit);
                }
            }

            _info = new Info();
            _slopeMeasurer = new SlopeMeasurer();
            _raycastAltitudeProvider = new RaycastAltitudeProvider(_bottomCameras, _cockpit.Orientation.Forward);
            _elevationAltitudeProvider = new ElevationAltitudeProvider();

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

        static float ToDeg(float rad) => rad * 57.2957795131f;

        static float ToRad(float deg) => deg / 57.2957795131f;

        static float Clamp(float a, float b, float x) => Math.Max(a, Math.Min(b, x));

        static double Clamp(double a, double b, double x) => Math.Max(a, Math.Min(b, x));

        static double Lerp(double a, double b, double t) => a + (b - a) * Clamp(0.0, 1.0, t);

        static Vector3 Lerp(Vector3 a, Vector3 b, float t) => a + (b - a) * Clamp(0f, 1f, t);

        new void Echo(object obj)
        {
            base.Echo(obj != null ? obj.ToString() : "null");
        }

        static string FormatGPS(Vector3 position, string gpsName) =>
            $"GPS:{gpsName}:{position.X.ToString("F5")}:{position.Y.ToString("F5")}:{position.Z.ToString("F5")}:";

        static string FormatGPS(Vector3D position, string gpsName) =>
            $"GPS:{gpsName}:{position.X.ToString("F5")}:{position.Y.ToString("F5")}:{position.Z.ToString("F5")}:";

        class SlopeMeasurer
        {
            public float _Slope { get; private set; }

            private float? lastGroundHeight;
            private Vector3? lastSeaLevelPos;
            private float lastUpdateTime;

            public SlopeMeasurer()
            {
                Reset();
            }

            public void Reset()
            {
                _Slope = 0f;
                lastGroundHeight = null;
                lastSeaLevelPos = null;
                lastUpdateTime = float.MinValue;
            }

            public void PushMeasurements(float groundHeight, Vector3 seaLevelPos, float time, bool forceUpdate = false)
            {
                if (lastGroundHeight.HasValue && lastSeaLevelPos.HasValue)
                {
                    float distanceBetweenSamples = Vector3.Distance(seaLevelPos, lastSeaLevelPos.Value);
                    bool shouldUpdateSlope = forceUpdate ||
                        (distanceBetweenSamples >= SLOPE_SAMPLE_RATE) ||
                        ((time - lastUpdateTime) >= SLOPE_SAMPLE_INTERVAL);

                    if (shouldUpdateSlope && distanceBetweenSamples >= 0.1f)
                    {
                        _Slope = (groundHeight - lastGroundHeight.Value) / distanceBetweenSamples;
                        lastGroundHeight = groundHeight;
                        lastSeaLevelPos = seaLevelPos;
                        lastUpdateTime = time;

                        if (_Slope < 0)
                        {
                            _Slope *= 0.5f;
                        }
                    }
                }
                else
                {
                    lastGroundHeight = groundHeight;
                    lastSeaLevelPos = seaLevelPos;
                    lastUpdateTime = time;
                }
            }
        }

        struct AltitudeData
        {
            public float altitude;
            public float groundHeight;
            public float time;

            public AltitudeData(float altitude, float groundHeight, float time)
            {
                this.altitude = altitude;
                this.groundHeight = groundHeight;
                this.time = time;
            }
        }

        class Info
        {
            public Vector3 gravityDown;
            public Vector3 gravityUp;
            public Vector3 planetCenter;
            public float verticalVelocity;
            public float time;
            // Distance from planet center to sea level
            public float sealevelRadius;
            // How fast the ground is rising in m/s
            public float slopeRate;

            public IMyCockpit cockpit;
        }

        interface AltitudeProvider
        {
            // Gets altitude data from the altitude provider. This method will push data to the slope measurer if necessary
            AltitudeData? GetAltitude(SlopeMeasurer slopeMeasurer, Info info);
        }

        class RaycastAltitudeProvider : AltitudeProvider
        {
            private List<ScanningCamera> _bottomCameras;
            private AltitudeData?[] _previousRaycasts;
            private int? _frontmostCameraIndex;
            private Base6Directions.Direction _cockpitForward;

            public RaycastAltitudeProvider(List<ScanningCamera> bottomCameras, Base6Directions.Direction cockpitForward)
            {
                Reset(bottomCameras, cockpitForward);
            }

            void Reset(List<ScanningCamera> bottomCameras, Base6Directions.Direction cockpitForward)
            {
                _bottomCameras = bottomCameras;
                _previousRaycasts = new AltitudeData?[bottomCameras.Count];
                _frontmostCameraIndex = CalcFrontmostCameraIndex(bottomCameras, cockpitForward);
                _cockpitForward = cockpitForward;
            }

            public AltitudeData? GetAltitude(SlopeMeasurer slopeMeasurer, Info info)
            {
                // Do raycasts, inform slope measurer of the measured altitude if necessary
                DoRaycasts(slopeMeasurer, info);

                // Predict the next altitude for each of the scanning cameras,
                // and return the data from the camera with the smallest altitude
                AltitudeData? min = null;
                for (int i = 0; i < _bottomCameras.Count; i++)
                {
                    AltitudeData? cameraData = PredictAltitudeForCamera(i, info);

                    if (cameraData.HasValue && (!min.HasValue || min.Value.altitude > cameraData.Value.altitude))
                        min = cameraData;
                }

                return min;
            }

            private void DoRaycasts(SlopeMeasurer slopeMeasurer, Info info)
            {
                for (int i = 0; i < _bottomCameras.Count; i++)
                {
                    ScanningCamera sc = _bottomCameras[i];

                    // Do a raycast if we haven't raycasted recently and the camera's distance limit is high enough
                    float timeSinceLastRaycast = info.time - (_previousRaycasts[i]?.time ?? float.MinValue); // TODO Stagger raycasts from cameras
                    double rayDistLimit = sc._cameraBlock.RaycastDistanceLimit;
                    if (timeSinceLastRaycast >= GROUND_RAYCAST_INTERVAL && (rayDistLimit >= MAX_DOWNWARDS_RAYCAST || rayDistLimit < 0))
                    {
                        // Do the raycast!
                        MyDetectedEntityInfo raycast = sc._cameraBlock.Raycast(sc._cameraBlock.GetPosition() + info.gravityDown * (float) MAX_DOWNWARDS_RAYCAST);
                        if (!raycast.IsEmpty())
                        {
                            // The raycast hit something!
                            // Calculate the altitude, based on the result of the raycast
                            float altitude = Vector3.Distance(sc._cameraBlock.WorldMatrix.Translation, raycast.HitPosition.Value);

                            // Calculate the ground height
                            Vector3 pos = sc._cameraBlock.GetPosition();
                            Vector3 sealevelPos = GetSealevelPos(info, pos);
                            float groundHeight = GetGroundHeight(info, sealevelPos, pos, altitude);

                            // Store the calculated altitude data
                            _previousRaycasts[i] = new AltitudeData(altitude, groundHeight, info.time);

                            // Tell slope measurer about this result (but only for ONE of the cameras, it would be confused if we kept giving it
                            // results from multiple)
                            int? idx = CheckFrontmostCameraIndex();
                            if (i == idx)
                            {
                                slopeMeasurer.PushMeasurements(groundHeight, sealevelPos, info.time, true);
                            }
                            else if (idx == null)
                            {
                                slopeMeasurer.Reset();
                            }
                        }
                    }
                }
            }

            private AltitudeData? PredictAltitudeForCamera(int scIndex, Info info)
            {
                // Ensure that we have at least one recent raycast
                if (_previousRaycasts[scIndex].HasValue) {
                    AltitudeData data = _previousRaycasts[scIndex].Value;
                    float t = info.time - data.time;

                    // Ensure that the data is recent enough
                    if (t >= GROUND_RAYCAST_EXPIRY)
                        return null;

                    // Predict the current altitude and ground height
                    float alt = data.altitude;
                    float gnd = data.groundHeight;

                    // Factor vertical speed and slope into the prediction
                    // *Note: both vertical speed and slope are 100% accurate in this case. Vertical speed can obviously be measured precisely
                    // and slope is only updated using actual raycast results (not estimated results)
                    float expectedAltitudeChange = (info.verticalVelocity - info.slopeRate) * t;
                    alt += expectedAltitudeChange;
                    gnd += expectedAltitudeChange;

                    return new AltitudeData(alt, gnd, info.time);
                }

                // No recent raycasts, there is nothing to go off of
                return null;
            }

            private int? CheckFrontmostCameraIndex()
            {
                if (!_frontmostCameraIndex.HasValue || !_bottomCameras[_frontmostCameraIndex.Value]._cameraBlock.IsFunctional)
                    _frontmostCameraIndex = CalcFrontmostCameraIndex(_bottomCameras, _cockpitForward);
                
                return _frontmostCameraIndex;
            }

            private static float GetGroundHeight(Info info, Vector3 sealevelPos, Vector3 pos, float altitude)
            {
                float heightAboveSeaLevel = Vector3.Dot(info.gravityUp, pos - sealevelPos);
                return heightAboveSeaLevel - altitude;
            }

            private static Vector3 GetSealevelPos(Info info, Vector3 pos)
            {
                Vector3 delta = pos - info.planetCenter;
                delta.Normalize();
                return info.planetCenter + delta * info.sealevelRadius;
            }

            private static int? CalcFrontmostCameraIndex(List<ScanningCamera> bottomCameras,
                Base6Directions.Direction cockpitForward)
            {
                int? frontmostCameraIndex = null;
                float frontmostCameraCoordinate = float.MinValue;
                for (int i = 0; i < bottomCameras.Count; i++)
                {
                    if (!bottomCameras[i]._cameraBlock.IsFunctional)
                        continue;

                    float coord = GetCoordinate(bottomCameras[i]._cameraBlock, cockpitForward);
                    if (coord > frontmostCameraCoordinate)
                    {
                        frontmostCameraIndex = i;
                        frontmostCameraCoordinate = coord;
                    }
                }

                return frontmostCameraIndex;
            }

            private static float GetCoordinate(IMyTerminalBlock block, Base6Directions.Direction dir) =>
                Vector3.Dot(block.Position * block.CubeGrid.GridSize, Base6Directions.GetVector(dir));
        }

        class ElevationAltitudeProvider : AltitudeProvider
        {
            public AltitudeData? GetAltitude(SlopeMeasurer slopeMeasurer, Info info)
            {
                double altitude;
                double sealevel;
                if (info.cockpit.TryGetPlanetElevation(MyPlanetElevation.Surface, out altitude) &&
                    info.cockpit.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out sealevel))
                {
                    float groundHeight = (float) (sealevel - altitude);
                    Vector3 sealevelPos = info.cockpit.GetPosition() + info.gravityDown * (float) sealevel;
                    slopeMeasurer.PushMeasurements(groundHeight, sealevelPos, info.time);
                    return new AltitudeData((float) altitude, groundHeight, info.time);
                }
                else
                {
                    return null;
                }
            }
        }

        class ScanningCamera
        {
            // The camera block this scanning camera uses
            public IMyCameraBlock _cameraBlock;
        }

        struct LcdMessage
        {
            public string message;
            public float initTime;
            public float duration;

            public LcdMessage(string message, float initTime, float duration)
            {
                this.message = message;
                this.initTime = initTime;
                this.duration = duration;
            }
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
