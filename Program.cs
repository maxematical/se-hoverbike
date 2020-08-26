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

        // If this is true, the script display on the programmable block's LCD screen, as well as any other LCDs in use. The
        // programmable block does not need to be added to the Hoverbike group.
        // Default value: false
        private const bool DISPLAY_ON_PB = false;

        // If this is true, the script will display its output on one of the cockpit's LCD screen as well. Similar to DISPLAY_ON_PB.
        // Default value: false
        private const bool DISPLAY_ON_COCKPIT = false;

        // Normally, the script will automatically set the parameters of the LCD panel to make sure the text is easily visible.
        // If this is false it will not do so.
        // Default value: true
        private const bool AUTOMATICALLY_SET_LCD_PARAMS = false;//TODO

        // The more this value is, the earlier the hoverbike will brake when it's high in the air. When braking early, it will keep
        // using maximal thrust if necessary, but otherwise will use less thrust to avoid stopping too early. Setting this value to 0
        // will disable the safe falling feature.
        // Default value: 3.0
        private const double SAFE_FALLING = 3.0;

        // The vertical speed that the script will try to maintain when landing.
        // Default value: 3.5
        private const double LANDING_VELOCITY = 2.5;

        // If the ship is landing, the script will automatically turn off all the thrusters when the ship's altitude is at or
        // below this number.
        // Default value: 2.0
        private const double LANDING_ALTITUDE_THRESHOLD = 1.0;

        // The minimum altitude the script will try to maintain while in hangar mode, in meters. Hangar mode can be activated by
        // running the argument "Hangar" and is turned off by running "Auto".
        // Default value: 1.0
        private const double HANGAR_MODE_MIN_ALTITUDE = 1.0;

        // The maximum altitude the script will try to maintain while in hangar mode, in meters. Hangar mode can be activated by
        // running the argument "Hangar" and is turned off by running "Auto".
        // Default value: 1.5
        private const double HANGAR_MODE_MAX_ALTITUDE = 1.5;

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
        private const double GROUND_RAYCAST_INTERVAL = 0.125;

        // (Advanced) How long, in seconds, the result of a raycast will be used before it is considered outdated and not used
        // anymore. In this case the script will fall back to the default altitude algorithms. See GROUND_RAYCAST_INTERVAL for more
        // info.
        // Default value: (2 * GROUND_RAYCAST_INTERVAL)
        private const double GROUND_RAYCAST_EXPIRY = 2 * GROUND_RAYCAST_INTERVAL;

        // The maximum distance, in meters, that downwards-facing cameras will raycast when trying to determine the height of the
        // ground. See GROUND_RAYCAST_INTERVAL for more info.
        // Default value: 150.0
        private const double MAX_DOWNWARDS_RAYCAST = 150.0;

        // When the ship is slowing down and its deceleration at least this value, then the ship's stopping distance will be
        // displayed on the third line of the LCD panel.
        // Default value: 2.0
        private const double DISPLAY_STOPDIST_THRESHOLD_ACCEL = 2.0;

        // When the ship is slowing down and its speed is greater than this value, then the ship's stopping distance will be
        // displayed on the third line of the LCD panel.
        // Default value: 5.0
        private const double DISPLAY_STOPDIST_THRESHOLD_SPEED = 5.0;

        // The script will only update once every X frames.
        // Defualt value: 2
        private const int UPDATE_INTERVAL = 2;

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

        private const string VERSION = "1.2_dev";
        private const string DEFAULT_BLOCK_GROUP = "Hoverbike";

        private string _blockGroupName;

        // Blocks and stuff needed for the script, set up in the Init() function
        private bool _initialized;
        private IMyCockpit _cockpit;
        private List<IMyThrust> _thrusters;
        private List<ScanningCamera> _forwardCameras;
        private List<ScanningCamera> _bottomCameras;
        private float _bottomCamerasMaxAngle;
        private List<IMyGyro> _gyros;
        private List<IMyTextSurface> _displays;
        private IMyMotorStator _dockingRotor;

        // Control variables (public to save state between script runs)
        private bool _controlling;
        private bool _landing;
        private bool _isHangarMode;
        private Vector3? _dockingLocation;

        private double _altitudeOffset;
        private Vector2D _autopilotDesiredSpeed;

        // Display variables
        private LcdMessage? _temporaryMessage;
        private string _echoMessage;

        private bool _hasEchoed;

        // Flight subsystems
        private Info _info;
        private SlopeMeasurer _slopeMeasurer;
        private AltitudeProvider _raycastAltitudeProvider;
        private AltitudeProvider _elevationAltitudeProvider;
        private WasdInputHandler _input;
        private PerformanceStats _perf;

        private double _totalTimeRan;
        private int _nUpdates;
        private double _dt;

        public Program()
        {
            _perf = new PerformanceStats();
            _nUpdates = 0;
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
            _hasEchoed = false;

            // Handle continuous update
            if ((updateSource & UpdateType.Update1) != 0)
            {
                _dt += Runtime.TimeSinceLastRun.TotalSeconds;
                if (_nUpdates++ % UPDATE_INTERVAL > 0)
                    return;

                if (_echoMessage != null)
                {
                    Echo(_echoMessage, false);
                }
                if (_initialized && _controlling)
                {
                    DoUpdate();
                    Echo($"---\n" +
                        $"Performance stats:\n" +
                        $"- current runtime: {Runtime.LastRunTimeMs:F2}ms\n" +
                        $"- average runtime: {_perf.avgRuntimeMs:F2}ms\n" +
                        $"- maximum runtime: {_perf.maxRuntimeMs:F2}ms", false);
                    _perf.Update(Runtime.LastRunTimeMs, _totalTimeRan);
                }
                _totalTimeRan += _dt;
                _dt = 0.0;
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
                _dockingLocation = null;
            }
            if (argument == "stop")
            {
                _controlling = false;
                if (_initialized)
                {
                    ResetThrusters();
                    ResetGyros();
                    ResetDisplays();
                }
            }
            if (argument == "land")
            {
                _landing = true;
                _controlling = true;
                _dockingLocation = null;
            }
            if (argument == "hangar")
            {
                _controlling = true;
                _landing = false;
                _isHangarMode = true;
                _autopilotDesiredSpeed = new Vector2D();
                _dockingLocation = null;
            }
            if (argument == "higher" || argument == "lower" || argument == "checkoffset" || argument == "offset0")
            {
                if (argument == "higher")
                    _altitudeOffset += ALTITUDE_OFFSET_INCREMENT;   
                else if (argument == "lower")
                    _altitudeOffset -= ALTITUDE_OFFSET_INCREMENT;
                else if (argument == "offset0")
                    _altitudeOffset = 0.0;

                float duration = (argument == "checkoffset") ? 5.0f : 2.0f;

                _altitudeOffset = Clamp(0f, MAX_ALTITUDE_OFFSET, _altitudeOffset);
                _temporaryMessage = new LcdMessage($"+{_altitudeOffset.ToString("F1")}m", (float) _totalTimeRan, 2.0f);
            }
            if (argument.StartsWith("dock "))
            {
                Vector3? point = TryParseGPS(argument.Substring("dock ".Length));
                if (_dockingRotor == null)
                {
                    Echo("No docking rotor");
                }
                else if (point != null)
                {
                    _dockingLocation = point;
                    Echo("Set docking location to the provided GPS point. Navigating to GPS.");
                }
                else
                {
                    Echo("Invalid GPS point. Please copy the GPS to clipboard and try again.");
                }
            }
        }

        void DoUpdate()
        {
            // Determine ship characteristics
            MyShipMass myMass = _cockpit.CalculateShipMass();
            float mass = myMass.PhysicalMass;

            MyShipVelocities myVelocity = _cockpit.GetShipVelocities();
            Vector3 velocity = myVelocity.LinearVelocity;

            // https://forum.keenswh.com/threads/7386434/#post-1286997268
            // X = pitch (positive up)
            // Y = yaw (positive left)
            // Z = roll (positive left)
            Vector3 angularVelocity = Vector3.Transform(myVelocity.AngularVelocity, Matrix.Transpose(_cockpit.WorldMatrix.GetOrientation()));

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
            Vector3 shipUp = -shipDown;

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

            // Update info, to be used by different subsystems
            _info.angularVelocity = angularVelocity;
            _info.cockpit = _cockpit;
            _info.forwardVelocity = forwardVelocity;
            _info.gravity = gravity;
            _info.gravityDown = gravityDown;
            _info.gravityUp = -gravityDown;
            _info.groundForward = groundForward;
            _info.groundRight = groundRight;
            _info.horizontalSpeed = horizontalVelocity.Length();
            _info.planetCenter = planetCenter;
            _info.sealevelRadius = sealevelRadius;
            _info.shipForward = shipForward;
            _info.shipPitch = shipPitch;
            _info.shipRoll = shipRoll;
            _info.shipUp = shipUp;
            _info.sidewaysVelocity = sidewaysVelocity;
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

            _info.surfaceVelocity = altitudeData.Value.surfaceVelocity;

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
                    verticalVelocity - _info.slopeRate, horizontalVelocity.Length(),
                    minAccel, safeAccel, maxAccel);
                accel = result.a;
                isManualInput = result.b;
            }

            // Land if there's no one in the cockpit!
            if (!_cockpit.IsUnderControl && _info.horizontalSpeed > 2.0f)
                _landing = true;

            // Determine whether we need autopilot
            _input.Update(_cockpit, _totalTimeRan);
            bool isAutopilot = false;
            bool displayAutopilotSpeed = false;
            bool displayAsDocking = false;
            if (_dockingLocation != null && _dockingRotor != null)
            {
                // Autopilot to the docking location
                isAutopilot = true;
                displayAsDocking = true;
                accel = UpdateDockingAutopilot();
            }
            else if (_landing)
            {
                // Stop the hoverbike
                isAutopilot = true;
                _autopilotDesiredSpeed = new Vector2D();
            }
            else if (_isHangarMode)
            {
                // We are in WASD mode
                isAutopilot = true;
                displayAutopilotSpeed = true;
                UpdateWasdControls();
            }
            _input.PostUpdate(_totalTimeRan, _dt);

            // Update gyroscopes, perform autopilot calculations if enabled
            if (isAutopilot) accel = ApplyAutopilotTurning(accel);
            else ResetGyros();

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

            // Update LCD (if any)
            foreach (IMyTextSurface display in _displays)
            {
                // Ignore keyboard on the programmable block
                if (Me.SurfaceCount >= 2 && display == Me.GetSurface(1))
                    continue;

                // Determine first two lines to write
                double dispForwardVelocity  = displayAutopilotSpeed ? _autopilotDesiredSpeed.Y : forwardVelocity;
                double dispSidewaysVelocity = displayAutopilotSpeed ? _autopilotDesiredSpeed.X : sidewaysVelocity;
                string line1 = (Math.Abs(dispForwardVelocity) > 5) ? dispForwardVelocity.ToString("F0") : dispForwardVelocity.ToString("F1");
                string line2 = (Math.Abs(dispSidewaysVelocity) > 5) ? dispSidewaysVelocity.ToString("F0") : dispSidewaysVelocity.ToString("F1");

                // Determine third line to write
                string line3 = "";
                if (_temporaryMessage.HasValue && (_totalTimeRan - _temporaryMessage.Value.initTime) < _temporaryMessage.Value.duration)
                {
                    line3 = _temporaryMessage.Value.message;
                }
                else if (displayAsDocking)
                {
                    line3 = "DOCK";
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

        float ApplyAutopilotTurning(float accel)
        {
            // Determined desired rotations, using math formulas
            double dvs = Vector3.Dot(_info.surfaceVelocity, _info.groundRight) + _autopilotDesiredSpeed.X - _info.sidewaysVelocity;
            double dvf = Vector3.Dot(_info.surfaceVelocity, _info.groundForward) + _autopilotDesiredSpeed.Y - _info.forwardVelocity;

            double desiredRoll = 0.0;
            double desiredPitch = 0.0;
            double maxDesiredAngle = 20.0;

            /**
             * The following code determines the ideal angle given the current velocity, so that if the ship follows this rotation
             * over time it will be brought to a complete halt (without wobbling).
             * In practice, however, the ship can't turn instantly, and the mathematical model does not account for that, so it will
             * have a bit of inaccuracy/wobbling as it moves.
             * Also, the model doesn't account for changes in thrust power due to hills/slopes, but this is ok because WASD is designed
             * to mostly be used in bases anyways.
             * 
             * Derivation of the formula:   (Done by maaaxaltan, you can use this too but please credit me first)
             * ==========================
             * We will first describe a hypothetical scenario where the ship undergoes a linear rotation along one axis (e.g. either pitch
             * or roll). Then, we will use the ship's rotation to predict how it accelerates and its actual velocity as time goes on.
             * We can finally use this scenario in the "real" Space Engineers by mapping the ship's current velocity to the model rotation.
             * 
             * Say the ship starts at time t=0 and rotation r(t)=r1, and over time interval [0,t1] seconds, rotates linearly from a rotation
             * of r1 to 0 radians. (Note: "rotation" can refer to either roll or pitch)
             * 
             * Thus we have the formula for the ship's current rotation, given the time in seconds:
             *   r(t) = r1 - (r1/t1) * t
             * 
             * Next we will determine the current acceleration of the ship given its rotation.
             * The total acceleration needed to keep the ship from falling can be calculated with "g / cos(theta)", m and g are mass and
             * gravitation acceleration, respectively, and theta is the ship's current angle between the ground plane (i.e. 0 degrees when
             * level with the ground, 90 degrees when pointing straight up/down).
             * However, what we really want is to determine its acceleration laterally. In this case the acceleration laterally can be determined
             * by multiplying the acceleration by the sine of the angle, so the lateral acceleration can be computed using "g * sin(theta) / cos(theta)".
             * 
             * Simplifying and using r(t) as the rotation, we have the ship's acceleration:
             *   a(t) = g * tan(r(t))
             * 
             * Now we will integrate its acceleration to determine its velocity.
             * Due to how we are using this model, we only need to solve for velocity in terms of rotation, not in terms of time, which makes the
             * integration slightly easier.
             * 
             * Velocity is simply the integral of acceleration over time, so we can say v(t) = fnint(a(t), t, 0, t)
             * To save space, all the steps of solving the integral are not written here. But the final result of the integration is
             *   v(t) = g*t1/r1 * ln|cos(r(t))|.
             * 
             * If the ship matches its rotation exactly with r(t) over time, and keeps the acceleration imparted by the thrusters exactly at
             * "g / cos(r(t))", then we can expect it to follow the velocity curve given by v(t).
             * 
             * In practice, we want the ship to get its rotation from its velocity, not the other way around. Having it follow this hypothetical
             * scenario exactly would not work because its velocity would get slightly off from things like slopes or turning slightly too slowly.
             * Therefore, we will solve for desired rotation in terms of v(t) and avoid having the desired rotation be based off the previous definition
             * of r(t) above.
             * 
             * Solving v(t) for rotation, we get:
             *   r(t) = acos(exp( (r1*v(t))/(g*t1) ))
             * which is the formula used by the code.
             */
            if (Math.Abs(dvs) > 0.01)
            {
                double r1 = 10.0;
                double t1 = 2.0;
                double g = _info.gravity;

                desiredRoll = Math.Acos(Math.Exp((r1 * Math.PI / 180.0) * -Math.Abs(dvs) / (g * t1))) * 180.0 / Math.PI;
                desiredRoll *= Math.Sign(dvs);
            }
            if (Math.Abs(dvf) > 0.01)
            {
                double r1 = 10.0;
                double t1 = 1.5;
                double g = _info.gravity;

                desiredPitch = Math.Acos(Math.Exp((r1 * Math.PI / 180.0) * -Math.Abs(dvf) / (g * t1))) * 180.0 / Math.PI;
                desiredPitch *= -Math.Sign(dvf);
            }

            desiredRoll = Clamp(-maxDesiredAngle, maxDesiredAngle, desiredRoll);
            desiredPitch = Clamp(-maxDesiredAngle, maxDesiredAngle, desiredPitch);

            // Reset roll/pitch to zero when ship is moving very slowly
            double rs = 20.0; // speed at which to roll
            double ps = 20.0; // speed at which to pitch

            if (Math.Abs(dvs) < 0.005 && Math.Abs(_info.shipRoll) < 0.5)
            {
                desiredRoll = 0.0;
                rs = 2.0;
            }
            if (Math.Abs(dvf) < 0.005 && Math.Abs(_info.shipPitch) < 0.5)
            {
                desiredPitch = 0.0;
                ps = 2.0;
            }

            // Calculate desired eulers to send to the gyroscopes
            double allowedYawSpeed = Lerp(10f, 2f, _info.horizontalSpeed / 5f + Math.Abs(_info.shipPitch / 20f) + Math.Abs(_info.shipRoll / 20f));

            double baseRollSpeed = Math.Sign(desiredRoll - _info.shipRoll) * Math.Min(1.0, Math.Abs((desiredRoll - _info.shipRoll) / rs));
            double basePitchSpeed = Math.Sign(desiredPitch - _info.shipPitch) * Math.Min(2.0, Math.Abs((desiredPitch - _info.shipPitch) / ps));
            double baseYawSpeed = _input._CurrentYaw * allowedYawSpeed - -_info.angularVelocity.Y;

            // Adjust eulers to spin around planet-based axes instead of local axes
            Vector3 transformedSpeeds = TransformEulerDeltas(new Vector3(_info.shipPitch, 0f, _info.shipRoll),
                new Vector3(basePitchSpeed, baseYawSpeed, baseRollSpeed),
                _info.shipForward, _info.shipUp, _info.groundForward, _info.gravityUp);

            // Send to gyros
            ApplyGyroOverride(transformedSpeeds.X, transformedSpeeds.Y, transformedSpeeds.Z, _gyros, _cockpit);
            return accel;
        }

        void UpdateWasdControls()
        {
            // Various settings
            Vector2D maxWasdSpeed = new Vector2D(8.0, 30.0);
            Vector2D maxWasdAccel = new Vector2D(4.0, 6.0);
            Vector2D wasdDoubletapSpeed = new Vector2D(4.0, 10.0);
            double wasdSpeedCutoff = 2.0;

            // Update desired speed
            // Increase/decrease speed when keys pressed, faster if the key has been held down for at least 0.5s
            _autopilotDesiredSpeed.X += maxWasdAccel.X * (_input._InputDuration.X > 0.5 ? 1.0 : 0.5) * _input._Current.X * _dt;
            _autopilotDesiredSpeed.Y += maxWasdAccel.Y * (_input._InputDuration.Y > 0.5 ? 1.0 : 0.5) * _input._Current.Y * _dt;

            // Reset speed when a) it is +-2m/s or less, b) pressed key in the opposite direction last tick, and c) not pressing key anymore
            if (_input._Last.X != 0 && Math.Sign(_input._Last.X) != Math.Sign(_autopilotDesiredSpeed.X) &&
                    _input._Current.X == 0 && Math.Abs(_autopilotDesiredSpeed.X) < wasdSpeedCutoff)
                _autopilotDesiredSpeed.X = 0.0;
            if (_input._Last.Y != 0 && Math.Sign(_input._Last.Y) != Math.Sign(_autopilotDesiredSpeed.Y) &&
                    _input._Current.Y == 0 && Math.Abs(_autopilotDesiredSpeed.Y) < wasdSpeedCutoff)
                _autopilotDesiredSpeed.Y = 0.0;

            // Double tapping gives either a speed boost if done in the same direction as current velocity, or resets speed for the opposite direction
            if (_input._IsDoublepressX)
            {
                _autopilotDesiredSpeed.X = (Math.Sign(_input._Actual.X) == Math.Sign(_autopilotDesiredSpeed.X)) ?
                    wasdDoubletapSpeed.X * Math.Sign(_input._Actual.X) :
                    0.0;
            }
            if (_input._IsDoublepressY)
            {
                _autopilotDesiredSpeed.Y = (Math.Sign(_input._Actual.Y) == Math.Sign(_autopilotDesiredSpeed.Y)) ?
                    wasdDoubletapSpeed.Y * Math.Sign(_input._Actual.Y) :
                    0.0;
            }

            // Cap desired speed
            _autopilotDesiredSpeed.X = Clamp(-maxWasdSpeed.X, maxWasdSpeed.X, _autopilotDesiredSpeed.X);
            _autopilotDesiredSpeed.Y = Clamp(-maxWasdSpeed.Y, maxWasdSpeed.Y, _autopilotDesiredSpeed.Y);
        }

        float UpdateDockingAutopilot()
        {
            Vector3 dockWith = _dockingLocation.Value + 0.05f * _info.gravityUp;
            Vector3 delta = dockWith - _dockingRotor.GetPosition();

            float verticalDelta = Vector3.Dot(delta, _info.gravityUp);
            Vector3 horizontalDelta = delta - verticalDelta * _info.gravityUp;

            float desiredVerticalDelta = Lerp(0f, -2.5f, horizontalDelta.Length() / 0.5f + _info.horizontalSpeed / 0.2f);
            float desiredVerticalSpeed = -Math.Sign(desiredVerticalDelta - verticalDelta) * Math.Min(1f, Math.Abs(desiredVerticalDelta - verticalDelta));
            float accel = (desiredVerticalSpeed - _info.verticalVelocity);

            Vector3 localHorizontalDelta = new Vector3(Vector3.Dot(_info.groundRight, horizontalDelta), 0f, Vector3.Dot(_info.groundForward, horizontalDelta));
            _autopilotDesiredSpeed.X = Math.Sign(localHorizontalDelta.X) * Clamp(0f, 10f, Math.Abs(localHorizontalDelta.X / 2f));
            _autopilotDesiredSpeed.Y = Math.Sign(localHorizontalDelta.Z) * Clamp(0f, 10f, Math.Abs(localHorizontalDelta.Z / 2f));

            // Try to attach
            // For some reason, this makes the rotor unusable
            if (horizontalDelta.LengthSquared() < 0.0001f && verticalDelta < 0.1f && false)
            {
                _dockingRotor.Attach();
            }
            if (_dockingRotor.IsAttached)
            {
                _dockingLocation = null;
            }

            return accel;
        }

        // Transforms desired euler angles to change so that they are relative to planet axes rather than local axes.
        // For example, normally applying a yaw rotation while pitch is non-zero will cause a change in the ship's roll.
        // This adjusts the euler angles so this does not happen.
        Vector3 TransformEulerDeltas(Vector3 shipEulers, Vector3 deltas,
            Vector3 shipForward, Vector3 shipUp, Vector3 groundForward, Vector3 gravityUp)
        {
            float shipPitch = ToRad(shipEulers.X);
            float shipRoll = ToRad(shipEulers.Z);

            float pitchSpeed = ToRad(deltas.X);
            float yawSpeed  = ToRad(deltas.Y);
            float rollSpeed = ToRad(deltas.Z);

            Quaternion q =
                        Quaternion.CreateFromAxisAngle(Vector3.Forward, shipRoll + rollSpeed) *
                        Quaternion.CreateFromAxisAngle(Vector3.Right, shipPitch + pitchSpeed) *
                        Quaternion.CreateFromAxisAngle(Vector3.Up, yawSpeed) *
                        Quaternion.Inverse(Quaternion.CreateFromForwardUp(shipForward, shipUp)) *
                        Quaternion.CreateFromForwardUp(groundForward, gravityUp);
            Vector3 rotateAxis;
            float rotateAngle;
            q.GetAxisAngle(out rotateAxis, out rotateAngle);

            Vector3 r = GetEulerAngles(rotateAxis, rotateAngle);
            return new Vector3(ToDeg(r.Z), ToDeg(r.Y), -ToDeg(r.X));
        }

        Vector3 GetEulerAngles(Vector3 axis, float radians)
        {
            // http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
            float x = axis.X;
            float y = axis.Y;
            float z = axis.Z;
            float a = radians;

            float sinA = (float) Math.Sin(a);
            float cosA = (float) Math.Cos(a);

            return new Vector3((float) Math.Asin(x * y * (1f - cosA) + z * sinA),
                (float) Math.Atan2(y * sinA - x * z * (1 - cosA), 1 - (y * y + z * z) * (1 - cosA)),
                (float) Math.Atan2(x * sinA - y * z * (1 - cosA), 1 - (x * x + z * z) * (1 - cosA)));
        }
        
        ValueTuple<float, bool> CalculateDesiredAccel(double altitude, double minAltitude, double maxAltitude,
            double slopeVelocity, float horizontalVelocity,
            float minAccel, float safeAccel, float maxAccel)
        {
            if (Math.Abs(_cockpit.MoveIndicator.Y) > 0.1f)
            {
                // Handle when the user presses Space or C
                float input = 0.5f + 0.5f * _cockpit.MoveIndicator.Y;
                float accel = Lerp(minAccel, maxAccel, input);
                return new ValueTuple<float, bool>(accel, true);
            }
            else if (_landing && altitude < LANDING_ALTITUDE_THRESHOLD)
            {
                // When landing, cut off thrusters below a certain altitude
                return new ValueTuple<float, bool>(minAccel, false);
            }
            else if (_landing && (horizontalVelocity < 0.5f || altitude < 2 * LANDING_ALTITUDE_THRESHOLD))
            {
                // We are landing, and aren't moving too much or near enough to the ground that it doesn't matter
                // Lower the ship at the speed designated by LANDING_VELOCITY
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

            // Initialize gyros
            _gyros = new List<IMyGyro>();
            group.GetBlocksOfType(_gyros);
            if (_gyros.Count == 0)
                Echo("Note: No gyros were found in the block group. If you would like to use WASD Controls or Auto-Dock, " +
                    "add the gyroscopes into the block group.");

            // Initialize rotor
            List<IMyMotorStator> rotorList = new List<IMyMotorStator>();
            group.GetBlocksOfType(rotorList);
            if (rotorList.Count > 1)
                Echo("Warning: Multiple docking rotors were added to the block group, but only one of them will be used for docking.");
            _dockingRotor = rotorList.Count > 0 ? rotorList[0] : null;

            // Setup subsystems
            _info = new Info();
            _slopeMeasurer = new SlopeMeasurer();
            _raycastAltitudeProvider = new RaycastAltitudeProvider(_bottomCameras, _cockpit.Orientation.Forward);
            _elevationAltitudeProvider = new ElevationAltitudeProvider();
            _input = new WasdInputHandler();

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
                else
                {
                    _controlling = false;
                    _landing = false;
                    _isHangarMode = false;
                }

                // Load altitude offset
                if (lines.Length >= 4 && double.TryParse(lines[3], out _altitudeOffset))
                {
                    _altitudeOffset = Clamp(0.0, MAX_ALTITUDE_OFFSET, _altitudeOffset);
                }
                else
                {
                    _altitudeOffset = 0.0;
                }

                // Load WASD speed
                bool loadedWasdSpeed = false;
                if (lines.Length >= 5)
                {
                    string[] split = lines[4].Split(' ');
                    if (split.Length == 2)
                    {
                        double x;
                        double y;
                        if (double.TryParse(split[0], out x) && double.TryParse(split[1], out y))
                        {
                            _autopilotDesiredSpeed = new Vector2D(x, y);
                            loadedWasdSpeed = true;
                        }
                    }
                }
                if (!loadedWasdSpeed)
                {
                    _autopilotDesiredSpeed = new Vector2D();
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

            Me.CustomData = VERSION + '\n'                              // Line 1: version
                + _blockGroupName + '\n' +                              // Line 2: name of block group
                state + '\n' +                                          // Line 3: current control state (controlling, landing, hangar)
                _altitudeOffset + '\n' +                                // Line 4: altitude offset
                $"{_autopilotDesiredSpeed.X:F2} {_autopilotDesiredSpeed.Y:F2}\n"; // Line 5: WASD desired speed
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

        static float Lerp(float a, float b, float t) => a + (b - a) * Clamp(0f, 1f, t);

        static Vector3 Lerp(Vector3 a, Vector3 b, float t) => a + (b - a) * Clamp(0f, 1f, t);

        new void Echo(object obj, bool appendToMessage = true)
        {
            string msg = obj != null ? obj.ToString() : "null";
            base.Echo(msg);

            if (appendToMessage)
            {
                if (!_hasEchoed)
                {
                    _hasEchoed = true;
                    _echoMessage = "";
                }
                _echoMessage += msg + '\n';
            }
        }

        static string FormatGPS(Vector3 position, string gpsName) =>
            $"GPS:{gpsName}:{position.X.ToString("F5")}:{position.Y.ToString("F5")}:{position.Z.ToString("F5")}:";

        static string FormatGPS(Vector3D position, string gpsName) =>
            $"GPS:{gpsName}:{position.X.ToString("F5")}:{position.Y.ToString("F5")}:{position.Z.ToString("F5")}:";

        static Vector3? TryParseGPS(string gps)
        {
            string[] split = gps.Split(':');
            if (split.Length < 5)
                return null;

            string xStr = split[2];
            string yStr = split[3];
            string zStr = split[4];

            float x, y, z;
            if (float.TryParse(xStr, out x) && float.TryParse(yStr, out y) && float.TryParse(zStr, out z))
                return new Vector3(x, y, z);
            else
                return null;
        }

        //Whip's ApplyGyroOverride Method v9 - 8/19/17
        static void ApplyGyroOverride(double pitch_speed, double yaw_speed, double roll_speed, List<IMyGyro> gyro_list, IMyTerminalBlock reference)
        {
            var rotationVec = new Vector3D(-pitch_speed, yaw_speed, roll_speed); //because keen does some weird stuff with signs 
            var shipMatrix = reference.WorldMatrix;
            var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);

            foreach (var thisGyro in gyro_list)
            {
                var gyroMatrix = thisGyro.WorldMatrix;
                var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));

                thisGyro.Pitch = (float) transformedRotationVec.X;
                thisGyro.Yaw = (float) transformedRotationVec.Y;
                thisGyro.Roll = (float) transformedRotationVec.Z;
                thisGyro.GyroOverride = true;
            }
        }

        void ResetGyros()
        {
            foreach (IMyGyro gyro in _gyros)
            {
                gyro.GyroOverride = false;
            }
        }

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
            public Vector3 surfaceVelocity;

            public AltitudeData(float altitude, float groundHeight, float time, Vector3 surfaceVelocity)
            {
                this.altitude = altitude;
                this.groundHeight = groundHeight;
                this.time = time;
                this.surfaceVelocity = surfaceVelocity;
            }
        }

        class Info
        {
            public Vector3 gravityDown;
            public Vector3 gravityUp;
            public Vector3 planetCenter;
            public float gravity;

            public float verticalVelocity;
            public float forwardVelocity;
            public float sidewaysVelocity;
            public float horizontalSpeed;

            public Vector3 surfaceVelocity;
            public Vector3 angularVelocity;

            public Vector3 shipUp;
            public Vector3 shipForward;
            public Vector3 groundForward;
            public Vector3 groundRight;

            public float shipPitch;
            public float shipRoll;

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

                            // Determine the speed of what we are hovering over
                            Vector3 surfaceVelocity = raycast.Velocity;

                            // Store the calculated altitude data
                            _previousRaycasts[i] = new AltitudeData(altitude, groundHeight, info.time, surfaceVelocity);

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

                    return new AltitudeData(alt, gnd, info.time, data.surfaceVelocity);
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
                    return new AltitudeData((float) altitude, groundHeight, info.time, Vector3.Zero);
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

        class WasdInputHandler
        {
            public Vector2 _Current { get; private set; }

            public float _CurrentYaw { get; private set; }

            public Vector2 _Actual { get; private set; }

            public Vector2 _Last { get; private set; }
            public Vector2D _InputDuration { get; private set; }

            public bool _IsDoublepressX { get; private set; }
            public bool _IsDoublepressY { get; private set; }

            private Vector2 _LastNonzeroInput { get; set; }
            private Vector2D _LastNonzeroInputTime { get; set; }
            private Vector2D _LastDoublepressTime { get; set; }

            public void Update(IMyCockpit cockpit, double currentTime)
            {
                Vector2 current = new Vector2(cockpit.RollIndicator, -cockpit.MoveIndicator.Z);
                _Actual = current;
                if (currentTime - _LastDoublepressTime.X < 0.25) current.X = 0;
                if (currentTime - _LastDoublepressTime.Y < 0.25) current.Y = 0;
                _Current = current;

                _CurrentYaw = cockpit.MoveIndicator.X;

                _IsDoublepressX = PopDoublePress(0, currentTime);
                _IsDoublepressY = PopDoublePress(1, currentTime);
            }

            public void PostUpdate(double currentTime, double dt)
            {
                // Update how long the inputs have been pressed
                Vector2D inputDuration = _InputDuration;
                if (_Actual.X == _Last.X) inputDuration.X += dt; else inputDuration.X = 0.0;
                if (_Actual.Y == _Last.Y) inputDuration.Y += dt; else inputDuration.Y = 0.0;
                _InputDuration = inputDuration;

                // Update last nonzero input
                Vector2 lastNonzeroInput = _LastNonzeroInput;
                Vector2D lastNonzeroInputTime = _LastNonzeroInputTime;

                if (_Actual.X == 0 && _Last.X != 0)
                {
                    lastNonzeroInput.X = _Last.X;
                    lastNonzeroInputTime.X = currentTime;
                }
                if (_Actual.Y == 0 && _Last.Y != 0)
                {
                    lastNonzeroInput.Y = _Last.Y;
                    lastNonzeroInputTime.Y = currentTime;
                }

                _LastNonzeroInput = lastNonzeroInput;
                _LastNonzeroInputTime = lastNonzeroInputTime;

                // Update last input
                _Last = _Actual;
            }

            private bool PopDoublePress(int component, double currentTime)
            {
                if (component < 0 || component > 1) throw new ArgumentException("Component must be 0 or 1 (x or y)");

                // Check double press
                bool doublepress = _Actual[component] != 0 && // key is down
                    _Actual[component] == _LastNonzeroInput[component] && // key was previously down
                    (currentTime - _LastNonzeroInputTime[component]) < 0.125; // key was down recently

                // If double press, clear the double press so this won't return true again until another double press
                if (doublepress)
                {
                    var v = _LastDoublepressTime;
                    v[component] = currentTime;
                    _LastDoublepressTime = v;

                    var w = _LastNonzeroInput;
                    w[component] = 0f;
                    _LastNonzeroInput = w;
                }

                return doublepress;
            }
        }

        class PerformanceStats
        {
            public double maxRuntimeMs;
            public double maxRuntimeTime;
            public double avgRuntimeMs;

            private double nextAvgRuntime;
            private int numberAvgRuntime;
            private double nextAvgRuntimeStart;

            public void Update(double ms, double time)
            {
                // Update average runtime
                nextAvgRuntime += ms;
                numberAvgRuntime++;
                if (time - nextAvgRuntimeStart > 1.0)
                {
                    avgRuntimeMs = nextAvgRuntime / numberAvgRuntime;
                    nextAvgRuntime = 0.0;
                    numberAvgRuntime = 0;
                    nextAvgRuntimeStart = time;
                }

                // Update max runtime
                if (ms > maxRuntimeMs || (time - maxRuntimeTime) > 10.0)
                {
                    maxRuntimeMs = ms;
                    maxRuntimeTime = time;
                }
            }
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
