﻿using Sandbox.Game.EntityComponents;
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

        // The minimum altitude, in meters, this script will try to maintain. If the ship is below this altitude, it will fly up.
        // Default value: 3.5
        private const double MIN_ALTITUDE = 3.5;

        // The maximum altitude, in meters, this script will try to maintain. If the ship is above this altitude, it will fly down.
        // Default value: 7.0
        private const double MAX_ALTITUDE = 7.0;

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
        
        // ==========================================================================
        // Don't change anything below this line (unless you want to edit the script)
        // ==========================================================================

        private const string VERSION = "1.0";
        private const string DEFAULT_BLOCK_GROUP = "Hoverbike";

        private string _blockGroupName;

        private bool _initialized;
        private IMyCockpit _cockpit;
        private List<IMyThrust> _thrusters;
        private IMyTextSurface _maybeLcd;

        private bool _controlling;
        private bool _landing;

        private double _slope;
        private Vector3 _lastSeaLevelPos;
        private double _lastGroundHeight;

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
            }
            if (argument == "stop")
            {
                _controlling = false;

                if (_initialized)
                    ResetThrusters();

                if (_maybeLcd != null)
                {
                    ResetLcd();
                }
            }
            if (argument == "land")
            {
                _controlling = true;
                _landing = true;
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

            // Determine maximum acceleration that can be provided by the thrusters
            float thrustAlignment = Vector3.Dot(shipDown, gravityDown);
            float minAccel = -gravity;
            float maxAccel = -gravity;
            foreach (IMyThrust thrust in _thrusters)
            {
                // Acceleration is theoretically "Force / Mass", but we have to take into account the fact that the thrust
                // vector might not be aligned with gravity
                float perfectAccel = thrust.MaxEffectiveThrust / mass;
                maxAccel += perfectAccel * thrustAlignment;
            }

            // Determine current altitude
            double altitude;
            if (!_cockpit.TryGetPlanetElevation(MyPlanetElevation.Surface, out altitude))
            {
                Echo("Not in gravity well");
                ResetThrusters();
                return;
            }

            // Determine current distance above sea level (this is needed to measure the slope of the ground)
            double distanceAboveSl;
            _cockpit.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out distanceAboveSl);
            double groundHeight = distanceAboveSl - altitude;
            Vector3D seaLevelPosition = _cockpit.GetPosition() - gravityDown * (float)distanceAboveSl;

            // Determine components of velocity relative to various axes
            float verticalVelocity = Vector3.Dot(velocity, -gravityDown);
            Vector3 horizontalVelocity = velocity + verticalVelocity * gravityDown;
            float forwardVelocity = Vector3.Dot(velocity, groundForward);
            float sidewaysVelocity = Vector3.Dot(velocity, groundRight);

            // Measure the slope of the ground by seeing how fast distance above sea level is changing
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

            // OK, we know everything we need to know to start doing the autopilot thingy
            // Determine desired acceleration
            float accel; // desired vertical acceleration after gravity is taken into account
            bool isUserControlled = false;
            if (Math.Abs(_cockpit.MoveIndicator.Y) > 0.1f)
            {
                float clampedInput = 0.5f + 0.5f * _cockpit.MoveIndicator.Y;
                if (clampedInput < -1) clampedInput = -1f;
                if (clampedInput > 1) clampedInput = 1f;

                accel = minAccel + (maxAccel - minAccel) * clampedInput;
                isUserControlled = true;
            }
            //else { accel = 0f; } if (true) { }
            else if (_landing)
            {
                // Handle when we want to land
                // We want to accelerate such that the current vertical velocity is equal to the constant -LANDING_VELOCITY
                // Note: thrusters will automatically cut off when altitude is less than LANDING_ALTITUDE_THRESHOLD
                accel = (float) (-LANDING_VELOCITY - slopeVelocity);
                if (accel < minAccel) accel = minAccel;
                if (accel > maxAccel) accel = maxAccel;
            }
            else if (altitude < MIN_ALTITUDE)
            {
                if (slopeVelocity > 0)
                {
                    // We are already moving up
                    // Predict at what altitude we will stop if we start stopping now
                    // If we will stop over the minimum altitude, then it's time to stop
                    double t = -slopeVelocity / minAccel;
                    double y = altitude + slopeVelocity * t + 0.5 * minAccel * t * t;

                    accel = (y > MIN_ALTITUDE) ? minAccel : maxAccel;
                }
                else
                {
                    // We are moving down right now -- this is BAD!!!! We might crash!
                    accel = maxAccel;
                }
            }
            else if (altitude > MAX_ALTITUDE)
            {
                if (slopeVelocity < 0)
                {
                    // We are moving down
                    // Predict at what altitude we will stop if we start braking now
                    // If we will stop under the maximum altitude, then it's time to brake
                    double t = -slopeVelocity / maxAccel;
                    double y = altitude + slopeVelocity * t + 0.5 * maxAccel * t * t;

                    accel = (y < MAX_ALTITUDE) ? maxAccel : minAccel;
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
                // Try to accelerate such that y-velocity will be zero
                accel = -verticalVelocity;
                if (accel > maxAccel) accel = maxAccel;
                if (accel < minAccel) accel = minAccel;
            }

            // Set the thrust override to achieve the desired acceleration
            float totalThrust = (accel + gravity) * mass;
            foreach (IMyThrust thrust in _thrusters)
            {
                float efficiency = thrust.MaxEffectiveThrust / thrust.MaxThrust;
                float thrustOverride = totalThrust / efficiency / thrustAlignment / _thrusters.Count;
                thrust.ThrustOverride = Math.Max(thrustOverride, 0.0000000001f);
            }

            // Turn off thrusters if we have landed
            foreach (IMyThrust thrust in _thrusters)
            {
                thrust.Enabled = !(_landing && altitude < LANDING_ALTITUDE_THRESHOLD);
            }

            // Update LCD (if any)
            if (_maybeLcd != null)
            {
                string line1 = (Math.Abs(forwardVelocity) > 5) ? forwardVelocity.ToString("F0") : forwardVelocity.ToString("F1");
                string line2 = (Math.Abs(sidewaysVelocity) > 5) ? sidewaysVelocity.ToString("F0") : sidewaysVelocity.ToString("F1");
                string line3 = "";
                if (stoppingDistance >= 0 &&
                    Math.Abs(forwardAcceleration) > DISPLAY_STOPDIST_THRESHOLD_ACCEL &&
                    Math.Abs(forwardVelocity) > DISPLAY_STOPDIST_THRESHOLD_SPEED)
                {
                    line3 = stoppingDistance.ToString("F0");
                }
                if (_landing)
                {
                    line3 = "LAND";
                }

                _maybeLcd.FontSize = 5.0f;
                _maybeLcd.WriteText(line1 + '\n' + line2 + '\n' + line3);

                // Determine background color of LCD panel
                // If we are user controlled, flash purple
                if (isUserControlled) _maybeLcd.BackgroundColor = (_totalTimeRan % 0.15 < 0.075) ? new Color(25, 0, 60) : new Color(30, 0, 50);
                // If maximum acceleration is too low, warn the user and set the background to yellow or red
                else if (maxAccel < 2f) _maybeLcd.BackgroundColor = new Color(75, 0, 0);
                else if (maxAccel < 4f) _maybeLcd.BackgroundColor = new Color(40, 25, 0);
                // If we are going right too fast, set background to blue
                else if (sidewaysVelocity > 2f) _maybeLcd.BackgroundColor = new Color(0, 0, (int) Math.Min(100, sidewaysVelocity * 10f));
                // If we are going left too fast, set background to green
                else if (sidewaysVelocity < -2f) _maybeLcd.BackgroundColor = new Color(0, (int) Math.Min(100, -sidewaysVelocity * 10f), 0);
                // Otherwise, set background to black (transparent)
                else _maybeLcd.BackgroundColor = Color.Black;
            }
        }

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

            _controlling = false;
            _landing = true;
        }

        bool SubInit()
        {
            IMyBlockGroup group = GridTerminalSystem.GetBlockGroupWithName(_blockGroupName);
            if (group == null)
            {
                Echo($"Block group \"{_blockGroupName}\" not found.");
                return false;
            }

            _thrusters = new List<IMyThrust>();
            group.GetBlocksOfType(_thrusters, x => x.IsSameConstructAs(Me));
            if (_thrusters.Count == 0)
            {
                Echo("No thrusters were found in the group.");
                return false;
            }

            List<IMyCockpit> cockpitList = new List<IMyCockpit>();
            group.GetBlocksOfType(cockpitList);
            if (cockpitList.Count != 1)
            {
                Echo("There must be exactly one cockpit in the group.");
                return false;
            }
            _cockpit = cockpitList[0];

            List<IMyTextPanel> lcdList = new List<IMyTextPanel>();
            group.GetBlocksOfType(lcdList);
            if (lcdList.Count > 0)
            {
                _maybeLcd = lcdList[0];
                if (AUTOMATICALLY_SET_LCD_PARAMS)
                {
                    _maybeLcd.Alignment = TextAlignment.CENTER;
                    _maybeLcd.FontColor = new Color(150, 150, 150);
                    _maybeLcd.Font = "Monospace";
                    _maybeLcd.FontSize = 5.0f;
                    _maybeLcd.ContentType = ContentType.TEXT_AND_IMAGE;
                }
                ResetLcd();
            }

            return true;
        }

        void LoadCustomData()
        {
            string data = Me.CustomData;

            string[] lines = data.Split('\n');
            if (lines[0] == VERSION && lines.Length >= 2)
            {
                // CustomData is valid, load the data
                _blockGroupName = lines[1];
            }
            else
            {
                // CustomData is invalid, load default data
                _blockGroupName = DEFAULT_BLOCK_GROUP;
            }

            SaveCustomData();
        }

        void SaveCustomData()
        {
            Me.CustomData = VERSION + '\n' + _blockGroupName;
        }

        void ResetLcd()
        {
            _maybeLcd.BackgroundColor = Color.Black;
            _maybeLcd.WriteText("");
        }
    }
}
