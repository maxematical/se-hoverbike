# se-hoverbike

This is a custom script I made for _Space Engineers_, a video game where you build your own spaceships. (It's a little like Minecraft,
the main point of the game is to be creative!) Scripts are little programs you can make that interact with the game's API in order to
automate certain things and customize the behavior of your spaceships.

Despite its name, Space Engineers actually has planets that you can go to and explore! **This script basically changes the controls for your
spaceship so that when you're on a planet, your spaceship will fly close to the ground and act like one of the speeder bikes from Star Wars.**
Then, you just fly by tilting the spaceship with your mouse and trying to maintain control the best you can!! As someone who loves adrenaline,
this is super fun even if it's just a video game and not actually real life danger :>

> **Check out the demo I posted on Reddit!**  
> https://old.reddit.com/r/spaceengineers/comments/i1fjpn/

The script is located in `Program.cs`.
Please note that the code is *a bit* ugly, unfortunately — this is mainly because the scripts in Space Engineers have to be
condensed into one file in order to work, but is also because the game obeys the laws of Newtonian physics,
so there's a LOT of math and edge cases involved. (Any miscalculations could cause your beautiful hoverbike to crash into
the side of a hill!) And there is a lot of functionality in the script; it even includes a fully functioning autopilot that can dock your hoverbike
to your main base.

## how it works

Although Space Engineers has realistic graphics and uses Newtonian physics, the rest of it is not that realistic.
You build spaceships out of blocks (like in Minecraft) and put special *thruster blocks* on it, which generate force in a certain direction.
The script works by detecting how high the spaceship is above the ground (among other things) and basically using that information
to turn on or off the downwards-facing thrusters on your spaceship in order to keep it a desired height above the ground.

You only have to tilt your hoverbike, using your mouse, and due to how physics work you'll begin moving forward, just like
how helicopters fly. The script continually measures the distance from the ground and uses that to predict the slope of the terrain
you're flying over to compensate for any hills or bumps. Additionally, the script can show you your speed and other information on
an in-game "LCD panel" block that you build on your hoverbike.

The last major feature I implemented is an autopilot, which fully takes control over your spaceship so that if you accidentally fall out it will
automatically tilt back at the exact right angle so it comes to a complete stop unharmed. Or, you can input coordinates and the autopilot
will traverse to the exact position you set it to, useful for parking your hoverbike in tight situations. This feature is probably accountable
for at least half the total lines of code. It isn't 100% perfect, because I'm a computer programmer and not an engineer, and it doesn't have
collision avoidance due to limitations with the game's API, but overall it works pretty well!

## installation

If you're interested in trying the script, please see the link I posted on reddit! (I don't like to tie my reddit and steam accounts to my real
identity more than I have to, haha)
