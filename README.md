# Minecraft-Ballistics-Calculator
Lua code for CC:tweaked to track a moving target in the VS Minecraft mod, requires a specific clockwork modpack.

IMPORTANT NOTE: THE PROGRAMS LISTED ARE DEVELOPED SPECIFICALLY FOR MODDED MINECRAFT AND HAVE NO REAL WORLD USE OR APPLICATION.

File Contents:

mathUtilsCC: a utility file containing math functions useful for vector and rotation transformations, some functions depend on the library provided by the CC:VS mod.
CKBCv2: contains the ballistic calculator, projectile simulation, and tracking PD controller functions, the majority of calculation is done here with the remaining files handling data processing.
tracker_processor: contains all settings and manages data collection and processing, as well as RPM output and program organization.

Description:

This project is designed to provide a robust ballistics calculator which can be used in various other programs by other people, despite this the current CKBC file is designed specifically for the
'tracker_processor' program, which may limit usage outside of the intended environemnt. with that said, I am developing a ballistics calculator library which can be used for a wider range of
scenarios for the CBC mod. This file will be called CKBCG once released and will be listed in this repository.

The program aims to produce RPM commands to control a rotary cannon such that it can track and destroy fast moving targets in the clockwork minecraft mod. It does this through converging on a ballistics solution
via simulation of target engagement. This simulation includes a wide range of factors, currently accounting for the following.

- Cannon mount rotation, position and velocity
- Projectile gravity, drag, and velocity
- Target position, velocity and acceleration

This creates a reasonably accurate model of the games ballistic mechanics allowing for accurate tracking and interception of a target ship even at high range and velocity.
A showcase of this program in-game is linked below:
/link not yet added/

