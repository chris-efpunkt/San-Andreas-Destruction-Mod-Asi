SA DESTRUCTION v3.6.2
=====================
Destructible terrain for GTA San Andreas, in the style of Red Faction's Geo-Mod.

Every explosion in the game - grenades, rockets, satchel charges, exploding
vehicles - carves a real crater into the original world. Thin roofs, walls and
bridge decks can be blown through, craters can be joined into tunnels, and
trees and lamp posts are knocked over. Both the visible world and the collision
are changed, so you can walk, drive and shoot through what you have blown open.


REQUIREMENTS
------------
- GTA San Andreas for PC with gta_sa.exe version 1.0 US.
  (Steam / Rockstar Launcher / 1.01 / 3.0 need a downgrade to 1.0 US.
  With any other exe the mod switches itself off and says so in its log.)
- An ASI loader (e.g. Silent's ASI Loader or Ultimate ASI Loader).
- Optional: Mod Loader.


INSTALLATION
------------
With Mod Loader (recommended):
    Copy SADestruction.asi to   <GTA folder>\modloader\SA Destruction\

Without Mod Loader:
    Copy SADestruction.asi to   <GTA folder>\      (or <GTA folder>\scripts\)

That is all. No game file is replaced or modified.
The "source" folder is not needed for playing.


UNINSTALL
---------
Delete SADestruction.asi (and SADestruction.log next to it). Nothing else was
changed.


KEYS
----
F8   Reset: every crater, hole and tunnel is taken back, destroyed objects and
     knocked-over trees / lamp posts stand again. If you are standing in a
     crater, you are put on top of the restored ground.
F7   Knocking over trees and lamp posts on / off.


GOOD TO KNOW
------------
- Only the outside world is destructible. Interiors stay intact.
- Nothing is permanent: craters are not stored in savegames, and an area you
  leave far behind is whole again when the game loads it the next time.
- The first blast leaves a bowl. Further blasts in the same spot dig deeper,
  and thin things (roofs, walls, bridge decks) are broken through.
- Buildings that are only an empty shell are not opened: there is nothing
  behind the wall to break through to.
- Objects that were hit very often can reach the size limit of a game model;
  further blasts there are then refused (see the log). F8 clears this.
- F8 can take back up to 512 changed objects. Beyond that, later objects only
  become whole again when the game reloads them.
- Other vehicles and pedestrians standing in a crater are not moved by F8.


COMPATIBILITY
-------------
- Works on the original map. Map mods should work as long as they use
  ordinary static world objects, but are not tested.
- Not for SA-MP / MTA: the changes exist only on your own PC.
- Other ASI mods that hook the same spot in the game loop are chained, not
  replaced.


TROUBLESHOOTING
---------------
The mod writes SADestruction.log next to SADestruction.asi.

- No log file at all:      the ASI loader is missing or the .asi is in the
                           wrong folder.
- "HOOK FAILED" in the log: gta_sa.exe is not version 1.0 US.
- No crater at some spot:  the log names the reason for every explosion.

When reporting a problem, please attach SADestruction.log and a screenshot.


CHANGELOG
---------
3.6.2  F8 now resets everything (craters, holes, tunnels, destroyed and
       knocked-over objects) instead of making a test crater.
3.6.1  Fixed: exploding vehicles sometimes produced a mound instead of a
       crater ("inverted crater").
3.6    Break-throughs through roofs, walls and bridge decks, tunnels,
       knocked-over trees and lamp posts.


SOURCE CODE
-----------
The complete source is in the "source" folder (plain C, no runtime library).
Build:  sh build.sh   (clang + lld, LLVM 18; produces SADestruction.asi)
The "test" folder contains the offline test programs that run the mod's logic
against a simulated game memory.
