#!/bin/sh
# Break-through scenes (v3.6): host_test binary, directory with the synthetic models (make_synthetic.py)
# usage: run_break.sh <host_test binary> <dir>
H=$1; D=$2; bad=0
t() { $H $D/$1.bin -break $2 $3 $4 $5 | tail -1 | grep -q " ok$" || { echo "FAILED: $1 $2 x$3"; $H $D/$1.bin -break $2 $3 $4 $5 | head -5; bad=1; }; }
s() { $H $D/$1.bin -stressbox | tail -1 | grep -q " ok$" || { echo "FAILED: $1 stress"; $H $D/$1.bin -stressbox | head -5; bad=1; }; }
t hangar through 1 3        # roof and wall 0.3 m thick over a room
t hangar_thin through 1 3   # inside faces lying on the outside faces
t skinroof hollow 1 0       # roof slab on a building that is only a skin: stays solid
t skin bowl 1 0             # a skin: nothing to break through to
t bridge bowl 1 0           # 1.6 m deck: the first blast leaves a bowl
t bridge through 3 50       # ... the third goes through
t deck through 1 50 1.2     # 0.4 m deck: open out to 1.2 m from the axis
t deck nothing 1 0          # the same with nothing below it (underside of the map): stays solid
t wall through 1 50 1.2     # free-standing wall
t billboard through 2 3     # legs 0.8 m thick: cut through with two blasts
t hangar nc-through 1 3     # the same things drawn without back-face culling
t skin nc-through 1 3
t skin nc-nothing 1 0
t skinroof nc-through 1 3
t bridge nc-through 3 50
t wall nc-through 1 50
t grid40 nc-nothing 1 0     # double-sided terrain with nothing below: stays solid
t fine nc-nothing 1 0
t grid40 bowl 1 0
s bridge                    # many blasts into the same thing: nothing pushed out of it, mesh limit kept
s hangar
s wall
$H $D/grid40.bin -tunnels | tail -1 | grep -q " ok$" || { echo "FAILED: tunnels"; $H $D/grid40.bin -tunnels | head -5; bad=1; }   # two shafts in open ground, joined underground
$H $D/grid40.bin -under | tail -1 | grep -q " ok$" || { echo "FAILED: under"; $H $D/grid40.bin -under | head -5; bad=1; }   # blasts a little under the surface (wrecked cars): bowls, not mounds
for g in grid40 hangar bridge skin; do $H $D/$g.bin -reset | tail -1 | grep -q " ok$" || { echo "FAILED: reset $g"; $H $D/$g.bin -reset | head -5; bad=1; }; done   # F8: craters and holes taken back, the original meshes are in place again, nothing left over
[ $bad = 0 ] && echo "break scenes: all ok"
exit $bad
