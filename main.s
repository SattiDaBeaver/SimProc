sub r0, r0
sub r1, r1
sub r2, r2
sub r3, r3

ori 1
add r2, r0

sub r0, r0
ori 24
shift l r0, 3

store r1, r0
add r1, r2
bz 1
j -4

add r0, r2
load r3, r0
add r3, r2
store r3, r0
bz 2
sub r0, r2
j -8

add r0, r2
load r3, r0
add r3, r2
store r3, r0
sub r0, r2
j -8